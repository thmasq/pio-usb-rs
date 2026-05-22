use core::cell::RefCell;
use core::marker::PhantomData;
use embassy_rp::dma::Channel;
use embassy_rp::gpio::{Output, Pull, SlewRate};
use embassy_rp::interrupt;
use embassy_rp::interrupt::typelevel::Interrupt;
use embassy_rp::pac::dma::vals::TreqSel;
use embassy_rp::pio::{Instance, StateMachine};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::waitqueue::AtomicWaker;
use embassy_usb::driver::{
    Bus, ControlPipe, Driver, Endpoint, EndpointAddress, EndpointAllocError, EndpointError,
    EndpointIn, EndpointInfo, EndpointOut, EndpointType, Event, Unsupported,
};
use pio::{Instruction, InstructionOperands, MovDestination, MovOperation, MovSource, SideSet};

embassy_rp::bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => PioUsbIrqHandler;
});

// Hardware interrupt handler that must execute in < 600ns
pub struct PioUsbIrqHandler;

impl interrupt::typelevel::Handler<interrupt::typelevel::PIO0_IRQ_0> for PioUsbIrqHandler {
    #[unsafe(link_section = ".data")]
    unsafe fn on_interrupt() {
        USB_STATE.lock(|cell| {
            let mut opt = cell.borrow_mut();
            if let Some(state) = opt.as_mut() {
                let irq_status = unsafe { core::ptr::read_volatile(state.pio_irq_ptr) };

                if (irq_status & (1 << 2)) != 0 {
                    unsafe { core::ptr::write_volatile(state.pio_irq_ptr, 1 << 2) };

                    // Pause RX DMA to read state
                    let mut ctrl = unsafe { core::ptr::read_volatile(state.rx_dma_ctrl_trig_ptr) };
                    ctrl &= !1; // Clear EN bit
                    unsafe { core::ptr::write_volatile(state.rx_dma_ctrl_trig_ptr, ctrl) };

                    // Calculate received length
                    let remaining =
                        unsafe { core::ptr::read_volatile(state.rx_dma_trans_count_ptr) };
                    let len = (128 - remaining) as usize;

                    if len < 2 {
                        // ---- RESET DETECTION ----
                        let sio_gpio_in = 0xd0000004 as *const u32;
                        let initial_pins = unsafe { core::ptr::read_volatile(sio_gpio_in) };

                        if (initial_pins & state.dp_dm_mask) == 0 {
                            cortex_m::asm::delay(400); // Wait 3.2us

                            let final_pins = unsafe { core::ptr::read_volatile(sio_gpio_in) };
                            if (final_pins & state.dp_dm_mask) == 0 {
                                state.bus_event = Some(Event::Reset);
                                state.bus_waker.wake();

                                for ep in state.ep_in_states.iter_mut() {
                                    ep.enabled = false;
                                    ep.status = EndpointStatus::Idle;
                                }
                                for ep in state.ep_out_states.iter_mut() {
                                    ep.enabled = false;
                                    ep.status = EndpointStatus::Idle;
                                }
                            }
                        }
                        // ---- END RESET DETECTION ----
                    } else {
                        // --- NORMAL PACKET PROCESSING ---
                        let pid_raw = state.rx_dma_buf[1];
                        let pid = pid_raw & 0x0F;
                        let pid_inv = (!pid_raw >> 4) & 0x0F;

                        if pid == pid_inv {
                            let fire_dma = |data: *const u8, data_len: usize| unsafe {
                                core::ptr::write_volatile(state.tx_dma_read_addr_ptr, data as u32);
                                core::ptr::write_volatile(
                                    state.tx_dma_trans_count_ptr,
                                    data_len as u32,
                                );
                                core::ptr::write_volatile(
                                    state.tx_dma_trigger_ptr,
                                    state.tx_dma_ctrl_word,
                                );
                            };

                            match pid {
                                0x0D | 0x01 => {
                                    // SETUP or OUT Token
                                    if len >= 4 {
                                        let ep = (state.rx_dma_buf[2] >> 7)
                                            | ((state.rx_dma_buf[3] & 0x07) << 1);
                                        let is_setup = pid == 0x0D;
                                        state.expect_data = Some((ep, is_setup));
                                    }
                                }
                                0x09 => {
                                    // IN Token
                                    if len >= 4 {
                                        let ep = (state.rx_dma_buf[2] >> 7)
                                            | ((state.rx_dma_buf[3] & 0x07) << 1);
                                        let ep_state = &mut state.ep_in_states[ep as usize];

                                        match ep_state.status {
                                            EndpointStatus::TxReady => {
                                                fire_dma(
                                                    ep_state.tx_encoded_buf.as_ptr(),
                                                    ep_state.tx_encoded_len,
                                                );
                                                state.last_tx_ep = Some(ep);
                                            }
                                            EndpointStatus::Stalled => fire_dma(
                                                state.encoded_stall.as_ptr(),
                                                state.encoded_stall.len(),
                                            ),
                                            _ => fire_dma(
                                                state.encoded_nak.as_ptr(),
                                                state.encoded_nak.len(),
                                            ),
                                        }
                                    }
                                }
                                0x03 | 0x0B => {
                                    // DATA0 or DATA1 Packet
                                    if let Some((ep, is_setup)) = state.expect_data.take() {
                                        let ep_state = &mut state.ep_out_states[ep as usize];

                                        if is_setup || ep_state.status == EndpointStatus::RxReady {
                                            if len >= 4 {
                                                let payload_len = len - 4;
                                                ep_state.rx_buf[..payload_len].copy_from_slice(
                                                    &state.rx_dma_buf[2..2 + payload_len],
                                                );
                                                ep_state.rx_len = payload_len;

                                                ep_state.status = if is_setup {
                                                    EndpointStatus::SetupReceived
                                                } else {
                                                    EndpointStatus::Idle
                                                };
                                                ep_state.waker.wake();

                                                fire_dma(
                                                    state.encoded_ack.as_ptr(),
                                                    state.encoded_ack.len(),
                                                );
                                            }
                                        } else if ep_state.status == EndpointStatus::Stalled {
                                            fire_dma(
                                                state.encoded_stall.as_ptr(),
                                                state.encoded_stall.len(),
                                            );
                                        } else {
                                            fire_dma(
                                                state.encoded_nak.as_ptr(),
                                                state.encoded_nak.len(),
                                            );
                                        }
                                    }
                                }
                                0x02 => {
                                    // ACK Handshake
                                    if let Some(ep) = state.last_tx_ep.take() {
                                        let ep_state = &mut state.ep_in_states[ep as usize];
                                        ep_state.status = EndpointStatus::Idle;
                                        ep_state.waker.wake();
                                    }
                                }
                                _ => {} // Ignore other PIDs
                            }
                        }
                    }

                    unsafe {
                        core::ptr::write_volatile(
                            state.rx_dma_write_addr_ptr,
                            state.rx_dma_buf.as_mut_ptr() as u32,
                        );
                        core::ptr::write_volatile(state.rx_dma_trans_count_ptr, 128);
                        core::ptr::write_volatile(
                            state.rx_dma_ctrl_trig_ptr,
                            state.rx_dma_ctrl_word,
                        );
                    }
                }

                if (irq_status & (1 << 3)) != 0 {
                    unsafe { core::ptr::write_volatile(state.pio_irq_ptr, 1 << 3) };
                }
            }
        });
    }
}

pub mod state {
    /// The state machines and DMA are claimed, but PIO programs are not loaded
    /// and pins are not configured.
    pub struct Uninitialized;

    /// The PIO programs are loaded, pin slew rates are set, and the hardware
    /// is ready for USB enumeration.
    pub struct Configured;
}

/// The Hardware container for PIO USB
/// Generic over the specific PIO instance, using a type-erased DMA Channel,
/// and strictly enforcing initialization via the `S` Typestate.
pub struct PioUsbHardware<
    'd,
    T: Instance,
    S,
    const SM_TX: usize,
    const SM_RX: usize,
    const SM_EOP: usize,
> {
    pub tx_sm: StateMachine<'d, T, SM_TX>,
    pub rx_sm: StateMachine<'d, T, SM_RX>,
    pub eop_sm: StateMachine<'d, T, SM_EOP>,
    pub tx_dma: Channel<'d>,
    pub rx_dma: Channel<'d>,
    pub dp_pin: u8,
    pub dm_pin: u8,
    _state: PhantomData<S>,
}

impl<'d, T: Instance, const SM_TX: usize, const SM_RX: usize, const SM_EOP: usize>
    PioUsbHardware<'d, T, state::Uninitialized, SM_TX, SM_RX, SM_EOP>
{
    /// Claim the raw peripherals from Embassy. The hardware is not yet ready to use.
    pub fn new(
        tx_sm: StateMachine<'d, T, SM_TX>,
        rx_sm: StateMachine<'d, T, SM_RX>,
        eop_sm: StateMachine<'d, T, SM_EOP>,
        tx_dma: Channel<'d>,
        rx_dma: Channel<'d>,
    ) -> Self {
        Self {
            tx_sm,
            rx_sm,
            eop_sm,
            tx_dma,
            rx_dma,
            dp_pin: 0,
            dm_pin: 0,
            _state: PhantomData,
        }
    }

    /// Consumes the uninitialized hardware, loads the PIO programs, configures
    /// the D+/D- pins, and returns the strictly-typed Configured hardware.
    pub fn configure(
        mut self,
        common: &mut embassy_rp::pio::Common<'d, T>,
        mut dp: embassy_rp::pio::Pin<'d, T>,
        mut dm: embassy_rp::pio::Pin<'d, T>,
    ) -> PioUsbHardware<'d, T, state::Configured, SM_TX, SM_RX, SM_EOP> {
        let tx_prog = pio::pio_file!("src/usb_tx.pio", select_program("usb_tx_dpdm"));
        let edge_prog = pio::pio_file!("src/usb_rx.pio", select_program("usb_edge_detector"));
        let nrzi_prog = pio::pio_file!("src/usb_rx.pio", select_program("usb_nrzi_decoder"));

        let tx_loaded = common.load_program(&tx_prog.program);
        let edge_loaded = common.load_program(&edge_prog.program);
        let nrzi_loaded = common.load_program(&nrzi_prog.program);

        dp.set_slew_rate(SlewRate::Fast);
        dp.set_schmitt(true);
        dp.set_pull(Pull::None);

        dm.set_slew_rate(SlewRate::Fast);
        dm.set_schmitt(true);
        dm.set_pull(Pull::None);

        let mut tx_cfg = embassy_rp::pio::Config::default();
        tx_cfg.use_program(&tx_loaded, &[&dp, &dm]);
        tx_cfg.clock_divider = 1u8.into();
        self.tx_sm.set_config(&tx_cfg);

        let mut edge_cfg = embassy_rp::pio::Config::default();
        edge_cfg.use_program(&edge_loaded, &[&dp, &dm]);
        edge_cfg.clock_divider = 1u8.into();
        self.rx_sm.set_config(&edge_cfg);

        let mut nrzi_cfg = embassy_rp::pio::Config::default();
        nrzi_cfg.use_program(&nrzi_loaded, &[&dp, &dm]);
        nrzi_cfg.clock_divider = 1u8.into();
        self.eop_sm.set_config(&nrzi_cfg);

        let dp_pin_num = dp.pin();
        let dm_pin_num = dm.pin();

        PioUsbHardware {
            tx_sm: self.tx_sm,
            rx_sm: self.rx_sm,
            eop_sm: self.eop_sm,
            tx_dma: self.tx_dma,
            rx_dma: self.rx_dma,
            dp_pin: dp_pin_num,
            dm_pin: dm_pin_num,

            _state: PhantomData,
        }
    }
}

// Helper trait to expose the underlying PAC struct for IRQ handler
pub trait PioExt {
    fn pio_regs() -> embassy_rp::pac::pio::Pio;
    fn is_pio0() -> bool;
}

impl PioExt for embassy_rp::peripherals::PIO0 {
    fn pio_regs() -> embassy_rp::pac::pio::Pio {
        embassy_rp::pac::PIO0
    }
    fn is_pio0() -> bool {
        true
    }
}

impl PioExt for embassy_rp::peripherals::PIO1 {
    fn pio_regs() -> embassy_rp::pac::pio::Pio {
        embassy_rp::pac::PIO1
    }
    fn is_pio0() -> bool {
        false
    }
}

/// The main Embassy Driver implementation
pub struct PioUsbDriver<
    'd,
    T: Instance + PioExt,
    const SM_TX: usize,
    const SM_RX: usize,
    const SM_EOP: usize,
> {
    hw: PioUsbHardware<'d, T, state::Configured, SM_TX, SM_RX, SM_EOP>,
    ep_in_alloc: u16,
    ep_out_alloc: u16,
    pullup_pin: Output<'d>,
}

impl<'d, T: Instance + PioExt, const SM_TX: usize, const SM_RX: usize, const SM_EOP: usize>
    PioUsbDriver<'d, T, SM_TX, SM_RX, SM_EOP>
{
    pub fn new(
        hw: PioUsbHardware<'d, T, state::Configured, SM_TX, SM_RX, SM_EOP>,
        pullup_pin: Output<'d>,
    ) -> Self {
        Self {
            hw,
            ep_in_alloc: 1,  // EP0 IN is automatically reserved
            ep_out_alloc: 1, // EP0 OUT is automatically reserved
            pullup_pin,
        }
    }
}

#[derive(Copy, Clone, PartialEq)]
pub enum EndpointStatus {
    Idle,          // Will respond with NAK
    RxReady,       // Async task is waiting for data (will receive and ACK)
    TxReady,       // Async task has prepared tx_encoded_buf (will transmit)
    Stalled,       // Will respond with STALL
    SetupReceived, // Used by EP0 ControlPipe
}

pub struct EndpointState {
    pub waker: AtomicWaker,
    // For OUT endpoints: The IRQ decodes data into here, then wakes the async task.
    pub rx_buf: [u8; 64],
    pub rx_len: usize,
    // For IN endpoints: The async task pre-encodes NRZI data into here.
    // Max packet (64) + CRC16 (2) with worst-case bit-stuffing is ~79 bytes, so 128 is safe.
    pub tx_encoded_buf: [u8; 128],
    pub tx_encoded_len: usize,
    pub status: EndpointStatus,
    pub enabled: bool,
}

impl EndpointState {
    pub const fn new() -> Self {
        Self {
            waker: AtomicWaker::new(),
            rx_buf: [0; 64],
            rx_len: 0,
            tx_encoded_buf: [0; 128],
            tx_encoded_len: 0,
            status: EndpointStatus::Idle,
            enabled: false,
        }
    }
}

/// Type-Erased state shared between the Embassy-USB tasks and the hardware IRQ
pub struct UsbIrqState {
    // Synchronous state for the IRQ handler
    pub ep_in_states: [EndpointState; 16],
    pub ep_out_states: [EndpointState; 16],

    // Pre-encoded handshake packets for immediate DMA transmission
    pub encoded_ack: [u8; 8],
    pub encoded_nak: [u8; 8],
    pub encoded_stall: [u8; 8],

    // Raw hardware pointers for the IRQ handler
    pub tx_dma_read_addr_ptr: *mut u32,
    pub tx_dma_trans_count_ptr: *mut u32,
    pub tx_dma_trigger_ptr: *mut u32,
    pub tx_dma_ctrl_word: u32,

    pub rx_dma_buf: [u8; 128],
    pub rx_dma_write_addr_ptr: *mut u32,
    pub rx_dma_trans_count_ptr: *mut u32,
    pub rx_dma_ctrl_trig_ptr: *mut u32,
    pub rx_dma_ctrl_word: u32,

    pub pio_irq_ptr: *mut u32,
    pub pio_fstat_ptr: *mut u32,
    pub dp_dm_mask: u32,

    // Global Bus Events
    pub bus_waker: AtomicWaker,
    pub bus_event: Option<Event>,

    // USB Protocol Tracking
    pub expect_data: Option<(u8, bool)>, // (endpoint, is_setup)
    pub last_tx_ep: Option<u8>,
}

unsafe impl Send for UsbIrqState {}
unsafe impl Sync for UsbIrqState {}

static USB_STATE: Mutex<CriticalSectionRawMutex, RefCell<Option<UsbIrqState>>> =
    Mutex::new(RefCell::new(None));

pub struct PioUsbBus<'d, T: Instance, const SM_TX: usize, const SM_RX: usize, const SM_EOP: usize> {
    _hw: PioUsbHardware<'d, T, state::Configured, SM_TX, SM_RX, SM_EOP>,
    pullup_pin: Output<'d>,
}

pub struct PioEndpointIn<'d> {
    info: EndpointInfo,
    _phantom: core::marker::PhantomData<&'d ()>,
}

pub struct PioEndpointOut<'d> {
    info: EndpointInfo,
    _phantom: core::marker::PhantomData<&'d ()>,
}

pub struct PioControlPipe<'d> {
    _phantom: core::marker::PhantomData<&'d ()>,
}

impl<'d, T: Instance + PioExt, const SM_TX: usize, const SM_RX: usize, const SM_EOP: usize>
    Driver<'d> for PioUsbDriver<'d, T, SM_TX, SM_RX, SM_EOP>
{
    type EndpointOut = PioEndpointOut<'d>;
    type EndpointIn = PioEndpointIn<'d>;
    type ControlPipe = PioControlPipe<'d>;
    type Bus = PioUsbBus<'d, T, SM_TX, SM_RX, SM_EOP>;

    fn alloc_endpoint_in(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointIn, EndpointAllocError> {
        let index = match ep_addr {
            Some(addr) => {
                let idx = addr.index();
                if (self.ep_in_alloc & (1 << idx)) != 0 {
                    return Err(EndpointAllocError);
                }
                idx
            }
            None => {
                let idx = self.ep_in_alloc.trailing_ones() as usize;
                if idx >= 16 {
                    return Err(EndpointAllocError);
                }
                idx
            }
        };
        self.ep_in_alloc |= 1 << index;

        Ok(PioEndpointIn {
            info: EndpointInfo {
                addr: EndpointAddress::from_parts(index, embassy_usb::driver::Direction::In),
                ep_type,
                max_packet_size,
                interval_ms,
            },
            _phantom: PhantomData,
        })
    }

    fn alloc_endpoint_out(
        &mut self,
        ep_type: EndpointType,
        ep_addr: Option<EndpointAddress>,
        max_packet_size: u16,
        interval_ms: u8,
    ) -> Result<Self::EndpointOut, EndpointAllocError> {
        let index = match ep_addr {
            Some(addr) => {
                let idx = addr.index();
                if (self.ep_out_alloc & (1 << idx)) != 0 {
                    return Err(EndpointAllocError);
                }
                idx
            }
            None => {
                let idx = self.ep_out_alloc.trailing_ones() as usize;
                if idx >= 16 {
                    return Err(EndpointAllocError);
                }
                idx
            }
        };
        self.ep_out_alloc |= 1 << index;

        Ok(PioEndpointOut {
            info: EndpointInfo {
                addr: EndpointAddress::from_parts(index, embassy_usb::driver::Direction::Out),
                ep_type,
                max_packet_size,
                interval_ms,
            },
            _phantom: PhantomData,
        })
    }

    fn start(mut self, _control_max_packet_size: u16) -> (Self::Bus, Self::ControlPipe) {
        let fill_osr_instr = Instruction {
            operands: InstructionOperands::MOV {
                destination: MovDestination::OSR,
                op: MovOperation::Invert,
                source: MovSource::NULL,
            },
            delay: 0,
            side_set: None,
        };
        unsafe {
            self.hw
                .eop_sm
                .exec_instr(fill_osr_instr.encode(SideSet::default()))
        };

        let clear_x_instr = Instruction {
            operands: InstructionOperands::MOV {
                destination: MovDestination::X,
                op: MovOperation::None,
                source: MovSource::NULL,
            },
            delay: 0,
            side_set: None,
        };
        unsafe {
            self.hw
                .eop_sm
                .exec_instr(clear_x_instr.encode(SideSet::default()))
        };

        self.hw.rx_sm.set_enable(true);
        self.hw.eop_sm.set_enable(true);
        self.hw.tx_sm.set_enable(true);

        let mut encoded_ack = [0u8; 8];
        let mut encoded_nak = [0u8; 8];
        let mut encoded_stall = [0u8; 8];

        crate::phy::encode_tx_data(&[0x80, 0xD2], &mut encoded_ack);
        crate::phy::encode_tx_data(&[0x80, 0x5A], &mut encoded_nak);
        crate::phy::encode_tx_data(&[0x80, 0x1E], &mut encoded_stall);

        let pio_regs = T::pio_regs();
        let is_pio0 = pio_regs.as_ptr() as u32 == embassy_rp::pac::PIO0.as_ptr() as u32;

        let pio_irq_ptr = pio_regs.irq().as_ptr() as *mut u32;
        let pio_fstat_ptr = pio_regs.fstat().as_ptr() as *mut u32;

        let dma_ch = self.hw.tx_dma.regs();
        dma_ch
            .write_addr()
            .write_value(pio_regs.txf(SM_TX).as_ptr() as u32);

        let tx_dma_read_addr_ptr = dma_ch.read_addr().as_ptr() as *mut u32;
        let tx_dma_trans_count_ptr = dma_ch.trans_count().as_ptr() as *mut u32;
        let tx_dma_trigger_ptr = dma_ch.ctrl_trig().as_ptr() as *mut u32;

        let mut ctrl = embassy_rp::pac::dma::regs::CtrlTrig::default();
        ctrl.set_data_size(embassy_rp::pac::dma::vals::DataSize::SIZE_BYTE);

        let treq = if is_pio0 {
            match SM_TX {
                0 => TreqSel::PIO0_TX0,
                1 => TreqSel::PIO0_TX1,
                2 => TreqSel::PIO0_TX2,
                3 => TreqSel::PIO0_TX3,
                _ => panic!("Invalid PIO State Machine index"),
            }
        } else {
            match SM_TX {
                0 => TreqSel::PIO1_TX0,
                1 => TreqSel::PIO1_TX1,
                2 => TreqSel::PIO1_TX2,
                3 => TreqSel::PIO1_TX3,
                _ => panic!("Invalid PIO State Machine index"),
            }
        };
        ctrl.set_treq_sel(treq);
        ctrl.set_incr_read(true);
        ctrl.set_incr_write(false);
        ctrl.set_en(true);

        let dma_rx_ch = self.hw.rx_dma.regs();
        dma_rx_ch
            .read_addr()
            .write_value(pio_regs.rxf(SM_EOP).as_ptr() as u32);

        let rx_dma_write_addr_ptr = dma_rx_ch.write_addr().as_ptr() as *mut u32;
        let rx_dma_trans_count_ptr = dma_rx_ch.trans_count().as_ptr() as *mut u32;
        let rx_dma_ctrl_trig_ptr = dma_rx_ch.ctrl_trig().as_ptr() as *mut u32;

        let mut rx_ctrl = embassy_rp::pac::dma::regs::CtrlTrig::default();
        rx_ctrl.set_data_size(embassy_rp::pac::dma::vals::DataSize::SIZE_BYTE);

        let rx_treq = if is_pio0 {
            match SM_EOP {
                0 => TreqSel::PIO0_RX0,
                1 => TreqSel::PIO0_RX1,
                2 => TreqSel::PIO0_RX2,
                3 => TreqSel::PIO0_RX3,
                _ => panic!("Invalid PIO State Machine index"),
            }
        } else {
            match SM_EOP {
                0 => TreqSel::PIO1_RX0,
                1 => TreqSel::PIO1_RX1,
                2 => TreqSel::PIO1_RX2,
                3 => TreqSel::PIO1_RX3,
                _ => panic!("Invalid PIO State Machine index"),
            }
        };
        rx_ctrl.set_treq_sel(rx_treq);
        rx_ctrl.set_incr_read(false);
        rx_ctrl.set_incr_write(true);
        rx_ctrl.set_en(true);

        let mut rx_dma_buf = [0u8; 128];
        dma_rx_ch
            .write_addr()
            .write_value(rx_dma_buf.as_mut_ptr() as u32);
        dma_rx_ch.trans_count().write_value(128);
        dma_rx_ch.ctrl_trig().write_value(rx_ctrl);

        let dp_dm_mask = (1 << self.hw.dp_pin) | (1 << self.hw.dm_pin);

        let irq_state = UsbIrqState {
            ep_in_states: core::array::from_fn(|_| EndpointState::new()),
            ep_out_states: core::array::from_fn(|_| EndpointState::new()),

            encoded_ack,
            encoded_nak,
            encoded_stall,

            tx_dma_read_addr_ptr,
            tx_dma_trans_count_ptr,
            tx_dma_trigger_ptr,
            tx_dma_ctrl_word: ctrl.0,

            rx_dma_buf,
            rx_dma_write_addr_ptr,
            rx_dma_trans_count_ptr,
            rx_dma_ctrl_trig_ptr,
            rx_dma_ctrl_word: rx_ctrl.0,

            pio_irq_ptr,
            pio_fstat_ptr,

            dp_dm_mask,
            bus_waker: AtomicWaker::new(),
            bus_event: None,
            expect_data: None,
            last_tx_ep: None,
        };

        USB_STATE.lock(|cell| {
            *cell.borrow_mut() = Some(irq_state);
        });

        unsafe {
            if is_pio0 {
                embassy_rp::interrupt::typelevel::PIO0_IRQ_0::enable();
            } else {
                embassy_rp::interrupt::typelevel::PIO1_IRQ_0::enable();
            }
        }

        (
            PioUsbBus {
                _hw: self.hw,
                pullup_pin: self.pullup_pin,
            },
            PioControlPipe {
                _phantom: PhantomData,
            },
        )
    }
}

impl<'d> Endpoint for PioEndpointIn<'d> {
    fn info(&self) -> &EndpointInfo {
        &self.info
    }

    async fn wait_enabled(&mut self) {
        let ep = self.info.addr.index();
        core::future::poll_fn(|cx| {
            USB_STATE.lock(|cell| {
                let mut opt = cell.borrow_mut();
                let state = opt.as_mut().unwrap();
                let ep_state = &mut state.ep_in_states[ep];
                if ep_state.enabled {
                    core::task::Poll::Ready(())
                } else {
                    ep_state.waker.register(cx.waker());
                    core::task::Poll::Pending
                }
            })
        })
        .await;
    }
}

impl<'d> EndpointIn for PioEndpointIn<'d> {
    async fn write(&mut self, buf: &[u8]) -> Result<(), EndpointError> {
        let ep = self.info.addr.index();
        let mut encoded = [0u8; 128];
        let encoded_len = crate::phy::encode_tx_data(buf, &mut encoded);

        USB_STATE.lock(|cell| {
            let mut opt = cell.borrow_mut();
            let state = opt.as_mut().unwrap();
            let ep_state = &mut state.ep_in_states[ep];
            ep_state.tx_encoded_buf[..encoded_len].copy_from_slice(&encoded[..encoded_len]);
            ep_state.tx_encoded_len = encoded_len;
            ep_state.status = EndpointStatus::TxReady;
        });

        core::future::poll_fn(|cx| {
            USB_STATE.lock(|cell| {
                let mut opt = cell.borrow_mut();
                let state = opt.as_mut().unwrap();
                let ep_state = &mut state.ep_in_states[ep];
                if ep_state.status == EndpointStatus::Idle {
                    core::task::Poll::Ready(Ok(()))
                } else {
                    ep_state.waker.register(cx.waker());
                    core::task::Poll::Pending
                }
            })
        })
        .await
    }
}

impl<'d> Endpoint for PioEndpointOut<'d> {
    fn info(&self) -> &EndpointInfo {
        &self.info
    }

    async fn wait_enabled(&mut self) {
        let ep = self.info.addr.index();
        core::future::poll_fn(|cx| {
            USB_STATE.lock(|cell| {
                let mut opt = cell.borrow_mut();
                let state = opt.as_mut().unwrap();
                let ep_state = &mut state.ep_out_states[ep];
                if ep_state.enabled {
                    core::task::Poll::Ready(())
                } else {
                    ep_state.waker.register(cx.waker());
                    core::task::Poll::Pending
                }
            })
        })
        .await;
    }
}

impl<'d> EndpointOut for PioEndpointOut<'d> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, EndpointError> {
        let ep = self.info.addr.index();

        USB_STATE.lock(|cell| {
            let mut opt = cell.borrow_mut();
            let state = opt.as_mut().unwrap();
            let ep_state = &mut state.ep_out_states[ep];
            ep_state.status = EndpointStatus::RxReady;
            ep_state.rx_len = 0;
        });

        core::future::poll_fn(|cx| {
            USB_STATE.lock(|cell| {
                let mut opt = cell.borrow_mut();
                let state = opt.as_mut().unwrap();
                let ep_state = &mut state.ep_out_states[ep];

                if ep_state.status == EndpointStatus::Idle {
                    let len = ep_state.rx_len;
                    buf[..len].copy_from_slice(&ep_state.rx_buf[..len]);
                    core::task::Poll::Ready(Ok(len))
                } else {
                    ep_state.waker.register(cx.waker());
                    core::task::Poll::Pending
                }
            })
        })
        .await
    }
}

impl<'d> ControlPipe for PioControlPipe<'d> {
    fn max_packet_size(&self) -> usize {
        64
    }

    async fn setup(&mut self) -> [u8; 8] {
        core::future::poll_fn(|cx| {
            USB_STATE.lock(|cell| {
                let mut opt = cell.borrow_mut();
                let state = opt.as_mut().unwrap();
                let ep_state = &mut state.ep_out_states[0];

                if ep_state.status == EndpointStatus::SetupReceived {
                    let mut setup_data = [0u8; 8];
                    setup_data.copy_from_slice(&ep_state.rx_buf[..8]);
                    ep_state.status = EndpointStatus::Idle;
                    core::task::Poll::Ready(setup_data)
                } else {
                    ep_state.waker.register(cx.waker());
                    core::task::Poll::Pending
                }
            })
        })
        .await
    }

    async fn data_out(
        &mut self,
        buf: &mut [u8],
        _first: bool,
        _last: bool,
    ) -> Result<usize, EndpointError> {
        USB_STATE.lock(|cell| {
            let mut opt = cell.borrow_mut();
            let state = opt.as_mut().unwrap();
            state.ep_out_states[0].status = EndpointStatus::RxReady;
        });

        core::future::poll_fn(|cx| {
            USB_STATE.lock(|cell| {
                let mut opt = cell.borrow_mut();
                let state = opt.as_mut().unwrap();
                let ep_state = &mut state.ep_out_states[0];

                if ep_state.status == EndpointStatus::Idle {
                    let len = ep_state.rx_len;
                    buf[..len].copy_from_slice(&ep_state.rx_buf[..len]);
                    core::task::Poll::Ready(Ok(len))
                } else {
                    ep_state.waker.register(cx.waker());
                    core::task::Poll::Pending
                }
            })
        })
        .await
    }

    async fn data_in(
        &mut self,
        data: &[u8],
        _first: bool,
        _last: bool,
    ) -> Result<(), EndpointError> {
        let mut encoded = [0u8; 128];
        let encoded_len = crate::phy::encode_tx_data(data, &mut encoded);

        USB_STATE.lock(|cell| {
            let mut opt = cell.borrow_mut();
            let state = opt.as_mut().unwrap();
            let ep_state = &mut state.ep_in_states[0];
            ep_state.tx_encoded_buf[..encoded_len].copy_from_slice(&encoded[..encoded_len]);
            ep_state.tx_encoded_len = encoded_len;
            ep_state.status = EndpointStatus::TxReady;
        });

        core::future::poll_fn(|cx| {
            USB_STATE.lock(|cell| {
                let mut opt = cell.borrow_mut();
                let state = opt.as_mut().unwrap();
                let ep_state = &mut state.ep_in_states[0];
                if ep_state.status == EndpointStatus::Idle {
                    core::task::Poll::Ready(Ok(()))
                } else {
                    ep_state.waker.register(cx.waker());
                    core::task::Poll::Pending
                }
            })
        })
        .await
    }

    async fn accept(&mut self) {
        let _ = self.data_in(&[], false, true).await;
    }

    async fn reject(&mut self) {
        USB_STATE.lock(|cell| {
            let mut opt = cell.borrow_mut();
            let state = opt.as_mut().unwrap();
            state.ep_in_states[0].status = EndpointStatus::Stalled;
            state.ep_out_states[0].status = EndpointStatus::Stalled;
        });
    }

    async fn accept_set_address(&mut self, _addr: u8) {
        // No-op for now. Hardware filtering isn't strict in software PHY
        // unless an address filter is added to the IRQ handler later.
    }
}

impl<'d, T: Instance, const SM_TX: usize, const SM_RX: usize, const SM_EOP: usize> Bus
    for PioUsbBus<'d, T, SM_TX, SM_RX, SM_EOP>
{
    async fn poll(&mut self) -> Event {
        core::future::poll_fn(|cx| {
            USB_STATE.lock(|cell| {
                let mut opt = cell.borrow_mut();
                let state = opt.as_mut().unwrap();

                if let Some(event) = state.bus_event.take() {
                    core::task::Poll::Ready(event)
                } else {
                    state.bus_waker.register(cx.waker());
                    core::task::Poll::Pending
                }
            })
        })
        .await
    }

    fn endpoint_set_stalled(&mut self, ep_addr: EndpointAddress, stalled: bool) {
        let ep = ep_addr.index();
        let is_in = ep_addr.is_in();

        USB_STATE.lock(|cell| {
            let mut opt = cell.borrow_mut();
            let state = opt.as_mut().unwrap();
            let ep_state = if is_in {
                &mut state.ep_in_states[ep]
            } else {
                &mut state.ep_out_states[ep]
            };

            if stalled {
                ep_state.status = EndpointStatus::Stalled;
            } else {
                ep_state.status = EndpointStatus::Idle;
            }
        });
    }

    fn endpoint_is_stalled(&mut self, ep_addr: EndpointAddress) -> bool {
        let ep = ep_addr.index();
        let is_in = ep_addr.is_in();

        USB_STATE.lock(|cell| {
            let mut opt = cell.borrow_mut();
            let state = opt.as_mut().unwrap();
            let ep_state = if is_in {
                &mut state.ep_in_states[ep]
            } else {
                &mut state.ep_out_states[ep]
            };

            ep_state.status == EndpointStatus::Stalled
        })
    }

    fn endpoint_set_enabled(&mut self, ep_addr: EndpointAddress, enabled: bool) {
        let ep = ep_addr.index();
        let is_in = ep_addr.is_in();

        USB_STATE.lock(|cell| {
            let mut opt = cell.borrow_mut();
            let state = opt.as_mut().unwrap();
            let ep_state = if is_in {
                &mut state.ep_in_states[ep]
            } else {
                &mut state.ep_out_states[ep]
            };

            ep_state.enabled = enabled;
            if enabled {
                ep_state.waker.wake();
            }
        });
    }

    async fn enable(&mut self) {
        self.pullup_pin.set_high();
    }

    async fn disable(&mut self) {
        self.pullup_pin.set_low();
    }

    async fn remote_wakeup(&mut self) -> Result<(), Unsupported> {
        // Not currently supported in this Software PHY
        Err(Unsupported)
    }
}
