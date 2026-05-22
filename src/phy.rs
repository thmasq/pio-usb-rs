//! Software PHY Layer for USB NRZI Encoding and Bit-Stuffing.

/// State mapping for PIO TX instructions.
/// Translates to jumps/pin directions inside the PIO Assembly.
pub const PIO_USB_TX_ENCODED_DATA_SE0: u8 = 0;
pub const PIO_USB_TX_ENCODED_DATA_K: u8 = 1;
pub const PIO_USB_TX_ENCODED_DATA_COMP: u8 = 2;
pub const PIO_USB_TX_ENCODED_DATA_J: u8 = 3;

/// Encodes a raw USB packet into PIO TX instructions.
///
/// Applies NRZI encoding and bit-stuffing (inserting a 0 after six consecutive 1s).
/// Appends the End-of-Packet (EOP) sequence (SE0, COMP) and pads the remaining
/// bits of the last byte with the `K` state to ensure byte alignment.
///
/// # Arguments
/// * `buffer` - The raw USB packet data (e.g., SYNC + PID + DATA + CRC).
/// * `encoded_data` - The buffer to write the 2-bit PIO instructions into. Must be
///   large enough to accommodate the encoded data (max length is `buffer.len() * 2 * 7 / 6 + 2`).
///
/// # Returns
/// The number of bytes written to `encoded_data`.
pub fn encode_tx_data(buffer: &[u8], encoded_data: &mut [u8]) -> usize {
    let mut bit_idx: usize = 0;
    let mut current_state = 1;
    let mut bit_stuffing = 6;

    {
        let mut push_bit = |state: u8| {
            let byte_idx = bit_idx >> 2;

            // Zero-initialize the byte on the first write
            if bit_idx & 3 == 0 {
                encoded_data[byte_idx] = 0;
            }

            encoded_data[byte_idx] <<= 2;
            encoded_data[byte_idx] |= state;
            bit_idx += 1;
        };

        for &data_byte in buffer {
            for b in 0..8 {
                if (data_byte & (1 << b)) != 0 {
                    let state = if current_state == 1 {
                        PIO_USB_TX_ENCODED_DATA_K
                    } else {
                        PIO_USB_TX_ENCODED_DATA_J
                    };

                    push_bit(state);
                    bit_stuffing -= 1;
                } else {
                    current_state ^= 1;

                    let state = if current_state == 1 {
                        PIO_USB_TX_ENCODED_DATA_K
                    } else {
                        PIO_USB_TX_ENCODED_DATA_J
                    };

                    push_bit(state);
                    bit_stuffing = 6;
                }

                if bit_stuffing == 0 {
                    current_state ^= 1;

                    let state = if current_state == 1 {
                        PIO_USB_TX_ENCODED_DATA_K
                    } else {
                        PIO_USB_TX_ENCODED_DATA_J
                    };

                    push_bit(state);
                    bit_stuffing = 6;
                }
            }
        }

        push_bit(PIO_USB_TX_ENCODED_DATA_SE0);
        push_bit(PIO_USB_TX_ENCODED_DATA_COMP);
    }

    while (bit_idx & 3) != 0 {
        let byte_idx = bit_idx >> 2;

        if bit_idx & 3 == 0 {
            encoded_data[byte_idx] = 0;
        }

        encoded_data[byte_idx] <<= 2;
        encoded_data[byte_idx] |= PIO_USB_TX_ENCODED_DATA_K;
        bit_idx += 1;
    }

    bit_idx >> 2
}
