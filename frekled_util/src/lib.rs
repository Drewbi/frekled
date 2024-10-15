#![no_std]

use bcm::calc_bcm_value;

const CHUNK_SIZE: usize = u8::BITS as usize;
const NUM_CHUNKS: usize = NUM_PIXELS / CHUNK_SIZE;
pub type Timeslice = [u8; NUM_CHUNKS];

pub type PixelBitDepth = u8;
pub const NUM_SLICES: usize = PixelBitDepth::BITS as usize;
pub const NUM_PIXELS: usize = 256;
pub type Frame = [f64; NUM_PIXELS];

mod bcm;

pub fn encode_packets(frame: &Frame, slices: &mut [Timeslice; NUM_SLICES]) {
    let mut out_value: u8;
    let mut bit_index: u8;

    for timeslice_index in 0..u8::BITS {
        for chunk_index in 0..NUM_CHUNKS {
            out_value = 0;
            bit_index = 1;

            for pixel_index in 0..CHUNK_SIZE {
                if calc_bcm_value(frame[chunk_index + pixel_index]) & (1 << timeslice_index) != 0 {
                    out_value |= bit_index;
                }
                bit_index <<= 1;
            }
            slices[timeslice_index as usize][chunk_index] = out_value;
        }
    }
}
