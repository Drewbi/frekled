use esp_hal::{
    clock::ClockControl, delay::Delay, gpio::{self, GpioPin, Io, Level, Output}, peripherals::{Peripherals, SPI2}, prelude::*, spi::{master::Spi, FullDuplexMode, SpiMode}, system::SystemControl
};

const NUM_PIXELS: usize = 256;
const NUM_SLICES: usize = u64::BITS as usize;

const CHUNK_SIZE: usize = u8::BITS as usize;
const NUM_CHUNKS: usize = NUM_PIXELS / CHUNK_SIZE;

pub type Frame = [f64; NUM_PIXELS];
type Packet = [u8; NUM_PIXELS / 8];

pub struct Device<'a> {
    spi: Spi<'a, SPI2, FullDuplexMode>,
    delay: Delay,
    latch: Output<'a, GpioPin<4>>
}

impl<'a> Device<'a> {
    pub fn init() -> Device<'static> {
        let peripherals = Peripherals::take();
        let system = SystemControl::new(peripherals.SYSTEM);
        let clocks = ClockControl::max(system.clock_control).freeze();

        let delay = Delay::new(&clocks);
        let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
        let mut lak = Output::new(io.pins.gpio4, Level::Low);
        let mut en = Output::new(io.pins.gpio1, Level::Low);
    
        lak.set_high();
        en.set_low();
        
        let clk = io.pins.gpio3;
        let da = io.pins.gpio2;

        let spi = Spi::new(
            peripherals.SPI2,
            40u32.MHz(),
            SpiMode::Mode0,
            &clocks,
        ).with_pins(Some(clk), Some(da), gpio::NO_PIN, gpio::NO_PIN);

        Device {
            spi,
            delay,
            latch: lak
        }
    }

    pub fn display(&mut self, frame: &Frame) {

        // let packets: [Packet; 25] = [
        //     [0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0],
        //     [0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0],
        //     [0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0],
        //     [0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0],
        //     [0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0],
        //     [0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0],
        //     [0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0],
        //     [0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0],
        //     [0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0],
        //     [0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0],
        //     [0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0],
        //     [0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0],
        //     [0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0],
        //     [0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0],
        //     [0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0],
        //     [0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0],
        //     [0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0],
        //     [0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0],
        //     [0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0],
        //     [0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0],
        //     [0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0],
        //     [0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0],
        //     [0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0],
        //     [0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0],
        //     [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255],
        // ];

        // for i in 0..packets.len() {
        //     self.latch.set_low();
        //     let _ = Spi::write_bytes(&mut self.spi, &packets[i]);
        //     self.latch.set_high();
        // }

        let mut packets: [Packet; NUM_SLICES] = [[0; NUM_PIXELS / 8]; NUM_SLICES];

        encode_packets(&frame, &mut packets);
        transmit(&packets,  self);
    }
}

fn encode_packets(frame: &Frame, packets: &mut [Packet; NUM_SLICES]) {
    for pixel_index in 0..frame.len() {
        let chunk_num = pixel_index/CHUNK_SIZE;

        // Get its binary encoded value
        let encoded = generate_bit_sequence(frame[pixel_index]) as u64;

        // Set each bit for this pixel in the packets
        for slice_num in 0..NUM_SLICES {
            // Get the corresponding encoded bit for the slice
            // Starting with a bit in the first packet
            let mask = 1 << (NUM_SLICES - 1) - (NUM_SLICES - 1 - slice_num);
            let target_bit = encoded & mask;
            let is_on = target_bit.count_ones() > 0;
            let pixel_mask = if is_on { 
                1 << (pixel_index & (CHUNK_SIZE - 1))
            } else { 0 };
            
            // Use the mask to apply only the relevant bit to the packets
            packets[slice_num][chunk_num] |= pixel_mask;
        }
    }
}

fn generate_bit_sequence(val: f64) -> u64 {
    let num_ones = round(NUM_SLICES as f64 * val);
    let mut sequence: u64 = 0;

    // Early shortcuts
    if num_ones == 0 {
        return 0;
    } else if num_ones == NUM_SLICES {
        return 1 << (NUM_SLICES - 1);
    }

    // Calculate how often to place the 1s
    let interval = NUM_SLICES as f64 / num_ones as f64;
    let mut position = 0.0;

    // Distribute 1s in the sequence
    for _ in 0..num_ones {
        sequence |= 1 << round(position);
        position += interval;
    }

    sequence
}

fn round(val: f64) -> usize {
    (val + 0.5) as usize
}

fn transmit(packets: &[Packet; NUM_SLICES], device: &mut Device) {
    // log::info!("{:?}", packets);
    for i in 0..packets.len() {
        device.latch.set_low();
        let _ = Spi::write_bytes(&mut device.spi, &packets[i]);
        device.latch.set_high();
    }
    device.delay.delay_millis(10);
}
