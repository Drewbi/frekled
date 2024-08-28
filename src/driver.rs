use esp_hal::{
    clock::ClockControl,
    gpio::{self, Io, Level, Output},
    peripherals::{Peripherals, SPI2},
    prelude::*,
    spi::{master::Spi, FullDuplexMode, SpiMode},
    system::SystemControl
};

pub type Frame = [f64; 256];

type Packet = [u8; 32];
const NUM_BITS: usize = 8;
const MAX_PACKETS: usize = 256;

pub struct Device<'a> {
    spi: Spi<'a, SPI2, FullDuplexMode>,
}

impl<'a> Device<'a> {
    pub fn init() -> Device<'static> {
        let peripherals = Peripherals::take();
        let system = SystemControl::new(peripherals.SYSTEM);
        let clocks = ClockControl::max(system.clock_control).freeze();

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
        }
    }

    pub fn display(&mut self, frame: &Frame) {
        let mut packets: [Packet; MAX_PACKETS] = [[0; 32]; MAX_PACKETS];
        encode_packets(&frame, &mut packets);
        transmit(&packets, &mut self.spi);
    }
}

fn encode_packets(frame: &Frame, packets: &mut [Packet; MAX_PACKETS]) {
    // For each pixel
    for i in 0..frame.len() {
        // Get its BCM number
        let encoded = (frame[i] * (MAX_PACKETS - 1) as f64) as u8;
        // Set each bit for this pixel in the packets
        for j in 0..MAX_PACKETS {
            // Find the index of the most significant bit and create a mask
            // log::info!("{}", j);
            let mask = 1 << (NUM_BITS - 1) - ((if j < 255 { j + 1 } else {j}) as u8).leading_zeros() as usize;
            // log::info!("{:b}", mask);
            // log::info!("----");
            // Use the mask to apply only the relevant bit to the packets
            packets[j][i/NUM_BITS] |= encoded & mask;
        }
    }
}

fn transmit(packets: &[Packet; MAX_PACKETS], spi: &mut Spi<SPI2, FullDuplexMode>) {
    for i in 0..packets.len() {
        let _ = Spi::write_bytes(spi, &packets[i]);
    }
}