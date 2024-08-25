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
const MAX_PACKETS: usize = 10;

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

    pub fn display(&mut self, frame: Frame) {
        let packets = generate_packets(frame);
        transmit(packets, &mut self.spi);
    }
}

fn generate_packets(frame: Frame) -> [Packet; MAX_PACKETS] {
    let mut packets: [Packet; MAX_PACKETS] = [[0b00000000; 32]; MAX_PACKETS];
    for i in 0..MAX_PACKETS {
        let activation_ratio = i as f64 / MAX_PACKETS as f64;
        for j in 0..256 {
            if frame[j] >= activation_ratio {
                packets[i][j / 8] += 2_u8.pow(j as u32 % 8);
            }
        }
    } 
    packets
}

fn transmit(packets: [Packet; MAX_PACKETS], spi: &mut Spi<SPI2, FullDuplexMode>) {
    for i in 0..packets.len() {
        let _ = Spi::write_bytes(spi, &packets[i]);
    }
}