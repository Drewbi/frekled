#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    gpio::{self, Io, Level, Output},
    peripherals::{Peripherals, SPI2},
    prelude::*,
    spi::{master::Spi, FullDuplexMode, SpiMode},
    system::SystemControl
};

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);

    let clocks = ClockControl::max(system.clock_control).freeze();
    let delay = Delay::new(&clocks);
    
    esp_println::logger::init_logger_from_env();

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut lak = Output::new(io.pins.gpio4, Level::Low);
    let mut en = Output::new(io.pins.gpio1, Level::Low);

    let clk = io.pins.gpio3;
    let da = io.pins.gpio2;

    let mut spi = Spi::new(
        peripherals.SPI2,
        40u32.MHz(),
        SpiMode::Mode0,
        &clocks,
    ).with_pins(Some(clk), Some(da), gpio::NO_PIN, gpio::NO_PIN);
    log::info!("HELLO");
    lak.set_high();
    en.set_low();
    loop {
        for i in 0..1000 {
            set_brightness(i, &mut spi);
        }
        for i in (0..1000).rev() {
            set_brightness(i, &mut spi);
        }
    }

}

fn set_brightness(value: u32, spi: &mut Spi<SPI2, FullDuplexMode>) {
    let mut on_data: [u8; 32] = [0xff; 32];
    let mut off_data: [u8; 32] = [0x00; 32];

    const DELAY_TIME: u32 = 1;
    const MAX_VALUE: u32 = 500;
    let ratio: f32 = value as f32 / 1000.0;
    let norm_val = (ratio * MAX_VALUE as f32) as u32;

    for _ in 0..DELAY_TIME {
        for _ in 0..norm_val {
            let _ = Spi::write_bytes(spi, &mut on_data);
        }
        for _ in 0..MAX_VALUE-norm_val {
            let _ = Spi::write_bytes(spi, &mut off_data);
        }
    }
}