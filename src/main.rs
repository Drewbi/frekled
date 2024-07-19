#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    gpio::{Io, Level, Output},
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
};

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);

    let clocks = ClockControl::max(system.clock_control).freeze();
    let delay = Delay::new(&clocks);
    
    esp_println::logger::init_logger_from_env();
    log::info!("Hello world!");

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut lak = Output::new(io.pins.gpio10, Level::Low);
    let mut clk = Output::new(io.pins.gpio12, Level::Low);
    let mut da = Output::new(io.pins.gpio11, Level::Low);
    let mut en = Output::new(io.pins.gpio13, Level::Low);

    lak.set_high();
    da.set_high();
    en.set_low();
    clk.set_low();

    loop {
        clk.set_high();
        clk.set_low();
        delay.delay(100.millis());
    }
}
