#![no_std]
#![no_main]

use driver::Frame;
use esp_backtrace as _;
use esp_hal::entry;

mod driver;

#[entry]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    log::info!("Starting");
    
    let val: f64 = 0.4;

    let mut device = driver::Device::init();
    
    let frame: Frame = [val; 256];
    // device.display(&frame);

    loop {

    }
}
