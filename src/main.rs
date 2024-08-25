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
    
    let mut frame: Frame = [0.0; 256];

    for i in 0..256 {
        if i < 128 {
            frame[i] = 0.5;
        } else {
            frame[i] = 1.0;
        }
    }

    let mut device = driver::Device::init();

    loop {
        device.display(frame)
    }

}
