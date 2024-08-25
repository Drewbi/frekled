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

    log::info!("{:?}", frame);

    let mut device = driver::Device::init();

    loop {
        for i in 0..frame.len() {
            frame[i] = (frame[i] + 0.001) % 1.0;
        }
        device.display(frame);
    }

}
