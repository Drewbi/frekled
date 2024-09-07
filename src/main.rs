#![no_std]
#![no_main]

use driver::{Frame, NUM_PIXELS};
use esp_backtrace as _;
use esp_hal::{entry, time};

mod driver;

#[entry]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    log::info!("Starting");
    
    driver::init();

    
    loop {
        for i in 1..50 {
            let val = i as f64 / 50.0;
            // log::info!("{}", val);
            let frame: Frame = [val; NUM_PIXELS];
            driver::update(&frame);
        }
        for i in (1..50).rev() {
            let val = i as f64 / 50.0;
            // log::info!("{}", val);
            let frame: Frame = [val; NUM_PIXELS];
            driver::update(&frame);
        }
    }
}
