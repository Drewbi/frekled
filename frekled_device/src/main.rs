#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::entry;
use frekled_util::{Frame, NUM_PIXELS};

mod driver;

#[entry]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    log::info!("Starting");
    
    driver::init();

    // let val = 0.1;
    // let frame: Frame = [val; NUM_PIXELS];
    // driver::update(&frame);

    loop {
        for i in 1..1000 {
            let val = i as f64 / 1000.0;
            let frame: Frame = [val; NUM_PIXELS];
            driver::update(&frame);
        }
        for i in (1..1000).rev() {
            let val = i as f64 / 1000.0;
            let frame: Frame = [val; NUM_PIXELS];
            driver::update(&frame);
        }
    }
}
