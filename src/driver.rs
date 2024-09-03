use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    gpio::{ self, GpioPin, Io, Level, Output },
    interrupt::{ self, Priority },
    peripherals::{ Interrupt, Peripherals, SPI2 },
    prelude::*,
    spi::{ master::Spi, FullDuplexMode, SpiMode },
    system::SystemControl,
    time,
    timer::systimer::{ Alarm, Periodic, SystemTimer },
    Blocking
};
use critical_section::Mutex;
use core::cell::RefCell;
use fugit::ExtU32;

const NUM_PIXELS: usize = 256;
const NUM_SLICES: usize = u16::BITS as usize;

const CHUNK_SIZE: usize = u8::BITS as usize;
const NUM_CHUNKS: usize = NUM_PIXELS / CHUNK_SIZE;

pub type Frame = [f64; NUM_PIXELS];
type Packet = [u8; NUM_PIXELS / 8];

static ALARM0: Mutex<RefCell<Option<Alarm<Periodic, Blocking, 0>>>> = Mutex::new(RefCell::new(None));

pub struct Device<'a> {
    spi: Spi<'a, SPI2, FullDuplexMode>,
    delay: Delay,
    latch: Output<'a, GpioPin<4>>,
}

impl<'a> Device<'a> {
    pub fn init() -> Device<'static> {
        let peripherals = Peripherals::take();
        let system = SystemControl::new(peripherals.SYSTEM);
        let clocks = ClockControl::max(system.clock_control).freeze();

        
        let systimer = SystemTimer::new(peripherals.SYSTIMER);
        
        critical_section::with(|cs| {
            let mut alarm0 = systimer.alarm0.into_periodic();
            alarm0.set_interrupt_handler(display_interrupt);
            alarm0.set_period(1u32.secs());
            alarm0.enable_interrupt(true);
    
            ALARM0.borrow_ref_mut(cs).replace(alarm0);
        });
    
        interrupt::enable(Interrupt::SYSTIMER_TARGET0, Priority::Priority1).unwrap();
        interrupt::enable(Interrupt::SYSTIMER_TARGET1, Priority::Priority3).unwrap();
        interrupt::enable(Interrupt::SYSTIMER_TARGET2, Priority::Priority3).unwrap();
        
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
        
        let delay = Delay::new(&clocks);

        Device {
            spi,
            delay,
            latch: lak
        }
    }

    pub fn display(&mut self, frame: &Frame) {

        // let packets: [Packet; 16] = [
        //     [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255],
        //     [0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0],
        //     [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255],
        //     [0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0],
        //     [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255],
        //     [0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0],
        //     [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255],
        //     [0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0],
        //     [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255],
        //     [0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0],
        //     [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255],
        //     [0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0],
        //     [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255],
        //     [0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0],
        //     [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255],
        //     [0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0],
        // ];

        
        // for i in 0..packets.len() {
        //     self.latch.set_low();
        //     let _ = Spi::write_bytes(&mut self.spi, &packets[i]);
        //     self.latch.set_high();
        // }
        // self.delay.delay_millis(10);
        
        let mut packets: [Packet; NUM_SLICES] = [[0; NUM_PIXELS / 8]; NUM_SLICES];

        encode_packets(&frame, &mut packets);
        transmit(&packets,  self);
    }
}

fn encode_packets(frame: &Frame, packets: &mut [Packet; NUM_SLICES]) {
    let start_time = time::current_time();
    log::info!("Start encoding - {}", start_time);
    for pixel_index in 0..frame.len() {
        let chunk_num = pixel_index/CHUNK_SIZE;

        // Get its binary encoded value
        let encoded = generate_bit_sequence(frame[pixel_index]) as u16;

        // Set each bit for this pixel in the packets
        for slice_num in 0..NUM_SLICES {
            // Get the corresponding encoded bit for the slice
            let mask = 1 << (NUM_SLICES - 1) - slice_num;
            let target_bit = encoded & mask;
            let is_on = target_bit > 0;
            let pixel_mask = if is_on { 
                1 << (pixel_index & (CHUNK_SIZE - 1))
            } else { 0 };
            
            // Use the mask to apply only the relevant bit to the packets
            packets[slice_num][chunk_num] |= pixel_mask;
        }
    }
    let end_time = time::current_time();
    log::info!("Finish encoding - {} - diff {}", end_time, end_time - start_time);
}

fn generate_bit_sequence(val: f64) -> u16 {
    let num_ones = round(NUM_SLICES as f64 * val);
    let mut sequence: u16 = 0;

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
    let start_time = time::current_time();
    log::info!("Start transmit - {}", start_time);
    // log::info!("{:?}", packets);
    for i in 0..packets.len() {
        device.latch.set_low();
        let _ = Spi::write_bytes(&mut device.spi, &packets[i]);
        device.latch.set_high();
    }

    let end_time = time::current_time();
    log::info!("Finish transmit - {} - diff {}", end_time, end_time - start_time);
}

#[handler]
fn display_interrupt() {
    log::info!("Triggered handler");
    critical_section::with(|cs| {
        ALARM0
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .clear_interrupt()
    });
}