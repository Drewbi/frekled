use esp_hal::{
    clock::ClockControl, delay::Delay, gpio::{ self, GpioPin, Io, Level, Output }, interrupt::{ self, Priority }, peripherals::{ Interrupt, Peripherals, SPI2 }, prelude::*, spi::{ master::Spi, FullDuplexMode, SpiMode }, system::SystemControl, time, timer::systimer::{ Alarm, Periodic, SystemTimer }, Blocking
};
use critical_section::Mutex;
use core::cell::RefCell;
use fugit::ExtU32;

const TX_RATE: u32 = 40; // MHz
const INTERRUPT_DELAY: u32 = 1000; // Microseconds

pub const NUM_PIXELS: usize = 256;
const NUM_SLICES: usize = u128::BITS as usize;

const CHUNK_SIZE: usize = u8::BITS as usize;
const NUM_CHUNKS: usize = NUM_PIXELS / CHUNK_SIZE;

pub type Frame = [f64; NUM_PIXELS];
type Packet = [u8; NUM_PIXELS / 8];

static ALARM0: Mutex<RefCell<Option<Alarm<Periodic, Blocking, 0>>>> = Mutex::new(RefCell::new(None));
static DEVICE: Mutex<RefCell<Option<Device>>> = Mutex::new(RefCell::new(None));

static DISPLAY_BUFFER: Mutex<RefCell<Option<[Packet; NUM_SLICES]>>> = Mutex::new(RefCell::new(None));
static UPDATE_BUFFER: Mutex<RefCell<Option<[Packet; NUM_SLICES]>>> = Mutex::new(RefCell::new(None));
static SWAP_BUFFERS: Mutex<RefCell<Option<bool>>> = Mutex::new(RefCell::new(None));

static TEST_PACKETS: [Packet; NUM_SLICES] = [
        [255; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32],  
        [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32],  
        [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32],  
        [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32],  
        [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32],  
        [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32],  
        [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32],  
        [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32],  
        [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32],  
        [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32],  
        [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32],  
        [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32],  
        [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [0; 32], [255; 32]
];

pub struct Device<'a> {
    spi: Spi<'a, SPI2, FullDuplexMode>,
    delay: Delay,
    latch: Output<'a, GpioPin<4>>,
}

pub fn init() {
    critical_section::with(|cs| {
        let peripherals = Peripherals::take();
        let system = SystemControl::new(peripherals.SYSTEM);
        let clocks = ClockControl::max(system.clock_control).freeze();
        let systimer = SystemTimer::new(peripherals.SYSTIMER);
        
        let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
        let mut lak = Output::new(io.pins.gpio4, Level::Low);
        let mut en = Output::new(io.pins.gpio1, Level::Low);
    
        lak.set_high();
        en.set_low();
        
        let clk = io.pins.gpio3;
        let da = io.pins.gpio2;
        
        let spi = Spi::new(
            peripherals.SPI2,
            TX_RATE.MHz(),
            SpiMode::Mode0,
            &clocks,
        ).with_pins(Some(clk), Some(da), gpio::NO_PIN, gpio::NO_PIN);
        
        let delay = Delay::new(&clocks);
        
        let device = Device {
            spi,
            delay,
            latch: lak
        };

        DEVICE.borrow_ref_mut(cs).replace(device);

        DISPLAY_BUFFER.borrow_ref_mut(cs).replace([[0; NUM_PIXELS / 8]; NUM_SLICES]);
        UPDATE_BUFFER.borrow_ref_mut(cs).replace([[0; NUM_PIXELS / 8]; NUM_SLICES]);
        SWAP_BUFFERS.borrow_ref_mut(cs).replace(false);

        let mut alarm0 = systimer.alarm0.into_periodic();
        alarm0.set_interrupt_handler(display_interrupt);
        alarm0.set_period(INTERRUPT_DELAY.micros());
        alarm0.enable_interrupt(true);

        ALARM0.borrow_ref_mut(cs).replace(alarm0);
    });

    interrupt::enable(Interrupt::SYSTIMER_TARGET0, Priority::Priority1).unwrap();
    interrupt::enable(Interrupt::SYSTIMER_TARGET1, Priority::Priority3).unwrap();
    interrupt::enable(Interrupt::SYSTIMER_TARGET2, Priority::Priority3).unwrap();
}

pub fn update(frame: &Frame) {    
    let mut packets: [Packet; NUM_SLICES] = [[0; NUM_PIXELS / 8]; NUM_SLICES];

    encode_packets(&frame, &mut packets);
    
    critical_section::with(|cs| { 
        UPDATE_BUFFER.borrow_ref_mut(cs).replace(packets);
        SWAP_BUFFERS.borrow_ref_mut(cs).replace(true);
    })
}

fn encode_packets(frame: &Frame, packets: &mut [Packet; NUM_SLICES]) {
    // let start_time = time::current_time();
    for pixel_index in 0..frame.len() {
        let chunk_num = pixel_index/CHUNK_SIZE;

        // Get its binary encoded value
        let encoded = generate_bit_sequence(frame[pixel_index]) as u128;

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
    // let end_time = time::current_time();
    // log::info!("Finish encoding - {}", end_time - start_time);
}

fn generate_bit_sequence(val: f64) -> u128 {
    let num_ones = round(NUM_SLICES as f64 * val);
    let mut sequence: u128 = 0;

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
    // let start_time = time::current_time();
    // log::info!("{:?}", packets);
    for i in 0..packets.len() {
        device.latch.set_low();
        let _ = Spi::write_bytes(&mut device.spi, &packets[i]);
        device.latch.set_high();
    }

    // let end_time = time::current_time();
}

#[handler]
fn display_interrupt() {
    // let start_time = time::current_time();
    critical_section::with(|cs| {
        let mut display_buffer_binding = DISPLAY_BUFFER.borrow_ref_mut(cs);
        
        let mut swap_buffers_binding = SWAP_BUFFERS.borrow_ref_mut(cs);
        let swap_buffers = swap_buffers_binding.as_mut().unwrap();
        
        if *swap_buffers {
            // log::info!("SWAPPING");
            let mut update_buffer_binding = UPDATE_BUFFER.borrow_ref_mut(cs);
            let update_buffer = update_buffer_binding.as_mut().unwrap();
            
            let temp_buffer = display_buffer_binding.replace(*update_buffer);
            *update_buffer = temp_buffer.unwrap();
            
            *swap_buffers = false;
        }
        
        let mut device_ref = DEVICE.borrow_ref_mut(cs);
        let device = device_ref.as_mut().unwrap();
        
        let display_buffer = display_buffer_binding.as_mut().unwrap();
        transmit(display_buffer, device);

        ALARM0
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .clear_interrupt()
    });
    // let end_time = time::current_time();
    // log::info!("Finish interrupt - {}", end_time - start_time);
}