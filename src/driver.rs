use core::cell::RefCell;
use critical_section::Mutex;
use esp_hal::{
    clock::ClockControl,
    gpio::{self, GpioPin, Io, Level, Output},
    interrupt::{self, Priority},
    peripherals::{Interrupt, Peripherals, SPI2, TIMG0},
    prelude::*,
    spi::{master::Spi, FullDuplexMode, SpiMode},
    system::SystemControl,
    timer::timg::{Timer, Timer0, TimerGroup},
};

type PixelBitDepth = u8;

const TX_RATE: u32 = 40; // MHz
const INTERRUPT_DELAY: u64 = 10; // Microseconds

pub const NUM_PIXELS: usize = 256;
const NUM_SLICES: usize = PixelBitDepth::BITS as usize;
const CHUNK_SIZE: usize = u8::BITS as usize;
const NUM_CHUNKS: usize = NUM_PIXELS / CHUNK_SIZE;

pub type Frame = [f64; NUM_PIXELS];
type Packet = [u8; NUM_CHUNKS];

static TIMER0: Mutex<RefCell<Option<Timer<Timer0<TIMG0>, esp_hal::Blocking>>>> = Mutex::new(RefCell::new(None));
static DEVICE: Mutex<RefCell<Option<Device>>> = Mutex::new(RefCell::new(None));

static DISPLAY_BUFFER: Mutex<RefCell<Option<[Packet; NUM_SLICES]>>> = Mutex::new(RefCell::new(None));
static UPDATE_BUFFER: Mutex<RefCell<Option<[Packet; NUM_SLICES]>>> = Mutex::new(RefCell::new(None));
static SWAP_BUFFERS: Mutex<RefCell<bool>> = Mutex::new(RefCell::new(false));
static DISPLAY_POSITION: Mutex<RefCell<u32>> = Mutex::new(RefCell::new(0));

pub struct Device<'a> {
    spi: Spi<'a, SPI2, FullDuplexMode>,
    latch: Output<'a, GpioPin<4>>,
}

pub fn init() {
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

    let spi = Spi::new(peripherals.SPI2, TX_RATE.MHz(), SpiMode::Mode0, &clocks).with_pins(Some(clk), Some(da), gpio::NO_PIN, gpio::NO_PIN);

    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let timer0 = timg0.timer0;
    timer0.set_interrupt_handler(display_interrupt);

    interrupt::enable(Interrupt::TG0_T0_LEVEL, Priority::Priority1).unwrap();
    timer0.load_value(INTERRUPT_DELAY.micros()).unwrap();
    timer0.start();
    timer0.listen();

    let device = Device { spi, latch: lak };

    critical_section::with(|cs| {
        DEVICE.borrow_ref_mut(cs).replace(device);

        DISPLAY_BUFFER
            .borrow_ref_mut(cs)
            .replace([[0; NUM_PIXELS / 8]; NUM_SLICES]);
        UPDATE_BUFFER
            .borrow_ref_mut(cs)
            .replace([[0; NUM_PIXELS / 8]; NUM_SLICES]);

        TIMER0.borrow_ref_mut(cs).replace(timer0);
    });
}

pub fn update(frame: &Frame) {
    let mut packets: [Packet; NUM_SLICES] = [[0; NUM_PIXELS / 8]; NUM_SLICES];

    encode_packets(&frame, &mut packets);

    critical_section::with(|cs| {
        UPDATE_BUFFER.borrow_ref_mut(cs).replace(packets);
        let mut swap_buffers = SWAP_BUFFERS.borrow_ref_mut(cs);
        *swap_buffers = true;
    })
}

fn encode_packets(frame: &Frame, packets: &mut [Packet; NUM_SLICES]) {
    // let start_time = time::current_time();
    for pixel_index in 0..frame.len() {
        let chunk_num = pixel_index / CHUNK_SIZE;

        // Get its binary code modulation value
        let encoded = (frame[pixel_index] * PixelBitDepth::MAX as f64) as PixelBitDepth;

        // Set each bit for this pixel in the packets
        for slice_num in 0..NUM_SLICES {
            // Get the corresponding encoded bit for the slice
            let mask: PixelBitDepth = 1 << slice_num;
            let target_bit: PixelBitDepth = encoded & mask;
            let is_on = target_bit > 0;
            let pixel_mask = if is_on {
                1 << (pixel_index & (CHUNK_SIZE - 1))
            } else {
                0
            };

            // Use the mask to apply only the relevant bit to the packets
            packets[slice_num][chunk_num] |= pixel_mask;
        }
    }
    // let end_time = time::current_time();
    // log::info!("Finish encoding - {}", end_time - start_time);
}

fn transmit(packet: Packet, device: &mut Device) {
    device.latch.set_low();
    let _ = Spi::write_bytes(&mut device.spi, &packet);
    device.latch.set_high();
}

#[handler]
fn display_interrupt() {
    // let start_time = time::current_time();
    critical_section::with(|cs| {
        let mut display_buffer_binding = DISPLAY_BUFFER.borrow_ref_mut(cs);

        let mut swap_buffers = SWAP_BUFFERS.borrow_ref_mut(cs);
        let mut display_position = DISPLAY_POSITION.borrow_ref_mut(cs);

        if *swap_buffers {
            *swap_buffers = false;
            *display_position = 0;

            let mut update_buffer = UPDATE_BUFFER.borrow_ref_mut(cs);
            let update_buffer = update_buffer.as_mut().unwrap();

            let temp_buffer = display_buffer_binding.replace(*update_buffer);
            *update_buffer = temp_buffer.unwrap();
        }

        let mut device = DEVICE.borrow_ref_mut(cs);
        let device = device.as_mut().unwrap();

        let display_buffer = display_buffer_binding.as_mut().unwrap();
        let display_slice = display_buffer[*display_position as usize];
        transmit(display_slice, device);

        let mut timer0 = TIMER0.borrow_ref_mut(cs);
        let timer0 = timer0.as_mut().unwrap();

        let position_exponent = (2 as u64).pow(*display_position);
        let delay_time = INTERRUPT_DELAY * position_exponent;

        timer0.clear_interrupt();
        timer0.load_value(delay_time.micros()).unwrap();
        timer0.start();

        *display_position = (1 + *display_position) & PixelBitDepth::BITS - 1 as u32;
    });
    // let end_time = time::current_time();
    // log::info!("Finish interrupt - {}", end_time - start_time);
}
