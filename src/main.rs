#![no_std]
#![no_main]

extern crate alloc;

use alloc::format;
use core::mem::MaybeUninit;
use dht_sensor::DhtReading;
use embedded_graphics::geometry::Dimensions;
use embedded_graphics::mono_font::iso_8859_1::{FONT_6X10, FONT_9X18_BOLD};
use embedded_graphics::mono_font::MonoTextStyleBuilder;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::text::{Alignment, Text};

use embedded_svc::ipv4::Interface;
use embedded_svc::wifi::{AccessPointInfo, ClientConfiguration, Configuration, Wifi};

use esp_backtrace::*;
use esp_println::println;
use hal::{clock::ClockControl, peripherals::Peripherals, prelude::*, Delay, IO};

use esp_wifi::wifi::utils::create_network_interface;
use esp_wifi::wifi::{WifiError, WifiStaDevice};
use esp_wifi::wifi_interface::WifiStack;
use esp_wifi::{current_millis, initialize, EspWifiInitFor};

use hal::i2c::I2C;
use hal::prelude::nb::block;
use hal::timer::TimerGroup;
use hal::{systimer::SystemTimer, Rng};
use smoltcp::iface::SocketStorage;
use ssd1306::prelude::*;
use ssd1306::{I2CDisplayInterface, Ssd1306};

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        ALLOCATOR.init(HEAP.as_mut_ptr() as *mut u8, HEAP_SIZE);
    }
}

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");

#[entry]
fn main() -> ! {
    init_heap();
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();

    let clocks = ClockControl::max(system.clock_control).freeze();
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let timer = SystemTimer::new(peripherals.SYSTIMER).alarm0;

    let mut timer0 = timer_group0.timer0;
    let mut delay = Delay::new(&clocks);

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let i2c = I2C::new(
        peripherals.I2C0,
        io.pins.gpio1,
        io.pins.gpio2,
        100u32.kHz(),
        &clocks,
    );

    timer0.start(5u64.secs());

    let inferface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(inferface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    // Specify different text styles
    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();
    let text_style_big = MonoTextStyleBuilder::new()
        .font(&FONT_9X18_BOLD)
        .text_color(BinaryColor::On)
        .build();

    // setup logger
    // To change the log_level change the env section in .cargo/config.toml
    // or remove it and set ESP_LOGLEVEL manually before running cargo run
    // this requires a clean rebuild because of https://github.com/rust-lang/cargo/issues/10358
    esp_println::logger::init_logger_from_env();
    log::info!("Logger is setup");
    println!("Hello world!");

    Text::with_alignment(
        "Temp logger started",
        display.bounding_box().center() + Point::new(0, 0),
        text_style,
        Alignment::Center,
    )
    .draw(&mut display)
    .unwrap();
    display.flush().unwrap();

    display.clear(BinaryColor::Off).unwrap();
    block!(timer0.wait()).unwrap();

    let mut dht_pin = io.pins.gpio8.into_open_drain_output();
    dht_pin.set_high().unwrap();
    delay.delay_ms(1000u16);
    log::info!("Setting up Wifi");
    // Set up Wifi
    let init = initialize(
        EspWifiInitFor::Wifi,
        timer,
        Rng::new(peripherals.RNG),
        system.radio_clock_control,
        &clocks,
    )
    .unwrap();
    let wifi = peripherals.WIFI;
    let mut socket_set_entries: [SocketStorage; 3] = Default::default();
    let (iface, device, mut controller, sockets) =
        create_network_interface(&init, wifi, WifiStaDevice, &mut socket_set_entries).unwrap();
    let wifi_stack = WifiStack::new(iface, device, sockets, current_millis);

    let client_config = Configuration::Client(ClientConfiguration {
        ssid: SSID.try_into().unwrap(),
        password: PASSWORD.try_into().unwrap(),
        ..Default::default()
    });
    let res = Wifi::set_configuration(&mut controller, &client_config);

    let res: Result<(heapless::Vec<AccessPointInfo, 10>, usize), WifiError> = controller.scan_n();
    println!("Connect {:?}", controller.connect());

    // wait to get connected
    println!("Wait to get connected");
    loop {
        let res = controller.is_connected();
        match res {
            Ok(connected) => {
                if connected {
                    break;
                }
            }
            Err(err) => {
                println!("{:?}", err);
                loop {}
            }
        }
    }
    println!("{:?}", controller.is_connected());

    // wait for getting an ip address
    println!("Wait to get an ip address");
    loop {
        wifi_stack.work();

        if wifi_stack.is_iface_up() {
            println!("got ip {:?}", wifi_stack.get_ip_info());
            break;
        }
    }

    loop {
        let measurement = dht_sensor::dht11::Reading::read(&mut delay, &mut dht_pin);
        if let Ok(reading) = measurement {
            let temp = reading.temperature;
            let hum = reading.relative_humidity;
            log::info!("Temp: {} \n Hum: {}", temp, hum);
            Text::with_alignment(
                &format!("Temp: {temp}°C"),
                display.bounding_box().center() + Point::new(0, -10),
                text_style_big,
                Alignment::Center,
            )
            .draw(&mut display)
            .unwrap();
            Text::with_alignment(
                &format!("Hum: {hum}%"),
                display.bounding_box().center() + Point::new(0, 20),
                text_style_big,
                Alignment::Center,
            )
            .draw(&mut display)
            .unwrap();
            Text::with_alignment(
                "Optimal Levels: 23°C, 50-70%",
                display.bounding_box().center() + Point::new(0, 50),
                text_style,
                Alignment::Center,
            );
            display.flush().unwrap();
            display.clear(BinaryColor::Off).unwrap();
            delay.delay_ms(500u16)
        } else {
            log::info!("Error reading DHT11: {:?}", measurement);
        }
    }
}
