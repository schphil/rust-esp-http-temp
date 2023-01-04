#![no_std]
#![no_main]
#![feature(c_variadic)]
#![feature(const_mut_refs)]

use esp32c3_hal as hal;
use hal::{
    adc::{AdcConfig, AdcPin, Attenuation, ADC, ADC1},
    clock::{ClockControl, CpuClock},
    gpio::{Analog, Bank0GpioRegisterAccess, GpioPin, InputOutputAnalogPinType, IO},
    pac::Peripherals,
    prelude::*,
    system::SystemExt,
    timer::TimerGroup,
    Delay,
    Rng,
    Rtc,
};

use embedded_io::blocking::*;
use embedded_svc::{
    ipv4::Interface,
    wifi::{AccessPointInfo, ClientConfiguration, Configuration, Wifi},
};
use esp_backtrace as _;
use esp_println::{logger::init_logger, println};
use esp_wifi::{
    create_network_stack_storage,
    current_millis,
    initialize,
    network_stack_storage,
    wifi::utils::create_network_interface,
    wifi_interface::{Network, WifiError},
};

use arrform::{arrform, ArrForm};
use core::cell::RefCell;
use critical_section::Mutex;
use serde_json_core::ser;

use smoltcp::wire::Ipv4Address;

// use error::Error;

pub mod error;
pub mod temp_sensor;

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");

#[riscv_rt::entry]
fn main() -> ! {
    println!("Init!");
    init_logger(log::LevelFilter::Info);
    esp_wifi::init_heap();

    let peripherals = Peripherals::take().unwrap();
    let mut system = peripherals.SYSTEM.split();
    // let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
    let clocks = ClockControl::configure(system.clock_control, CpuClock::Clock160MHz).freeze();

    // Disable the RTC and TIMG watchdog timers
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
    let mut wdt1 = timer_group1.wdt;

    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    let mut storage = create_network_stack_storage!(3, 8, 1, 1);
    let ethernet = create_network_interface(network_stack_storage!(storage));
    let mut wifi_interface = esp_wifi::wifi_interface::Wifi::new(ethernet);

    use hal::systimer::SystemTimer;
    let syst = SystemTimer::new(peripherals.SYSTIMER);
    initialize(syst.alarm0, Rng::new(peripherals.RNG), &clocks).unwrap();

    println!("is wifi started: {:?}", wifi_interface.is_started());

    println!("Start Wifi Scan");
    let res: Result<(heapless::Vec<AccessPointInfo, 10>, usize), WifiError> = wifi_interface.scan_n();
    if let Ok((res, _count)) = res {
        for ap in res {
            println!("{:?}", ap);
        }
    }

    println!("Call wifi_connect");
    let client_config = Configuration::Client(ClientConfiguration {
        ssid: SSID.into(),
        password: PASSWORD.into(),
        ..Default::default()
    });
    let res = wifi_interface.set_configuration(&client_config);
    println!("wifi_set_configuration returned {:?}", res);

    println!("{:?}", wifi_interface.get_capabilities());
    println!("wifi_connect {:?}", wifi_interface.connect());

    // wait to get connected
    println!("Wait to get connected");
    loop {
        let res = wifi_interface.is_connected();
        match res {
            Ok(connected) =>
                if connected {
                    break;
                },
            Err(err) => {
                println!("{:?}", err);
                loop {}
            },
        }
    }
    println!("{:?}", wifi_interface.is_connected());

    // wait for getting an ip address
    println!("Wait to get an ip address");
    let network = Network::new(wifi_interface, current_millis);
    loop {
        network.poll_dhcp().unwrap();

        network.work();

        if network.is_iface_up() {
            println!("got ip {:?}", network.get_ip_info());
            break;
        }
    }

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    // Create ADC instances
    let analog = peripherals.APB_SARADC.split();
    let mut adc1_config = AdcConfig::new();
    let mut pin = adc1_config.enable_pin(io.pins.gpio0.into_analog(), Attenuation::Attenuation11dB);
    let mut adc1 = ADC::<ADC1>::adc(&mut system.peripheral_clock_control, analog.adc1, adc1_config).unwrap();
    let pin_mutex = Mutex::new(RefCell::new(pin));
    let adc1_mutex = Mutex::new(RefCell::new(adc1));

    let mut rx_buffer = [0u8; 1536];
    let mut tx_buffer = [0u8; 1536];
    let mut socket = network.get_socket(&mut rx_buffer, &mut tx_buffer);

    socket.listen(8080).unwrap();

    loop {
        socket.work();

        if !socket.is_open() {
            socket.listen(8080).unwrap();
        }

        if socket.is_connected() {
            println!("Connected");

            let mut time_out = false;
            let wait_end = current_millis() + 20 * 1000;
            let mut buffer = [0u8; 1024];
            let mut pos = 0;
            loop {
                if let Ok(len) = socket.read(&mut buffer[pos..]) {
                    let to_print = unsafe { core::str::from_utf8_unchecked(&buffer[..(pos + len)]) };

                    if to_print.contains("\r\n\r\n") {
                        println!("{}", to_print);
                        println!();
                        break;
                    }

                    pos += len;
                } else {
                    break;
                }

                if current_millis() > wait_end {
                    println!("Timeout");
                    time_out = true;
                    break;
                }
            }

            let pin_value = critical_section::with(|cs| {
                let pin = pin_mutex.borrow_ref_mut(cs);
                let adc1 = adc1_mutex.borrow_ref_mut(cs);
                let pin_value = temp_sensor::read_temp_send(adc1, pin);
                println!("PIN0 ADC reading = {}", pin_value);
                pin_value
            });

            let mut data = temp_sensor::Data { temperature: pin_value };
            let data_string: heapless::String<64> = ser::to_string(&mut data).unwrap();

            let af = arrform!(
                2000,
                "HTTP/1.0 200 OK\r\n\r\n\
             <html>\
                 <body>\
                        {}
                 </body>\
             </html>\r\n\
             ",
                data_string
            );

            if !time_out {
                socket.write_all(af.as_bytes()).unwrap();
                socket.flush().unwrap();
            }

            socket.close();

            println!("Done\n");
            println!();
        }
    }
}
