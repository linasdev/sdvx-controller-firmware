#![no_std]
#![no_main]

use panic_reset as _;

use cortex_m_rt::entry;
use embedded_hal::digital::v2::InputPin;
use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::usb::{Peripheral, UsbBus, UsbBusType};
use stm32f1xx_hal::stm32::{interrupt, Interrupt};
use stm32f1xx_hal::pac::{Peripherals, NVIC};
use stm32f1xx_hal::gpio::{ExtiPin, Edge, Input, PullUp};
use stm32f1xx_hal::gpio::gpioa::{PA7, PA8, PA9, PA10};
use usb_device::prelude::*;
use usb_device::bus::UsbBusAllocator;
use usbd_hid::descriptor::generator_prelude::*;
use usbd_hid::descriptor::KeyboardReport;
use usbd_hid::hid_class::HIDClass;

use crate::keycode::KeyCode;

mod keycode;

static MAX_KEYCODE_COUNT: usize = 6;
static ROLLOVER_ERROR_REPORT: KeyboardReport = KeyboardReport {
    modifier: 0x00,
    leds: 0x00,
    keycodes: [
        KeyCode::ErrorRollOver as u8,
        KeyCode::ErrorRollOver as u8,
        KeyCode::ErrorRollOver as u8,
        KeyCode::ErrorRollOver as u8,
        KeyCode::ErrorRollOver as u8,
        KeyCode::ErrorRollOver as u8,
    ],
};

static mut USB_BUS: Option<UsbBusAllocator<UsbBusType>> = None;
static mut USB_HID: Option<HIDClass<UsbBusType>> = None;
static mut USB_DEV: Option<UsbDevice<UsbBusType>> = None;

static mut ROTARY1_CLOCK_INPUT: Option<PA7<Input<PullUp>>> = None;
static mut ROTARY1_DATA_INPUT: Option<PA8<Input<PullUp>>> = None;
static mut ROTARY1_COUNTER: i8 = 0;

static mut ROTARY2_CLOCK_INPUT: Option<PA9<Input<PullUp>>> = None;
static mut ROTARY2_DATA_INPUT: Option<PA10<Input<PullUp>>> = None;
static mut ROTARY2_COUNTER: i8 = 0;

#[entry]
fn main() -> ! {
    let dp = Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();

    let clocks = rcc
        .cfgr
        .use_hse(8.mhz())
        .sysclk(72.mhz())
        .hclk(72.mhz())
        .pclk1(36.mhz())
        .pclk2(72.mhz())
        .freeze(&mut flash.acr);

    assert!(clocks.usbclk_valid());

    let mut afio = dp.AFIO.constrain(&mut rcc.apb2);
    let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);

    let start_input = gpioa.pa0.into_pull_down_input(&mut gpioa.crl);
    let button1_input = gpioa.pa1.into_pull_down_input(&mut gpioa.crl);
    let button2_input = gpioa.pa2.into_pull_down_input(&mut gpioa.crl);
    let button3_input = gpioa.pa3.into_pull_down_input(&mut gpioa.crl);
    let button4_input = gpioa.pa4.into_pull_down_input(&mut gpioa.crl);
    let fx_l_input = gpioa.pa5.into_pull_down_input(&mut gpioa.crl);
    let fx_r_input = gpioa.pa6.into_pull_down_input(&mut gpioa.crl);

    let mut rotary1_clock_input = gpioa.pa7.into_pull_up_input(&mut gpioa.crl);
    let rotary1_data_input = gpioa.pa8.into_pull_up_input(&mut gpioa.crh);
    rotary1_clock_input.make_interrupt_source(&mut afio);
    rotary1_clock_input.trigger_on_edge(&dp.EXTI, Edge::FALLING);
    rotary1_clock_input.enable_interrupt(&dp.EXTI);

    let mut rotary2_clock_input = gpioa.pa9.into_pull_up_input(&mut gpioa.crh);
    rotary2_clock_input.make_interrupt_source(&mut afio);
    rotary2_clock_input.trigger_on_edge(&dp.EXTI, Edge::FALLING);
    rotary2_clock_input.enable_interrupt(&dp.EXTI);

    let rotary2_data_input = gpioa.pa10.into_pull_up_input(&mut gpioa.crh);

    let usb = Peripheral {
        usb: dp.USB,
        pin_dm: gpioa.pa11,
        pin_dp: gpioa.pa12,
    };

    unsafe {
        USB_BUS = Some(UsbBus::new(usb));
        ROTARY1_CLOCK_INPUT = Some(rotary1_clock_input);
        ROTARY1_DATA_INPUT = Some(rotary1_data_input);
        ROTARY2_CLOCK_INPUT = Some(rotary2_clock_input);
        ROTARY2_DATA_INPUT = Some(rotary2_data_input);
    }

    let usb_bus = unsafe { USB_BUS.as_ref().unwrap() };
    let usb_hid = HIDClass::new(usb_bus, KeyboardReport::desc(), 1);
    let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x69af, 0x420b))
            .device_class(0x03) // HID (Human Interface Device)
            .device_sub_class(0x00) // No subclass (can't be used as a Boot Device)
            .device_protocol(0x00) // No protocol (can't be used as a Boot Device)
            .device_release(0x0001) // 00.01
            .self_powered(false)
            .manufacturer("Linas Nikiperavičius <linas@linasdev.com>")
            .product("SDVX Controller")
            .max_power(500) // 500mA
            .build();

    unsafe {
        USB_HID = Some(usb_hid);
        USB_DEV = Some(usb_dev);
    }

    unsafe {
        NVIC::unmask(Interrupt::USB_HP_CAN_TX);
        NVIC::unmask(Interrupt::USB_LP_CAN_RX0);
        NVIC::unmask(Interrupt::EXTI9_5);
    }

    let usb_hid = unsafe { USB_HID.as_ref().unwrap() };

    let rotary1_counter = unsafe { &mut ROTARY1_COUNTER };
    let rotary2_counter = unsafe { &mut ROTARY2_COUNTER };

    let mut last_keycodes = [0u8; 6];

    loop {
        let mut keycodes = [0u8; 6];
        let mut current_keycode = 0;

        if start_input.is_high().unwrap() {
            keycodes[current_keycode] = KeyCode::Kb1 as u8;
            current_keycode += 1;
        }

        if button1_input.is_high().unwrap() {
            keycodes[current_keycode] = KeyCode::E as u8;
            current_keycode += 1;
        }
        
        if button2_input.is_high().unwrap() {
            keycodes[current_keycode] = KeyCode::R as u8;
            current_keycode += 1;
        }

        if button3_input.is_high().unwrap() {
            keycodes[current_keycode] = KeyCode::I as u8;
            current_keycode += 1;
        }

        if button4_input.is_high().unwrap() {
            keycodes[current_keycode] = KeyCode::O as u8;
            current_keycode += 1;
        }

        if fx_l_input.is_high().unwrap() {
            keycodes[current_keycode] = KeyCode::C as u8;
            current_keycode += 1;
        }

        if fx_r_input.is_high().unwrap() {
            if check_and_push_rollover(current_keycode + 1, &usb_hid) {
                continue;
            }

            keycodes[current_keycode] = KeyCode::Comma as u8;
            current_keycode += 1;
        }

        if *rotary1_counter < 0 {
            if check_and_push_rollover(current_keycode + 1, &usb_hid) {
                continue;
            }

            keycodes[current_keycode] = KeyCode::Q as u8;
            current_keycode += 1;
            *rotary1_counter = 0;
        } else if *rotary1_counter > 0 {
            if check_and_push_rollover(current_keycode + 1, &usb_hid) {
                continue;
            }

            keycodes[current_keycode] = KeyCode::W as u8;
            current_keycode += 1;
            *rotary1_counter = 0;
        }

        if *rotary2_counter < 0 {
            if check_and_push_rollover(current_keycode + 1, &usb_hid) {
                continue;
            }

            keycodes[current_keycode] = KeyCode::P as u8;
            *rotary2_counter = 0;
        } else if *rotary2_counter > 0 {
            if check_and_push_rollover(current_keycode + 1, &usb_hid) {
                continue;
            }

            keycodes[current_keycode] = KeyCode::LBracket as u8;
            *rotary2_counter = 0;
        }

        if keycodes == last_keycodes {
            continue;
        }

        let report = KeyboardReport {
            modifier: 0x00,
            leds: 0x00,
            keycodes,
        };

        match usb_hid.push_input(&report) {
            Ok(_) => last_keycodes = keycodes,
            Err(_) => {},
        }
    }
}

fn check_and_push_rollover(keycode_count: usize, usb_hid: &HIDClass<'_, UsbBus<Peripheral>>) -> bool {
    if keycode_count > MAX_KEYCODE_COUNT {
        usb_hid.push_input(&ROLLOVER_ERROR_REPORT);
        return true;
    }

    return false;
}

fn poll_usb() {
    let usb_hid = unsafe { USB_HID.as_mut().unwrap() };
    let usb_dev = unsafe { USB_DEV.as_mut().unwrap() };

    if usb_dev.poll(&mut [usb_hid]) {
        let mut data = [0u8, 64];
        usb_hid.pull_raw_output(&mut data);
    }
}

#[interrupt]
fn EXTI9_5() {
    let rotary1_clock_input = unsafe { ROTARY1_CLOCK_INPUT.as_mut().unwrap() };

    if rotary1_clock_input.check_interrupt() {
        let rotary1_data_input = unsafe { ROTARY1_DATA_INPUT.as_ref().unwrap() };
        let rotary1_counter = unsafe { &mut ROTARY1_COUNTER };
        
        *rotary1_counter += if rotary1_data_input.is_high().unwrap() {
            -1i8
        } else {
            1i8
        };

        rotary1_clock_input.clear_interrupt_pending_bit();
    }

    let rotary2_clock_input = unsafe { ROTARY2_CLOCK_INPUT.as_mut().unwrap() };

    if rotary2_clock_input.check_interrupt() {
        let rotary2_data_input = unsafe { ROTARY2_DATA_INPUT.as_ref().unwrap() };
        let rotary2_counter = unsafe { &mut ROTARY2_COUNTER };
        
        *rotary2_counter += if rotary2_data_input.is_high().unwrap() {
            -1i8
        } else {
            1i8
        };

        rotary2_clock_input.clear_interrupt_pending_bit();
    }
}

#[interrupt]
fn USB_HP_CAN_TX() {
    poll_usb();
}

#[interrupt]
fn USB_LP_CAN_RX0() {
    poll_usb();
}
