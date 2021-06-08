#![no_std]
#![no_main]

use panic_semihosting as _;

use cortex_m_rt::entry;
use embedded_hal::digital::v2::InputPin;
use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::usb::{Peripheral, UsbBus, UsbBusType};
use stm32f1xx_hal::stm32::{interrupt, Interrupt};
use stm32f1xx_hal::pac::{Peripherals, NVIC};
use usb_device::prelude::*;
use usb_device::bus::UsbBusAllocator;
use usbd_hid::descriptor::generator_prelude::*;
use usbd_hid::descriptor::KeyboardReport;
use usbd_hid::hid_class::HIDClass;

use crate::keycode::KeyCode;

mod keycode;

static mut USB_BUS: Option<UsbBusAllocator<UsbBusType>> = None;
static mut USB_HID: Option<HIDClass<UsbBusType>> = None;
static mut USB_DEV: Option<UsbDevice<UsbBusType>> = None;

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

    let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);

    let button1_input = gpioa.pa1.into_pull_down_input(&mut gpioa.crl);
    let button2_input = gpioa.pa2.into_pull_down_input(&mut gpioa.crl);
    let button3_input = gpioa.pa3.into_pull_down_input(&mut gpioa.crl);
    let button4_input = gpioa.pa4.into_pull_down_input(&mut gpioa.crl);

    let usb = Peripheral {
        usb: dp.USB,
        pin_dm: gpioa.pa11,
        pin_dp: gpioa.pa12,
    };

    unsafe {
        USB_BUS = Some(UsbBus::new(usb));
    }

    let usb_bus = unsafe { USB_BUS.as_ref().unwrap() };
    let usb_hid = HIDClass::new(usb_bus, KeyboardReport::desc(), 5);
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
    }

    let usb_hid = unsafe { USB_HID.as_ref().unwrap() };

    let rollover_error_report = KeyboardReport {
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

    loop {
        let mut keycodes = [0u8; 6];
        let mut current_keycode = 0;

        if button1_input.is_high().unwrap() {
            keycodes[current_keycode] = KeyCode::E as u8;
            current_keycode += 1;
            if current_keycode >= 6 {
                usb_hid.push_input(&rollover_error_report);
                continue;
            }
        }
        
        if button2_input.is_high().unwrap() {
            keycodes[current_keycode] = KeyCode::R as u8;
            current_keycode += 1;
            if current_keycode >= 6 {
                usb_hid.push_input(&rollover_error_report);
                continue;
            }
        }

        if button3_input.is_high().unwrap() {
            keycodes[current_keycode] = KeyCode::I as u8;
            current_keycode += 1;
            if current_keycode >= 6 {
                usb_hid.push_input(&rollover_error_report);
                continue;
            }
        }

        if button4_input.is_high().unwrap() {
            keycodes[current_keycode] = KeyCode::O as u8;
            current_keycode += 1;
            if current_keycode >= 6 {
                usb_hid.push_input(&rollover_error_report);
                continue;
            }
        }

        let report = KeyboardReport {
            modifier: 0x00,
            leds: 0x00,
            keycodes,
        };

        usb_hid.push_input(&report);
    }
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
fn USB_HP_CAN_TX() {
    poll_usb();
}

#[interrupt]
fn USB_LP_CAN_RX0() {
    poll_usb();
}
