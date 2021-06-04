#![no_std]
#![no_main]

use panic_semihosting as _;

use cortex_m_rt::entry;
use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::usb::{Peripheral, UsbBus, UsbBusType};
use stm32f1xx_hal::stm32::{interrupt, Interrupt};
use stm32f1xx_hal::pac::{Peripherals, NVIC};
use usb_device::prelude::*;
use usb_device::bus::UsbBusAllocator;
use usbd_hid::descriptor::generator_prelude::*;
use usbd_hid::descriptor::KeyboardReport;
use usbd_hid::hid_class::HIDClass;

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

    let gpioa = dp.GPIOA.split(&mut rcc.apb2);

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
            .product("SVDX Controller")
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

    loop {
    }
}

fn poll_usb() {
    let usb_hid = unsafe { USB_HID.as_mut().unwrap() };
    let usb_dev = unsafe { USB_DEV.as_mut().unwrap() };

    usb_dev.poll(&mut [usb_hid]);
}

#[interrupt]
fn USB_HP_CAN_TX() {
    poll_usb();
}

#[interrupt]
fn USB_LP_CAN_RX0() {
    poll_usb();
}
