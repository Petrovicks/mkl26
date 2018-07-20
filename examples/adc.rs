#![feature(lang_items)]
#![no_main]
#![no_std]
#![no_builtins]

extern crate cortex_m;
#[macro_use]
extern crate cortex_m_rt;
extern crate mkl26;

use core::fmt::Write;

use mkl26::adc::{Divisor, Resolution, VoltageRef};
use mkl26::mcg::{Clock, Mcg, OscRange};
use mkl26::osc::Osc;
use mkl26::port::PortName;
use mkl26::sim::{Sim, Uart0ClkSrc};
use mkl26::sim::cop::Cop;
use mkl26::uart;

#[link_section = ".flashconfig"]
#[no_mangle]
pub static _FLASHCONFIG: [u8; 16] = [
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xDE, 0xF9, 0xFF, 0xFF
];

pre_init!(disable_wdog);

unsafe fn disable_wdog() {
    Cop::new().init(None);
}

entry!(main);

fn main() -> ! {
    // Enable the crystal oscillator with 10pf of capacitance
    let osc_token = Osc::new().enable(10);

    // Set our clocks:
    // core/system: 48Mhz
    // bus: 24MHz
    // flash: 24MHz
    let mut sim = Sim::new();
    sim.set_dividers(1, 2);
    // We would also set the USB divider here if we wanted to use it.
    let mcg = Mcg::new();
    if let Clock::Fei(mut fei) = mcg.clock() {
        // Our 16MHz xtal is "very fast", and needs to be divided
        // by 512 to be in the acceptable FLL range.
        let ext_token = fei.enable_xtal(OscRange::VeryHigh, osc_token);
        let fbe = fei.use_external_bypass(512, ext_token);

        // PLL is 24/8 * xtal == 48MHz
        let pbe = fbe.enable_pll(24, 8, &mut sim);
        pbe.use_pll();
    } else {
        panic!("Somehow the clock wasn't in FEI mode");
    }

    let port_b = sim.port(PortName::B);
    let port_c = sim.port(PortName::C);

    let mut led = port_c.pin(5).to_gpio();
    led.output();

    let rx = port_b.pin(16).to_uart_rx().ok();
    let tx = port_b.pin(17).to_uart_tx().ok();
    unsafe { sim.set_uart0_clksrc(Uart0ClkSrc::McgXLL); }
    let mut uart = sim.uart(0, rx, tx, uart::calc_clkdiv(115200, 24_000_000)).unwrap();

    // Internal temperature sensor
    let mut adc = sim.adc(
        0,
        26,
        Resolution::Bits16,
        Divisor::Div2,
        VoltageRef::Alternative,
        None
    ).unwrap();

    led.high();

    write!(uart, "ADC test\r\n").unwrap();

    if let Ok(calib) = adc.calibrate() {
        write!(uart, "ADC calibrated with {}\r\n", calib).unwrap();
    } else {
        write!(uart, "ADC calibration failed\r\n").unwrap();
    }

    loop {
        if let Ok(_) = uart.read_byte() {
            led.low();
            adc.start_conv();
            while !adc.is_conv_done() {}
            led.high();
            write!(uart, "ADC value {0}\r\n", adc.read()).unwrap();
        }
    }
}

//TODO: change to use USB_Listen for the panic messages
#[lang = "panic_impl"]
#[no_mangle]
pub extern fn rust_begin_panic(_info: &core::panic::PanicInfo) -> ! {
    // Reset the MCU after we've printed our panic.
    /*
    let aircr = unsafe {
        &mut *(0xE000ED0C as *mut Volatile<u32>)
    };
    aircr.write(0x05FA0004);
    */
    loop {}
}

// the hard fault handler
exception!(HardFault, hard_fault);

fn hard_fault(_ef: &cortex_m_rt::ExceptionFrame) -> ! {
    loop {}
}

// the default exception handler
exception!(*, default_handler);

fn default_handler(_irqn: i16) {}