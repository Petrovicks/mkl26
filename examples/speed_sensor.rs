#![no_main]
#![no_std]
#![no_builtins]

extern crate cortex_m;
#[macro_use]
extern crate cortex_m_rt;
#[macro_use]
extern crate mkl26;

use cortex_m::interrupt;
use cortex_m::peripheral::{Peripherals, NVIC};

use mkl26::interrupts::Interrupt;
use mkl26::mcg::{Clock, Mcg, OscRange};
use mkl26::osc::Osc;
use mkl26::port::{Gpio, Port, PortName, Pull, InterruptModes};
use mkl26::sim::cop::Cop;
use mkl26::sim::{ClkSrc, Sim};
use mkl26::tpm::{CaptureEdge, ChannelMode, ChannelSelect, Tpm};
use mkl26::tpm::{Channel, ClockMode, Prescale, PwmSelect, TpmNum};

#[cfg_attr(rustfmt, rustfmt_skip)]
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

static mut PORT_C: Option<Port> = None;
static mut PORT_D: Option<Port> = None;
// static mut TPM_PIN: Option<TpmPin<'static>> = None;
static mut TPM0_: Option<Tpm> = None;
static mut TPM0_CHANNEL: Option<Channel<'static, 'static>> = None;

static mut LED_PIN: Option<Gpio<'static>> = None;
static mut OVERFLOW_COUNT: f32 = 0.0;
static mut SPEED: f32 = 0.0;
static mut TIME_INTERVAL: f32 = 0.0;
static mut RPM: f32 = 0.0;
static mut RADIUS: f32 = 3.0;

entry!(main);

fn main() -> ! {
    let mut peripherals = Peripherals::take().unwrap();

    // Enable the crystal oscillator with 10 pf of capacitance
    let osc_token = Osc::new().enable(10);

    // Set our clocks:
    // core/system: 48 Mhz
    // bus: 24 MHz
    // flash: 24 MHz
    let mut sim = Sim::new();
    sim.set_dividers(1, 2);
    let mcg = Mcg::new();
    if let Clock::Fei(mut fei) = mcg.clock() {
        // Our 16 MHz xtal is "very fast", and needs to be divided
        // by 512 to be in the acceptable FLL range.
        let ext_token = fei.enable_xtal(OscRange::VeryHigh, osc_token);
        let fbe = fei.use_external_bypass(512, ext_token);

        // PLL is 24/8 * xtal == 48 MHz
        let pbe = fbe.enable_pll(24, 8, &mut sim);
        pbe.use_pll();
    } else {
        panic!("Somehow the clock wasn't in FEI mode");
    }

    unsafe {
        sim.set_tpm_clksrc(ClkSrc::McgXLL);
        PORT_C = Some(sim.port(PortName::C));
        PORT_C.as_mut().unwrap().pin(1).pull(Pull::Up);

        //Set up Port D pin 6 (Pin 21 on teensy) to a digital pin pulled up with interrupt enabled.
        PORT_D = Some(sim.port(PortName::D));
        let whatIsMyPurpose = Some(PORT_D.as_mut().unwrap().pin(6).to_gpio());
        PORT_D.as_mut().unwrap().pin(6).pull(Pull::Up);
        PORT_D.as_mut().unwrap().pin(6).set_interrupt(InterruptModes::InterruptFallingEdge as u32);

        let tpm_pin = PORT_C.as_mut().unwrap().pin(1).to_tpm().ok();

        LED_PIN = Some(PORT_C.as_mut().unwrap().pin(5).to_gpio());
        LED_PIN.as_mut().unwrap().output();

        //Load 0xFFFF to the counter. Setting a longer period just makes things easier to work
        //with since less interrupts are generated for overflows.
        TPM0_ = Some(
            sim.tpm(
                TpmNum::TPM0,
                PwmSelect::Up,
                ClockMode::EveryClock,
                Prescale::Div8,
                0xFFFF,
            ).unwrap(),
        );

        //Writing to CnV is ignored when mode is set to InputCapture.
        //NOTE: Tpm module writes to cnv before setting InputCapture.
        //I considered this a dummy variable as it is irrelevant for this example.
        TPM0_CHANNEL = Some(
            TPM0_
                .as_mut()
                .unwrap()
                .channel(
                    ChannelSelect::Ch0,
                    ChannelMode::InputCapture(CaptureEdge::Falling),
                    0xFFFF,
                    tpm_pin,
                ).unwrap(),
        );

        //Pin 21 - Port D - Enable pin interrupt (PORTC_D)
        interrupt::free(|_| {
            NVIC::unpend(Interrupt::PORTC_D);
            peripherals.NVIC.enable(Interrupt::PORTC_D);
        });

        //Note usage of _TPM0 above due to conflict here.
        interrupt::free(|_| {
        	NVIC::unpend(Interrupt::TPM0);
        	peripherals.NVIC.enable(Interrupt::TPM0);
        });
    }
    loop {
    	unsafe {
    		//These should be updated with a delay instead of with every loop iteration.
    		//A simple counter or utilization of the PIT would work.
            //60 seconds = 1 minute
            //(1 rotation)/(time in seconds)*(60 times per minute) = rpm
            RPM = 60.0/ (TIME_INTERVAL);

            //2*pi*radius = circumference
            //speed = rpm*(2*pi*radius)*(60 times per hour)-> radius units per hour
            SPEED = RPM * (2.0 * 3.14159 * RADIUS) * 60.0;
        }
    }
}

//Both interrupts use this isr.
interrupt!(TPM0, tpm_isr);
interrupt!(PORTC_D, tpm_isr);

fn tpm_isr() {
    unsafe {
        //Checks if overflow triggered the ISR.
        if TPM0_.as_mut().unwrap().overflow() {
            OVERFLOW_COUNT = OVERFLOW_COUNT + 1.0;
            TIME_INTERVAL = TIME_INTERVAL + OVERFLOW_COUNT * 0xFFFF as f32 / 48000000.0;
            TPM0_.as_mut().unwrap().reset_overflow_flag();
        }

        //Checks if the channel trigger caused the ISR to execute.
        // if TPM0_CHANNEL.as_mut().unwrap().channel_flag() {
        //     TPM0_CHANNEL.as_mut().unwrap().reset_channel_flag();

        //Checks if the GPIO interrupt triggered the ISR.
        if PORT_D.as_mut().unwrap().pin(6).is_interrupt() {
        	PORT_D.as_mut().unwrap().pin(6).reset_interrupt_flag();
            LED_PIN.as_mut().unwrap().toggle();

        	//Time for one full revolution in seconds.
            //(Prescale)*(channel_value+overflowcount*MOD)/(48MHz - CLK speed)
            //This value is used to check if the last reading happened too quickly. This is a
            //very rough debouncing mechanism. If the time interval is shorter than this then
            //the bike is currently going faster than ~30mph, which we will already consider as
            //too fast.
            let dummy_interval = (8.0)
                * (TPM0_CHANNEL.as_mut().unwrap().get_value() as f32
                    + OVERFLOW_COUNT * 0xFFFF as f32)
                / (48000000.0);

            //Time estimation gained by solving the following:
            //(1/(x seconds) * (2pi*(radius in inches) ) * 60/(miles in an inch -> 1.57E-5) ~= 30mph
            //x ~= 30ms, a good "upper threshold". Using 5ms interval for switch debounce works decent.
            if dummy_interval > 0.005 {
	            TIME_INTERVAL = dummy_interval;
	            OVERFLOW_COUNT = 0.0;
            }

            //Rough example of handling if it's below this threshold consistently.
            // else if dummy_interval < 0.025 {
            // 	if counter > 10 {
            // 		ESTOP
            // 	}
            // 	counter = counter + 1;
            // }
        }
    }
}



//TODO: change to use USB_Listen for the panic messages
#[panic_handler]
pub fn rust_begin_panic(_info: &core::panic::PanicInfo) -> ! {
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
