#![feature(lang_items)]
#![no_main]
#![no_std]
#![no_builtins]

extern crate cortex_m;
#[macro_use]
extern crate cortex_m_rt;
extern crate mkl26;

use cortex_m::asm;
use core::fmt::Write;

use mkl26::adc::{Divisor, Resolution, VoltageRef};
use mkl26::mcg::{Clock, Mcg, OscRange};
use mkl26::osc::Osc;
use mkl26::port::PortName;
use mkl26::sim::{Sim, Uart0ClkSrc};
use mkl26::sim::cop::Cop;
use mkl26::tpm::{ClockMode, Prescale, PwmSelect, TimerNum, ChannelSelect, Mode};
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

//Using compact. Pololu is an alternative protocol, but only really useful when you want to
//specify a specific device in a network.
pub enum Motor {
	Off = 0xFF,
	Forward = 0xE1,
	Reverse = 0xE0,
	Current = 0x8F,
	GetError = 0xB5,

	//0xA5 for two bytes, 0x86 just for the high byte.
	GetFeedback = 0x85,
}

entry!(main);

fn main() -> ! {
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

    let port_b = sim.port(PortName::B);
    let port_c = sim.port(PortName::C);
    let port_d = sim.port(PortName::D);

    let mut led = port_c.pin(5).to_gpio();
    led.output();

    /*
    *
    *
    //TODO: Xbee SPI communication and pins.
	*
	*
	*/

    //Uart 1 (pins 0,1 - rx, tx). Pololu communication.
    let rx = port_b.pin(16).to_uart_rx().ok();
    let tx = port_b.pin(17).to_uart_tx().ok();
    unsafe { sim.set_uart0_clksrc(Uart0ClkSrc::McgXLL); }

    //Followed formatting of uart example.
    let mut uart = sim
    	.uart(0, rx, tx, uart::calc_clkdiv(115200, 24_000_000))
    	.unwrap();

    //pin 6
    let _pwm_pin1 = port_d.pin(4).to_pwm().ok();
    //pin 20
    let _pwm_pin2 = port_d.pin(5).to_pwm().ok();

    //Sets register value in sopt2 to source TPM to PLL (hardcoded for now).
    unsafe { sim.set_tpm_clksrc(); }

    let mut tpm0 = sim.tpm(
        TimerNum::TPM0,
        PwmSelect::Up,
        ClockMode::EveryClock,
        Prescale::Div8,
        0x6000,
        _pwm_pin1,
    ).unwrap();

    //Buzzer.
    //Alert light. Will change based on ADC reading while XBee packet "braking" bit is high (logic still needed).
    tpm0.channel(ChannelSelect::Ch4).channel_mode(Mode::EdgePWM, 0b10);
    tpm0.channel(ChannelSelect::Ch5).channel_mode(Mode::EdgePWM, 0b10);
    
    let mut adc = sim
        .adc(
            0,
            26,
            Resolution::Bits16,
            Divisor::Div2,
            VoltageRef::Alternative,
            None,
        ).unwrap();

    if let Ok(calib) = adc.calibrate() {
        write!(uart, "ADC calibrated with {}\r\n", calib).unwrap();
    } else {
        write!(uart, "ADC calibration failed\r\n").unwrap();
    }

    //Waits for an Ok() result.
    //Sequential message required for position setting.
    //0xE1 -> Full forward
    //0x7F -> Magnitude
    //Target = 2048 + 16 x magnitude (FORWARD)
    //Target = 2048 + 16 x magnitude (REVERSE)
    while let Err(_) = uart.write_byte(0xE1) {}
    while let Err(_) = uart.write_byte(0x7F) {}
    asm::delay(10_000_000);

    //Position query.
    while let Err(_) = uart.write_byte(0x86) {}
    asm::delay(10_000_000);

    let mut position = uart.read_byte().unwrap();

    //Motor "turning on", doesn't care about commands yet 
    //as it needs to go to its initial resting position (fully extended for now).
   	while position < 14 {
   		while let Err(_) = uart.write_byte(0x86) {} //Potentially not needed but not verified. See line 180 comment.
   		asm::delay(10_000_000);
   		led.toggle();
   		position = uart.read_byte().unwrap();
   	}


	let mut count = 0;
	let mut current = 0;
	let mut stop = 1;
	let mut battery_monitor = 1;

    loop {

    	//This is actually a part of the calibration (stops when it senses a load). This
    	//is treated for now as a generic "stop" event (going as far back as it can).
    	//NOTE: THIS IS NOT THE FINAL LOGIC FOR STOPPING. However, similar logic is used
    	//where it will set a position to retract to based on the stopping flag being set.
    	if stop == 1 {
   			while let Err(_) = uart.write_byte(0xE0) {}
   			while let Err(_) = uart.write_byte(0x7F) {}
   			asm::delay(10_000_000);

	    	//NOTE: Delay needed because the pololu will continually spit out readings based on
	    	//the previous query. Without the delay the pololu will not have enough time to react
	    	//and will simply keep spitting out its position from when we sent the position query earlier.
	    	while let Err(_) = uart.write_byte(0x8F) {}
	    	asm::delay(10_000_000);

	    	//Count exists to set a threshold to ignore the natural jitter that the actuator
	    	//sometimes exhibits when moving on its down. Allowing it to trigger
	    	//on just one reading makes it too sensitive to any nonlinearities in its
	    	//movement, particularly when changing its direction. 
	    	//This can be more robust with a PIT.
	    	current = uart.read_byte().unwrap();
	    	if current > 5 {
	    		count = count + 1;
	    	}

   			//Battery monitoring off, alerts on.
   			battery_monitor = 0;
		    tpm0.channel(ChannelSelect::Ch4).channel_trigger(0x1E00 as u32);
		    tpm0.channel(ChannelSelect::Ch5).channel_trigger(0x1E00 as u32);

   			//Finished condition.
			if count > 2 {
	    		//Stops actuator and puts it into an "awaiting command" state. 
	    		while let Err(_) = uart.write_byte(0xFF) {}
	    		asm::delay(10_000_000);

	    		//Get current position of when we sensed a load.
	    		while let Err(_) = uart.write_byte(0x86) {}
	    		asm::delay(10_000_000);
	    		position = uart.read_byte().unwrap();

	    		//Battery monitoring on, alerts off.
	   			battery_monitor = 1;
			    tpm0.channel(ChannelSelect::Ch4).channel_trigger(0xFFFF as u32);
			    tpm0.channel(ChannelSelect::Ch5).channel_trigger(0xFFFF as u32);
	    		count = 0;
			}
		}

		// else if (overspeed) {
		// 	stuff
		// }

		// Only goes here if both overspeed and estop aren't set.
		// else if (slider) {
		// 	go to position based on slider reading
		// 	turn off batt monitor
		//  turn on alerts
		// }

		if battery_monitor == 1 {
	        adc.start_conv();
	        while !adc.is_conv_done() {}

	        if adc.read() < 865 {
	        	led.high();

	        	//This was a ghetto trigger that I haven't tested yet. I wanted the system to do a soft reset
	        	//if it detected a voltage below 12V through the divider as a means of demoing the ADC detection logic 
	        	//since the bike has no output screen for now.
	        	// while let Err(_) = uart.write_byte(0xE1) {}
	        	// while let Err(_) = uart.write_byte(0x7F) {}
	        	// asm::delay(100_000_000);
	        	// led.low()
	        }
	        else { led.low(); }
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
