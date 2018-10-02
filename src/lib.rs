#![feature(const_fn)]
#![feature(untagged_unions)]
#![no_std]

extern crate arraydeque;
extern crate bit_field;
extern crate cortex_m;
extern crate embedded_hal;
extern crate nb;
extern crate volatile_register;

#[macro_use]
pub mod interrupts;

pub mod adc;
pub mod atomic;
pub mod i2c;
pub mod mcg;
pub mod osc;
pub mod port;
pub mod sim;
pub mod spi;
pub mod tpm;
pub mod uart;
