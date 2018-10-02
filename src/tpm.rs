use volatile_register::{RW};
use bit_field::BitField;
use core::cell::UnsafeCell;
use cortex_m::asm;

//KL26 manual - pp 570-571
const TPM0_ADDR: usize = 0x4003_8000;
const TPM1_ADDR: usize = 0x4003_9000;
const TPM2_ADDR: usize = 0x4003_A000;

#[derive(Clone, Copy)]
pub enum TimerNum {
    TPM0,
    TPM1,
    TPM2
}

#[repr(C,packed)]
struct TPMRegs {
    sc:         RW<u32>,
    cnt:        RW<u32>,
    modu:       RW<u32>,
    c0sc:       RW<u32>,
    c0v:        RW<u32>,
    c1sc:       RW<u32>,
    c1v:        RW<u32>,
    c2sc:       RW<u32>,
    c2v:        RW<u32>,
    c3sc:       RW<u32>,
    c3v:        RW<u32>,
    c4sc:       RW<u32>,
    c4v:        RW<u32>,
    c5sc:       RW<u32>,
    c5v:        RW<u32>,
    _pad0: [u8; 20],
    tpm_status: RW<u32>,
    _pad1: [u8; 48],
    tpm_conf:   RW<u32>,
}

#[derive(PartialEq)]
pub enum PwmSelect {
    Up = 0,
    UpDown = 1,
}

pub enum ClockMode {
    Disabled = 0,
    EveryClock = 1,
    ExternalCLock = 2,
}

pub enum Prescale {
    Div1 = 0b000,
    Div2 = 0b001,
    Div4 = 0b010,
    Div8 = 0b011,
    Div16 = 0b100,
    Div32 = 0b101,
    Div64 = 0b110,
    Div128 = 0b111,
}

pub struct Tpm {
    _reg: &'static mut TPMRegs,
}

impl Tpm {
    // TODO: allow the selection of the channel
    // TODO: pass in pin and double check it matches the selected channel
    pub unsafe fn new(
        name: TimerNum,
        cpwms: PwmSelect,
        cmod: ClockMode,
        clkdivider: Prescale,
        count: u16,
        channel_sel: u8,
        pwm_trigger: u16,
    ) -> Tpm {
            let reg = &mut * match name {
                TimerNum::TPM0 => TPM0_ADDR as *mut TPMRegs,
                TimerNum::TPM1 => TPM1_ADDR as *mut TPMRegs,
                TimerNum::TPM2 => TPM2_ADDR as *mut TPMRegs,
            };

            // clear SC
            reg.sc.write(0);
            // reset CNT
            reg.cnt.write(0);

            // set MOD
            reg.modu.write(count as u32);
            // set SC values
            let mut sc = 0b0110_0000;
            sc.set_bit(5, cpwms == PwmSelect::UpDown);
            sc.set_bits(0..3, clkdivider as u32);
            reg.sc.write(sc);

            // Need to reset channel SC reg before setting it.
            reg.c4sc.write(0);

            // No delay needed as the TPM is still disabled.

            // channel select and mode
            // TODO : Edge selection for PWM output (high-true default: pp575)
            // CH4 - high-true : 01010
            reg.c4sc.write(channel_sel as u32);

            // pwm_trigger sets the value where the PWM pule ends
            reg.c4v.write(pwm_trigger as u32);

            // Enable the TPM.
            reg.sc.modify(|mut sc| {
                sc.set_bits(3..5, cmod as u32);
                sc
            });

            Tpm {
                _reg: reg,
            }
    }
}


//Tests for correct memory addresses.
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tpm0_test() {
        unsafe {
            let reg = & *(TPM0_ADDR as *const TPMRegs);
            assert_eq!(0x4003_8000 as *const RW<u32>, &reg.sc           as *const RW<u32>, "sc");
            assert_eq!(0x4003_8004 as *const RW<u32>, &reg.cnt          as *const RW<u32>, "cnt");
            assert_eq!(0x4003_8008 as *const RW<u32>, &reg.modu         as *const RW<u32>, "modu");
            assert_eq!(0x4003_800C as *const RW<u32>, &reg.c0sc         as *const RW<u32>, "c0sc");
            assert_eq!(0x4003_8010 as *const RW<u32>, &reg.c0v          as *const RW<u32>, "c0v");
            assert_eq!(0x4003_8014 as *const RW<u32>, &reg.c1sc         as *const RW<u32>, "c1sc");
            assert_eq!(0x4003_8018 as *const RW<u32>, &reg.c1v          as *const RW<u32>, "c1v");
            assert_eq!(0x4003_801C as *const RW<u32>, &reg.c2sc         as *const RW<u32>, "c2sc");
            assert_eq!(0x4003_8020 as *const RW<u32>, &reg.c2v          as *const RW<u32>, "c2v");
            assert_eq!(0x4003_8024 as *const RW<u32>, &reg.c3sc         as *const RW<u32>, "c3sc");
            assert_eq!(0x4003_8028 as *const RW<u32>, &reg.c3v          as *const RW<u32>, "c3v");
            assert_eq!(0x4003_802C as *const RW<u32>, &reg.c4sc         as *const RW<u32>, "c4sc");
            assert_eq!(0x4003_8030 as *const RW<u32>, &reg.c4v          as *const RW<u32>, "c4v");
            assert_eq!(0x4003_8034 as *const RW<u32>, &reg.c5sc         as *const RW<u32>, "c5sc");
            assert_eq!(0x4003_8038 as *const RW<u32>, &reg.c5v          as *const RW<u32>, "c5v");
            assert_eq!(0x4003_8050 as *const RW<u32>, &reg.tpm_status   as *const RW<u32>, "tpm_status");
            assert_eq!(0x4003_8084 as *const RW<u32>, &reg.tpm_conf     as *const RW<u32>, "tpm_conf");
        }
    }
}
