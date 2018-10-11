use volatile_register::{RW};
use bit_field::BitField;
use port::PwmPin;
use sim::ClockGate;

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

pub enum ChannelSelect {
    Ch0 = 0,
    Ch1 = 1,
    Ch2 = 2,
    Ch3 = 3,
    Ch4 = 4,
    Ch5 = 5,
}

pub struct Tpm<'a> {
    reg: &'static mut TPMRegs,
    _pin: Option<PwmPin<'a>>,
    _gate: ClockGate,
}

use cortex_m::asm;

impl<'a> Tpm<'a> {
    // TODO: pass in pin and double check it matches the selected channel
    pub unsafe fn new(
        name: TimerNum,
        cpwms: PwmSelect,
        cmod: ClockMode,
        clkdivider: Prescale,
        count: u16,
        pin: Option<PwmPin<'a>>,
        gate: ClockGate,
    ) -> Result<Tpm<'a>, ()> {
            //TODO: Use this to assert correct pin usage
            //let pin_info = pin.as_ref().map(|p| (p.port_name(), p.pin()));

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
            //0 - DMA disable
            //1 - Clear TOF flag
            //1 - TOIE
            //0 - up-counting
            let mut sc = 0b0110_0000;
            sc.set_bit(5, cpwms == PwmSelect::UpDown);
            sc.set_bits(0..3, clkdivider as u32);
            reg.sc.write(sc);

            // Enable the TPM.
            reg.sc.modify(|mut sc| {
                sc.set_bits(3..5, cmod as u32);
                sc
            });

            Ok(Tpm {
                reg,
                _pin:pin,
                _gate: gate,
            })
    }

    pub fn channel(&mut self, channel:ChannelSelect) -> TpmChannel {
        TpmChannel {
            _tpm: self,
            _channel: channel,
        }
    }

    pub fn set_period(&mut self, period: u16) {
        unsafe{ self.reg.modu.write(period as u32); }
    }
}

pub struct TpmChannel<'a> {
    _tpm: &'a Tpm<'a>,
    _channel: ChannelSelect,
}

//See Table 31-34 for specific definitions of these modes.
pub enum Mode {
    InputCapture = 0,
    OutputCompare = 1,
    EdgePWM = 2,
    PulseOutputCompare = 3,
    CenterPWM = 4,
}

//TODO: More robust channel options using enums.
impl<'a> TpmChannel<'a> {
    pub fn channel_mode(&mut self, mode:Mode, edge:u8) {
        let mut cnsc = 0;

        cnsc.set_bit(7, true); //Clear CHF
        cnsc.set_bit(6, true); //CHIE = true
        cnsc.set_bits(4..6, mode as u8); //set channel mode
        cnsc.set_bits(2..4, edge as u8); //edge and level selection

        //0b1110_1000 works
        match self._channel {
            ChannelSelect::Ch0 => unsafe { self._tpm.reg.c0sc.write(cnsc as u32); },
            ChannelSelect::Ch1 => unsafe { self._tpm.reg.c1sc.write(cnsc as u32); },
            ChannelSelect::Ch2 => unsafe { self._tpm.reg.c2sc.write(cnsc as u32); },
            ChannelSelect::Ch3 => unsafe { self._tpm.reg.c3sc.write(cnsc as u32); },
            ChannelSelect::Ch4 => unsafe { self._tpm.reg.c4sc.write(cnsc as u32); },
            ChannelSelect::Ch5 => unsafe { self._tpm.reg.c5sc.write(cnsc as u32); },
        }

    }

    pub fn channel_trigger(&mut self, channel_val: u32) {
        match self._channel {
            ChannelSelect::Ch0 => unsafe { self._tpm.reg.c0v.write(channel_val as u32); },
            ChannelSelect::Ch1 => unsafe { self._tpm.reg.c1v.write(channel_val as u32); },
            ChannelSelect::Ch2 => unsafe { self._tpm.reg.c2v.write(channel_val as u32); },
            ChannelSelect::Ch3 => unsafe { self._tpm.reg.c3v.write(channel_val as u32); },
            ChannelSelect::Ch4 => unsafe { self._tpm.reg.c4v.write(channel_val as u32); },
            ChannelSelect::Ch5 => unsafe { self._tpm.reg.c5v.write(channel_val as u32); },
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
