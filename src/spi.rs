//! FIFO is 8 bytes big on SPI1. No FIFO on SPI0.

use core::marker::PhantomData;

use bit_field::BitField;
use embedded_hal::spi::{FullDuplex, Phase, Polarity};
use nb::{self, Error};
use volatile_register::{RO, RW};

use port::{SpiCs, SpiMiso, SpiMosi, SpiSck};
use sim::ClockGate;

const SPI0_ADDR: usize = 0x4007_6000;
const SPI1_ADDR: usize = 0x4007_7000;

#[repr(C, packed)]
struct SpiRegs {
    s:     RO<u8>,
    br:    RW<u8>,
    c2:    RW<u8>,
    c1:    RW<u8>,
    ml:    RW<u8>,
    mh:    RW<u8>,
    dl:    RW<u8>,
    dh:    RW<u8>,
    _pad0: [u8; 2],
    ci:    RW<u8>,
    c3:    RW<u8>,
}

pub struct SpiMaster<'a, 'b, 'c, 'd, W: Word> {
    reg:     &'static mut SpiRegs,
    _mosi:   Option<SpiMosi<'a>>,
    _miso:   Option<SpiMiso<'b>>,
    _sck:    SpiSck<'c>,
    _cs:     Option<SpiCs<'d>>,
    _gate:   ClockGate,
    _char:   PhantomData<W>,
    op_mode: OpMode,
}

#[cfg(feature = "spi-slave")]
pub struct SpiSlave<'a, 'b, 'c, 'd, W: Word> {
    _reg:   &'static mut SpiRegs,
    _mosi: Option<SpiMosi<'a>>,
    _miso: Option<SpiMiso<'b>>,
    _sck:  SpiSck<'c>,
    _cs:   SpiCs<'d>,
    _gate: ClockGate,
    _char: PhantomData<W>,
}

pub enum Prescale {
    Div1 = 0,
    Div2 = 1,
    Div3 = 2,
    Div4 = 3,
    Div5 = 4,
    Div6 = 5,
    Div7 = 6,
    Div8 = 7,
}

pub enum Divisor {
    Div2 = 0,
    Div4 = 1,
    Div8 = 2,
    Div16 = 3,
    Div32 = 4,
    Div64 = 5,
    Div128 = 6,
    Div256 = 7,
    Div512 = 8,
}

#[derive(PartialEq)]
pub enum OpMode {
    /// Immediate mode - All transmits and receives are blocking. Doesn't use ISR.
    IMM,
    #[cfg(feature = "spi-isr")]
    /// Interrupt mode - Transmits and receives are non-blocking. Uses ISR.
    ISR,
    #[cfg(feature = "spi-dma")]
    /// DMA mode - Like interrupt mode, but it uses DMA to do the magic
    DMA,
}

/// Marker trait for word sizes supported by the SPI peripheral
///
/// An implementation detail.
pub trait Word {
    fn size() -> usize;
}

impl Word for u8 {
    fn size() -> usize {
        8
    }
}

impl Word for u16 {
    fn size() -> usize {
        16
    }
}

// SPI0 uses the bus clock
// SPI1 uses the system clock
//pub fn calc_clkdiv(baud: u32, clock_freq: u32) -> (u3, u4) {
//    let clkdiv = (clock_freq / baud) as u8;
//    clkdiv = (sppr + 1) x 2^(spr + 1);
//}

// TODO: single wire mode
// TODO: low power mode
// TODO: DMA Rx
// TODO: DMA Tx
// TODO: MSb/LSb
// TODO: hardware match
// TODO: mode fault feature
impl<'a, 'b, 'c, 'd, W: Word> SpiMaster<'a, 'b, 'c, 'd, W> {
    pub(crate) unsafe fn new(
        mosi: Option<SpiMosi<'a>>,
        miso: Option<SpiMiso<'b>>,
        sck: SpiSck<'c>,
        cs: Option<SpiCs<'d>>,
        clkdiv: (Prescale, Divisor),
        op_mode: OpMode,
        polarity: Polarity,
        phase: Phase,
        fifo: bool,
        gate: ClockGate,
    ) -> Result<SpiMaster<'a, 'b, 'c, 'd, W>, ()> {
        let bus = sck.bus();

        if let Some(mosi) = mosi.as_ref() {
            if mosi.bus() != bus {
                return Err(());
            }
        }
        if let Some(miso) = miso.as_ref() {
            if miso.bus() != bus {
                return Err(());
            }
        }
        if let Some(cs) = cs.as_ref() {
            if cs.bus() != bus {
                return Err(());
            }
        }

        let reg = match bus {
            0 => &mut *(SPI0_ADDR as *mut SpiRegs),
            1 => &mut *(SPI1_ADDR as *mut SpiRegs),
            _ => unreachable!(),
        };

        let mut c1 = 0;
        c1.set_bit(7, op_mode != OpMode::IMM); // SPRF and MODF *or* Rx FIFO Full interrupt enable
        c1.set_bit(6, true); // SPI enable
        c1.set_bit(5, op_mode != OpMode::IMM); // SPTEF or Tx FIFO empty interrupt enable
        c1.set_bit(4, true); // slave/master :: 0/1
        c1.set_bit(3, polarity == Polarity::IdleHigh); // clock polarity idle low/high :: 0/1
        c1.set_bit(2, phase == Phase::CaptureOnSecondTransition); // clock phase middle/start :: 0/1
        c1.set_bit(1, true); // Slave Select Output Enable
        c1.set_bit(0, false); // most/least significant bit :: 0/1
        reg.c1.write(c1);

        let mut c2 = 0;
        c2.set_bit(7, false); // match enable
        c2.set_bit(6, W::size() == 16); // 8/16 bits :: 0/1
        #[cfg(feature = "spi-dma")]
        c2.set_bit(5, op_mode == OpMode::DMA); // Tx DMA enable
        c2.set_bit(4, cs.is_some()); // Master Mode-Fault enable
        c2.set_bit(3, false); // Bidirectional mode input/output select
        #[cfg(feature = "spi-dma")]
        c2.set_bit(2, op_mode == OpMode::DMA); // Rx DMA enable
        c2.set_bit(1, false); // SPI stop in wait mode
        c2.set_bit(0, false); // SPI Bidirectional (single wire) mode enable
        reg.c2.write(c2);

        if bus == 1 {
            let mut c3 = 0;
            c3.set_bit(3, true); // FIFO interrpt flag clearing method
            c3.set_bit(2, fifo); // Tx FIFO nearly empty interrupt enable
            c3.set_bit(1, fifo); // Rx FIFO nearly full interrupt enable
            c3.set_bit(0, fifo); // FIFO mode enable
            reg.c3.write(c3);
        }

        // BR
        let mut br: u8 = 0;
        br.set_bits(4..7, clkdiv.0 as u8);
        br.set_bits(0..4, clkdiv.1 as u8);
        reg.br.write(br);

        // MH/ML
        // set if hardware match enabled

        Ok(SpiMaster {
            reg,
            _mosi: mosi,
            _miso: miso,
            _sck: sck,
            _cs: cs,
            _gate: gate,
            _char: PhantomData,
            op_mode,
        })
    }
}

impl<'a, 'b, 'c, 'd> FullDuplex<u8> for SpiMaster<'a, 'b, 'c, 'd, u8> {
    type Error = ();

    fn send(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        if self.op_mode == OpMode::IMM {
            if self.reg.s.read().get_bit(5) {
                unsafe {
                    self.reg.dl.write(word);
                }
                Ok(())
            } else {
                Err(Error::WouldBlock)
            }
        } else {
            unimplemented!()
        }
    }

    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        if self.op_mode == OpMode::IMM {
            if self.reg.s.read().get_bit(7) {
                Ok(self.reg.dl.read())
            } else {
                Err(Error::WouldBlock)
            }
        } else {
            unimplemented!()
        }
    }
}

impl<'a, 'b, 'c, 'd> FullDuplex<u16> for SpiMaster<'a, 'b, 'c, 'd, u16> {
    type Error = ();

    fn send(&mut self, word: u16) -> nb::Result<(), Self::Error> {
        if self.op_mode == OpMode::IMM {
            if self.reg.s.read().get_bit(5) {
                unsafe {
                    self.reg.dh.write((word >> 8) as u8);
                    self.reg.dl.write(word as u8);
                }
                Ok(())
            } else {
                Err(Error::WouldBlock)
            }
        } else {
            unimplemented!()
        }
    }

    fn read(&mut self) -> nb::Result<u16, Self::Error> {
        if self.op_mode == OpMode::IMM {
            if self.reg.s.read().get_bit(7) {
                let d = ((self.reg.dh.read() as u16) << 8) | (self.reg.dl.read() as u16);
                Ok(d)
            } else {
                Err(Error::WouldBlock)
            }
        } else {
            unimplemented!()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[cfg_attr(rustfmt, rustfmt_skip)]
    #[test]
    fn spi0_regs() {
        unsafe {
            let reg = &*(SPI0_ADDR as *const SpiRegs);
            assert_eq!(0x4007_6000 as *const RO<u8>, &reg.s     as *const RO<u8>, "s");
            assert_eq!(0x4007_6001 as *const RW<u8>, &reg.br    as *const RW<u8>, "br");
            assert_eq!(0x4007_6002 as *const RW<u8>, &reg.c2    as *const RW<u8>, "c2");
            assert_eq!(0x4007_6003 as *const RW<u8>, &reg.c1    as *const RW<u8>, "c1");
            assert_eq!(0x4007_6004 as *const RW<u8>, &reg.ml    as *const RW<u8>, "ml");
            assert_eq!(0x4007_6005 as *const RW<u8>, &reg.mh    as *const RW<u8>, "mh");
            assert_eq!(0x4007_6006 as *const RW<u8>, &reg.dl    as *const RW<u8>, "dl");
            assert_eq!(0x4007_6007 as *const RW<u8>, &reg.dh    as *const RW<u8>, "dh");
            assert_eq!(0x4007_600A as *const RW<u8>, &reg.ci    as *const RW<u8>, "ci");
            assert_eq!(0x4007_600B as *const RW<u8>, &reg.c3    as *const RW<u8>, "c3");
        }
    }

    #[cfg_attr(rustfmt, rustfmt_skip)]
    #[test]
    fn spi1_regs() {
        unsafe {
            let reg = &*(SPI1_ADDR as *const SpiRegs);
            assert_eq!(0x4007_7000 as *const RO<u8>, &reg.s     as *const RO<u8>, "s");
            assert_eq!(0x4007_7001 as *const RW<u8>, &reg.br    as *const RW<u8>, "br");
            assert_eq!(0x4007_7002 as *const RW<u8>, &reg.c2    as *const RW<u8>, "c2");
            assert_eq!(0x4007_7003 as *const RW<u8>, &reg.c1    as *const RW<u8>, "c1");
            assert_eq!(0x4007_7004 as *const RW<u8>, &reg.ml    as *const RW<u8>, "ml");
            assert_eq!(0x4007_7005 as *const RW<u8>, &reg.mh    as *const RW<u8>, "mh");
            assert_eq!(0x4007_7006 as *const RW<u8>, &reg.dl    as *const RW<u8>, "dl");
            assert_eq!(0x4007_7007 as *const RW<u8>, &reg.dh    as *const RW<u8>, "dh");
            assert_eq!(0x4007_700A as *const RW<u8>, &reg.ci    as *const RW<u8>, "ci");
            assert_eq!(0x4007_700B as *const RW<u8>, &reg.c3    as *const RW<u8>, "c3");
        }
    }
}
