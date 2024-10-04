// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2022.

//! UART driver.

use kernel::hil::uart;
use kernel::utilities::cells::OptionalCell;
use kernel::utilities::registers::interfaces::{ReadWriteable, Readable, Writeable};
use kernel::utilities::registers::{register_bitfields, register_structs};
use kernel::utilities::registers::{Aliased, ReadWrite};
use kernel::utilities::StaticRef;
use kernel::{hil, ErrorCode};

register_structs! {
    pub UartRegisters {
        (0x00 => rbr_thr: Aliased<u32, RBR::Register, THR::Register>),
        (0x04 => ier: ReadWrite<u32, IER::Register>),
        (0x08 => iir_fcr: Aliased<u32, IIR::Register, FCR::Register>),
        (0x0C => lcr: ReadWrite<u32, LCR::Register>),
        (0x10 => mcr: ReadWrite<u32, MCR::Register>),
        (0x14 => lsr: ReadWrite<u32, LSR::Register>),
        (0x18 => msr: ReadWrite<u32, MSR::Register>),
        (0x1c => scr: ReadWrite<u32, SCR::Register>),
        (0x20 => @END),
    }
}

register_bitfields! [u32,
    RBR [
        DATA    OFFSET(0) NUMBITS(8) [],
    ],
    THR [
        DATA    OFFSET(0) NUMBITS(8) [],
    ],
    IER [
        MSI     OFFSET(3) NUMBITS(1) [],
        RLSI    OFFSET(2) NUMBITS(1) [],
        THRI    OFFSET(1) NUMBITS(1) [],
        RDI     OFFSET(0) NUMBITS(1) [],
    ],
    IIR [
        FIFO_STS    OFFSET(6) NUMBITS(2) [ FIFO_DISABLED = 0, FIFO_ENABLED = 3 ],
        INT_ID      OFFSET(1) NUMBITS(3) [ MSI = 0, THRI = 1, RDI = 2, RLSI = 3, RX_TIMEOUT = 6 ],
        INT_PEND    OFFSET(0) NUMBITS(1) [] ,
    ],
    FCR [
        RX_FIFO_THR OFFSET(6) NUMBITS(2) [ BYTES1 = 0, BYTES4 = 1, BYTES8 = 2, BYTES14 = 3 ],
        TX_FIFO_RST OFFSET(2) NUMBITS(1) [],
        RX_FIFO_RST OFFSET(1) NUMBITS(1) [],
        FIFO_EN     OFFSET(0) NUMBITS(1) [],
    ],
    LCR [
        DLAB    OFFSET(7) NUMBITS(1) [],
        SBC     OFFSET(6) NUMBITS(1) [],
        SPAR    OFFSET(5) NUMBITS(1) [],
        EPS     OFFSET(4) NUMBITS(2) [ ODD = 0, EVEN = 1, HIGH = 2, LOW = 3 ],
        PEN     OFFSET(3) NUMBITS(1) [],
        STOP    OFFSET(2) NUMBITS(1) [ ONE = 0, ONEHALF_TWO = 1 ],
        CLS     OFFSET(0) NUMBITS(2) [ BITS5 = 0, BITS6 = 1, BITS7 = 2, BITS8 = 3 ],
    ],
    MCR [
        LOOP    OFFSET(4) NUMBITS(1) [],
        OUT2    OFFSET(3) NUMBITS(1) [],
        OUT1    OFFSET(2) NUMBITS(1) [],
        NRTS    OFFSET(1) NUMBITS(1) [],
        NDTR    OFFSET(0) NUMBITS(1) [],
    ],
    LSR [
        FIFOE   OFFSET(7) NUMBITS(1) [],
        TEMT    OFFSET(6) NUMBITS(1) [],
        THRE    OFFSET(5) NUMBITS(1) [],
        BI      OFFSET(4) NUMBITS(1) [],
        FE      OFFSET(3) NUMBITS(1) [],
        PE      OFFSET(2) NUMBITS(1) [],
        OE      OFFSET(1) NUMBITS(1) [],
        DR      OFFSET(0) NUMBITS(1) [],
    ],
    MSR [
        DDCD    OFFSET(3) NUMBITS(1) [],
        TERI    OFFSET(2) NUMBITS(1) [],
        DDSR    OFFSET(1) NUMBITS(1) [],
        DCTS    OFFSET(0) NUMBITS(1) [],
    ],
    SCR [
        DATA    OFFSET(0) NUMBITS(8) [],
    ],
];

pub struct Uart<'a> {
    regs: StaticRef<UartRegisters>,
    tx_client: OptionalCell<&'a dyn hil::uart::TransmitClient>,
    rx_client: OptionalCell<&'a dyn hil::uart::ReceiveClient>,
    clock_frequency: u32,
}

impl<'a> Uart<'a> {
    pub fn new(base: StaticRef<UartRegisters>) -> Uart<'a> {
        Uart {
            regs: base,
            tx_client: OptionalCell::empty(),
            rx_client: OptionalCell::empty(),
            clock_frequency: 1846153u32,
        }
    }
}

impl hil::uart::Configure for Uart<'_> {
    fn configure(&self, params: hil::uart::Parameters) -> Result<(), ErrorCode> {
        let regs = self.regs;
        let div = (self.clock_frequency / 16) / params.baud_rate;

        // set <baud_rate>n8
        regs.lcr.modify(LCR::DLAB::SET + LCR::CLS::BITS8);
        regs.rbr_thr.set(div.into());
        regs.ier.set((div >> 8).into());
        regs.lcr.modify(LCR::DLAB::CLEAR + LCR::CLS::BITS8);

        // set parity mode
        match params.parity {
            uart::Parity::Even => regs.lcr.modify(LCR::PEN::SET + LCR::EPS::EVEN),
            uart::Parity::Odd => regs.lcr.modify(LCR::PEN::SET + LCR::EPS::ODD),
            uart::Parity::None => regs.lcr.modify(LCR::PEN::CLEAR + LCR::EPS::CLEAR),
        }

        // set stop bits
        match params.stop_bits {
            uart::StopBits::One => regs.lcr.modify(LCR::STOP::ONE),
            uart::StopBits::Two => regs.lcr.modify(LCR::STOP::ONEHALF_TWO),
        }

        // enable FIFO
        regs.iir_fcr
            .set(FCR::FIFO_EN::SET.value + FCR::RX_FIFO_THR::BYTES8.value);

        // disable all interrupts for now
        regs.ier.set(0x0);

        Ok(())
    }
}

impl<'a> hil::uart::Transmit<'a> for Uart<'a> {
    fn set_transmit_client(&self, client: &'a dyn uart::TransmitClient) {
        self.tx_client.set(client)
    }

    fn transmit_buffer(
        &self,
        tx_buffer: &'static mut [u8],
        tx_len: usize,
    ) -> Result<(), (ErrorCode, &'static mut [u8])> {
        let regs = self.regs;
        let mut i = 0;

        while i < tx_len {
            if regs.lsr.is_set(LSR::TEMT) && regs.lsr.is_set(LSR::THRE) {
                regs.rbr_thr.write(THR::DATA.val(tx_buffer[i].into()));
                i += 1;
            }
        }

        //Ok(())

        Err((ErrorCode::FAIL, tx_buffer))
    }

    fn transmit_word(&self, _word: u32) -> Result<(), ErrorCode> {
        Err(ErrorCode::FAIL)
    }

    fn transmit_abort(&self) -> Result<(), ErrorCode> {
        Err(ErrorCode::FAIL)
    }
}

impl<'a> hil::uart::Receive<'a> for Uart<'a> {
    fn set_receive_client(&self, client: &'a dyn uart::ReceiveClient) {
        self.rx_client.set(client)
    }

    fn receive_buffer(
        &self,
        rx_buffer: &'static mut [u8],
        rx_len: usize,
    ) -> Result<(), (ErrorCode, &'static mut [u8])> {
        let regs = self.regs;
        let mut i = 0;

        while i < rx_len && regs.lsr.is_set(LSR::DR) {
            rx_buffer[i] = regs.rbr_thr.read(RBR::DATA) as u8;
            i = i + 1;
        }

        Ok(())
    }

    fn receive_word(&self) -> Result<(), ErrorCode> {
        Err(ErrorCode::FAIL)
    }

    fn receive_abort(&self) -> Result<(), ErrorCode> {
        Err(ErrorCode::FAIL)
    }
}
