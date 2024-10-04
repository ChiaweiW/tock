use crate::timer::TimerRegisters;
use crate::uart::UartRegisters;
use core::fmt::Write;
use kernel::utilities::registers::interfaces::{ReadWriteable, Readable};
use kernel::utilities::StaticRef;
use rv32i::csr::{mie::mie, mip::mip, CSR};
use rv32i::syscall::SysCall;

pub struct Ast2700<'a> {
    userspace_kernel_boundary: SysCall,
    timer: &'a crate::timer::Timer<'a>,
}

impl<'a> Ast2700<'a> {
    pub unsafe fn new(timer: &'a crate::timer::Timer<'a>) -> Self {
        Self {
            userspace_kernel_boundary: SysCall::new(),
            timer,
        }
    }
}

pub struct Ast2700DefaultPeripheral<'a> {
    pub uart: crate::uart::Uart<'a>,
    pub timer: crate::timer::Timer<'a>,
}

impl<'a> Ast2700DefaultPeripheral<'a> {
    pub fn new() -> Self {
        Self {
            uart: crate::uart::Uart::new(unsafe {
                StaticRef::new(0x14c33b00 as *const UartRegisters)
            }),
            timer: crate::timer::Timer::new(unsafe {
                StaticRef::new(0x14c36000 as *const TimerRegisters)
            }),
        }
    }
}

impl<'a> kernel::platform::chip::Chip for Ast2700<'a> {
    type MPU = ();
    type UserspaceKernelBoundary = SysCall;

    fn mpu(&self) -> &Self::MPU {
        &()
    }

    fn userspace_kernel_boundary(&self) -> &SysCall {
        &self.userspace_kernel_boundary
    }

    fn service_pending_interrupts(&self) {
        let mip = CSR.mip.extract();

        if mip.is_set(mip::mtimer) {
            self.timer.service_interrupt();
        }

        CSR.mie.modify(mie::mtimer::SET);
    }

    fn has_pending_interrupts(&self) -> bool {
        let mip = CSR.mip.extract();

        mip.is_set(mip::mtimer) || mip.is_set(mip::mext)
    }

    fn sleep(&self) {
        unsafe {
            rv32i::support::wfi();
        }
    }

    unsafe fn atomic<F, R>(&self, f: F) -> R
    where
        F: FnOnce() -> R,
    {
        rv32i::support::atomic(f)
    }

    unsafe fn print_state(&self, writer: &mut dyn Write) {
        rv32i::print_riscv_state(writer);
    }
}

#[export_name = "_start_trap_rust_from_kernel"]
pub unsafe extern "C" fn start_trap_rust() {}

/// Function that gets called if an interrupt occurs while an app was running.
/// mcause is passed in, and this function should correctly handle disabling the
/// interrupt that fired so that it does not trigger again.
#[export_name = "_disable_interrupt_trap_rust_from_app"]
pub unsafe extern "C" fn disable_interrupt_trap_handler(_mcause_val: u32) {}
