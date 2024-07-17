#![allow(stable_features, unknown_lints, async_fn_in_trait)]


use defmt::trace;
use embassy_futures::yield_now;
use embassy_rp::gpio::{Drive, Level, Output, Pin, Pull, SlewRate};
use embassy_rp::pio::{Common, Config, Direction, Instance, Irq, PioPin, ShiftDirection, StateMachine, instr};
use fixed::FixedU32;
use pio_proc::pio_asm;

pub struct PioSpiDs1302<'d, CS: Pin, PIO: Instance, const SM: usize> {
    cs: Output<'d, CS>,
    sm: StateMachine<'d, PIO, SM>,
    irq: Irq<'d, PIO, 0>,
    wrap_target: u8,
}

impl<'d, CS, PIO, const SM: usize> PioSpiDs1302<'d, CS, PIO, SM>
where
    CS: Pin,
    PIO: Instance,
{
    pub fn new<DIO, CLK>(
        common: &mut Common<'d, PIO>,
        mut sm: StateMachine<'d, PIO, SM>,
        irq: Irq<'d, PIO, 0>,
        cs: Output<'d, CS>,
        dio: DIO,
        clk: CLK,
    ) -> Self
    where
        DIO: PioPin,
        CLK: PioPin,
    {
        let program = pio_asm!(
            ".side_set 1" // this is the CLK
            ".wrap_target"
            // write out x+1 bits 
            "lp:"
            "out pins, 1    side 0" // CLK low
            "jmp x-- lp     side 1" // CLK high
            // switch directions
            // "set pins, 0    side 0"
            "jmp !y done    side 0" // CLK low
            "set pindirs, 0 side 0" // CLK low
            // read in y-1 bits
            "lp2:"
            "in pins, 1     side 1" // CLK high
            "jmp y-- lp2    side 0" // CLK low
            "done:"
            // irq host so we kno when to turn off CS while reading 
            "irq 0          side 0" // CLK low Shal l stop on low
            ".wrap"
        );

        let mut pin_io: embassy_rp::pio::Pin<PIO> = common.make_pio_pin(dio);
        pin_io.set_pull(Pull::None);
        pin_io.set_schmitt(true);
        pin_io.set_input_sync_bypass(false);
        pin_io.set_drive_strength(Drive::_12mA);
        pin_io.set_slew_rate(SlewRate::Fast);

        let mut pin_clk = common.make_pio_pin(clk);
        pin_clk.set_drive_strength(Drive::_12mA);
        pin_clk.set_slew_rate(SlewRate::Fast);

        let mut cfg = Config::default();
        let loaded_program = common.load_program(&program.program);
        cfg.use_program(&loaded_program, &[&pin_clk]);
        cfg.set_out_pins(&[&pin_io]);
        cfg.set_in_pins(&[&pin_io]);
        cfg.set_set_pins(&[&pin_io]);
        cfg.shift_out.direction = ShiftDirection::Right; // lsb shal bi shifted out first
        cfg.shift_out.auto_fill = true;
        cfg.shift_out.threshold = 8;
        cfg.shift_in.direction = ShiftDirection::Right; // lsb is shifted in first
        cfg.shift_in.auto_fill = true;
        cfg.shift_in.threshold = 8;
        
        // This division gives a spi clock of ~ 1 MHz
        cfg.clock_divider = FixedU32::from_bits(0x4000);
    
        sm.set_config(&cfg);

        sm.set_pin_dirs(Direction::Out, &[&pin_clk, &pin_io]);
        sm.set_pins(Level::Low, &[&pin_clk, &pin_io]);

        Self {
            cs,
            sm,
            irq,
            wrap_target: loaded_program.wrap.target,
        }
    }

    pub async fn write(&mut self, write: &[u8]) {
        self.sm.set_enable(false);
        let write_bits = write.len() * 8 - 1; // -1 because pio runs x+1
        let read_bits = 0; // No data to read on write operation
        // defmt::info!("write={} read={}", write_bits, read_bits);
        unsafe {
            instr::set_x(&mut self.sm, write_bits as u32);
            instr::set_y(&mut self.sm, read_bits as u32);
            instr::set_pindir(&mut self.sm, 0b1);
            instr::exec_jmp(&mut self.sm, self.wrap_target);
        }
        self.sm.set_enable(true);
        for b in write{
            // trace!("Write {:02X}",b);
            self.sm.tx().wait_push(*b as u32).await;
        }
        self.irq.wait().await;
        // trace!("Done writing.");        
    }

    pub async fn cmd_read(&mut self, cmd: u8, read: &mut [u8]) {
        self.sm.set_enable(false);
        let write_bits = 8-1;// -1 because pio writes x+1 bits
        let read_bits = read.len() *8 -1;// -1 because pio reads y+1 bits        
        // defmt::info!("write={} read={}", write_bits, read_bits);
        unsafe {
            instr::set_y(&mut self.sm, read_bits as u32); 
            instr::set_x(&mut self.sm, write_bits as u32);
            instr::set_pindir(&mut self.sm, 0b1);
            instr::exec_jmp(&mut self.sm, self.wrap_target);
        }
        self.sm.set_enable(true);

                self.sm.tx().wait_push(cmd as u32).await;
        for i in 0..read.len(){
            let b = self.sm.rx().wait_pull().await; 
            read[i] = (b >> 24) as u8; // Since we shift right, our byte is the MSB in 32bit word.
        }
        self.irq.wait().await;
    }
}

/// Custom Spi Trait that _only_ supports the bus operation of the RTC chip DS1302
/// Implementors are expected to hold the CS pin High during an operation.
pub trait SpiBusDs1302 {
    /// Issues a write command on the bus
    /// First byte of `write` is expected to be a cmd word
    async fn cmd_write(&mut self, write: &[u8]) -> bool;

    /// Issues a read command on the bus
    /// `write` is expected to be a 8 bit cmd word
    /// `read` will contain the response of the device
    ///  works for single byte read and burst mode (read.len() = 8 for clock, 31 for ). 
    async fn cmd_read(&mut self, write_cmd: u8, read: &mut [u8]) -> bool;

    /// Wait for events from the Device. A typical implementation would wait for the IRQ pin to be high.
    /// The default implementation always reports ready, resulting in active polling of the device.
    async fn wait_for_event(&mut self) {
        yield_now().await;
    }
}

impl<'d, CS, PIO, const SM: usize> SpiBusDs1302 for PioSpiDs1302<'d, CS, PIO, SM>
where
    CS: Pin,
    PIO: Instance,
{
    async fn cmd_write(&mut self, write: &[u8]) -> bool {
        self.cs.set_high();
        self.write(write).await;
        self.cs.set_low();
        true
    }

    async fn cmd_read(&mut self, cmd: u8, read: &mut [u8]) -> bool {
        self.cs.set_high();
        self.cmd_read(cmd, read).await;
        self.cs.set_low();
        true
    }

    async fn wait_for_event(&mut self) {
        self.irq.wait().await;
    }
}
