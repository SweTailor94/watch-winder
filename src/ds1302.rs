//! ds1302 RTC implementation for RP2040 
//! Using modified CYW43-pio-spi-driver 
//! 


use defmt::info;

use crate::ds1302_pio_spi::SpiBusDs1302;



pub struct Ds1302< SPI > {
    spi: SPI
}

impl<SPI> Ds1302<SPI> 
where SPI: SpiBusDs1302,
{
    pub fn new(spi: SPI) -> Self{
        Ds1302{
            spi
        }
    } 

    pub async fn read_hour(&mut self) -> u8{
        let mut data = [0u8];
        self.spi.cmd_read(READ_HOUR, &mut data ).await;
        
        if data[0] & 0x80 > 0 { // 12 h system
            bcd_to_u8(data[0] & 0x1F) + if data[0] & 0x20 > 0 {12}else{0}
        } else {
            bcd_to_u8(data[0] & 0x3F)
        }
    }
    pub async fn read_minutes(&mut self) -> u8{
        let mut data = [0u8];
        self.spi.cmd_read(READ_MINUTES, &mut data ).await;
        bcd_to_u8(data[0])
    }
    pub async fn read_seconds(&mut self) -> u8{
        let mut data = [0u8];
        self.spi.cmd_read(READ_SECONDS, &mut data ).await;
        bcd_to_u8(data[0]&0x7F)
    }

    pub async fn read_clock(&mut self) -> (u8,u8,u8) {
        let mut data : [u8; 8] = [0;8];
        self.spi.cmd_read(READ_CLOCK_BURST, &mut data).await;
        info!("{}, {}, {}, {}, {}, {}, {}, {}",data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7]);
        let sec = bcd_to_u8(data[0]&0x7F);
        let min = bcd_to_u8(data[1]);
        let h = if data[2] & 0x80 > 0 { // 12 h system
            bcd_to_u8(data[2] & 0x1F) + if data[2] & 0x20 > 0 {12}else{0}
        } else {
            bcd_to_u8(data[2] & 0x3F)
        };
        (h,min,sec)
    }

    pub async fn read_ram(&mut self, addr:u8) -> u8{
        let cmd = RAM_START + addr * 2 + 1; // + 1 is read
        let mut data = [0u8];
        self.spi.cmd_read(cmd, &mut data).await;
        data[0]
    }

    pub async fn write_hour(&mut self, h:u8 ){
        let bcd = u8_to_bcd(h & 0x3f);
        let write = [WRITE_HOUR, bcd];
        self.spi.cmd_write(&write).await;
    }

    pub async fn write_minutes(&mut self, m:u8 ){
        let bcd = u8_to_bcd(m & 0x7F);
        let write = [WRITE_MINUTES, bcd];
        self.spi.cmd_write(&write).await;
    }
    pub async fn write_seconds(&mut self, seconds:u8 ){
        let bcd = u8_to_bcd(seconds & 0x7F);
        let write = [WRITE_SECONDS, bcd];
        self.spi.cmd_write(&write).await;
    }

    pub async fn write_ram(&mut self, addr:u8, val: u8){
        let cmd = RAM_START+addr*2;
        let write = [cmd,val];
        self.spi.cmd_write(&write).await;
    }

    pub async fn enable_write(&mut self){
        let write = [WRITE_WP,0];
        self.spi.cmd_write(&write).await;
    }



    
}

// Command bytes for
const READ_SECONDS: u8 = 0x81;
const WRITE_SECONDS: u8 = 0x80;
const READ_MINUTES: u8 = 0x83;
const WRITE_MINUTES: u8 = 0x82;
const READ_HOUR: u8 = 0x85;
const WRITE_HOUR: u8 = 0x84; 

const READ_CLOCK_BURST:u8 = 0xBF;
const RAM_START:u8 = 0xC0;

const WRITE_WP: u8 = 0x8E; // WriteProtection bit7 = 0 to enable write.


pub fn bcd_to_u8(bcd:u8) -> u8{
    ((bcd & 0xF0) >> 4) *10 +(bcd & 0x0F)
}

pub fn u8_to_bcd(val:u8) -> u8{
    let tio = val/10;
    (tio<<4 )+ val-10*tio  
}

