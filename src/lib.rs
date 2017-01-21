// Copyright 2016 Claus Matzinger
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

extern crate i2cdev;

use std::thread;
use std::time::Duration;
use std::error::Error;
use i2cdev::core::*;
use i2cdev::sensors::{Thermometer, Barometer};


// -------------------------------------------------------------------------------

pub const TSL2561_I2C_ADDR: u16 = 0x39; //TSL2561 default address.

///
/// Gain mode
///
#[derive(Clone, Debug)]
pub enum Gain {
    Auto = 0,
    Low = 1,
    High = 16,
}

// -------------------------------------------------------------------------------

// BMP085 register addresses
enum Register {
    Tsl2561Full = 0x8C, // R   Calibration data (16 bits)
    Tsl2561IR = 0x8E, // R   Calibration data (16 bits)
    Tsl2561Control = 0xF4,
    Tsl2561Data = 0x0C, // Data0
}

// BMP085 command register addresses
enum Command {
    Tsl2561ReadData = 0xAC,
    Bmp085ReadPressureCmd = 0x34,
}

// -------------------------------------------------------------------------------

// Helper method to convert an u16 x endian integer to a i16 big endian integer
fn i_to_be(r: u16) -> i16 {
    let a = r as i16;
    a.to_be()
}

// Helper method to convert an u16 x endian integer to a u16 big endian integer
fn u_to_be(r: u16) -> u16 {
    r.to_be()
}

// ---------------------------------------------------------------------------------

///
/// Reads the raw temperature data...
///
fn read_raw_temp<E: Error>(dev: &mut I2CDevice<Error = E>,
                           coeff: &BMP085SensorCoefficients)
                           -> Result<(i32, i32), E> {
    try!(dev.smbus_write_byte_data(Register::Bmp085Control as u8,
                                   Command::Bmp085ReadTempCmd as u8));

    thread::sleep(Duration::from_millis(5)); // sleep for 4.5 ms

    let ut: i32 = i_to_be(try!(dev.smbus_read_word_data(Register::Bmp085Data as u8))) as i32;
    let ac6: i32 = coeff.cal_ac6 as i32;
    let ac5: i32 = coeff.cal_ac5 as i32;
    let md: i32 = coeff.cal_md as i32;
    let mc: i32 = coeff.cal_mc as i32;


    let _ac5 = ac5 as i64;
    let x1: i32 = ((ut - ac6) as i64 * _ac5 >> 15) as i32; // Note: X>>15 == X/(pow(2,15))
    let x2: i32 = (mc << 11) / (x1 + md); // Note: X<<11 == X<<(pow(2,11))
    let b5: i32 = x1 + x2;
    let t: i32 = (b5 + 8) >> 4;
    Ok((t, b5))
}

///
/// Reads the raw full (IR + Lux) data...
///
fn read_raw_full<E: Error>(dev: &mut I2CDevice<Error = E>) -> Result<u16, E> {
    let ir = try!(dev.smbus_read_word_data(Register::Tsl2561Full as u8))
    Ok(ir)
}

///
/// Reads the raw infrafred data...
///
fn read_raw_ir<E: Error>(dev: &mut I2CDevice<Error = E>) -> Result<u16, E> {
    let ir = try!(dev.smbus_read_word_data(Register::Tsl2561IR as u8))
    Ok(ir)
}


///
/// Reads the raw data for visible light...
///
fn read_raw_lux<E: Error>(dev: &mut I2CDevice<Error = E>, gain: Gain) -> Result<(i32, i32), E> {
    match gain {
        Gain::High -> ,
        Gain::Low -> ,
        Gain::Auto ->,
    }

    try!(dev.smbus_write_byte_data(Register::Bmp085Control as u8,
                                   Command::Bmp085ReadTempCmd as u8));

    thread::sleep(Duration::from_millis(5)); // sleep for 4.5 ms

    let ut: i32 = i_to_be(try!(dev.smbus_read_word_data(Register::Bmp085Data as u8))) as i32;
    let ac6: i32 = coeff.cal_ac6 as i32;
    let ac5: i32 = coeff.cal_ac5 as i32;
    let md: i32 = coeff.cal_md as i32;
    let mc: i32 = coeff.cal_mc as i32;


    let _ac5 = ac5 as i64;
    let x1: i32 = ((ut - ac6) as i64 * _ac5 >> 15) as i32; // Note: X>>15 == X/(pow(2,15))
    let x2: i32 = (mc << 11) / (x1 + md); // Note: X<<11 == X<<(pow(2,11))
    let b5: i32 = x1 + x2;
    let t: i32 = (b5 + 8) >> 4;
    Ok((t, b5))
}


// -------------------------------------------------------------------------------

///
/// The TSL2561 luminosity sensor
///
pub struct TSL2561LuminositySensor<T: I2CDevice + Sized> {
    pub dev: T,
}

impl<T> TSL2561LuminositySensor<T>
    where T: I2CDevice + Sized
{
    ///
    /// Calibrates and creates a sensor representation.
    ///
    /// # Examples
    /// ```rust,ignore
    /// use i2cdev::linux::*;
    /// use bmp085::*;
    /// use i2cdev::sensors::{Barometer, Thermometer};
    /// let i2c_dev = LinuxI2CDevice::new("/dev/i2c-1", TSL2561_I2C_ADDR).unwrap();
    /// let mut s = BMP085BarometerThermometer::new(i2c_dev,
    ///                     SamplingMode::Standard).unwrap();
    /// println!("Temperature: {:?} C", s.temperature_celsius().unwrap());
    /// println!("Pressure:    {:?} kPa", s.pressure_kpa().unwrap());
    /// ```
    pub fn new(mut dev: T)
               -> Result<TSL2561LuminositySensor<T>, T::Error> {
        Ok(TSL2561LuminositySensor {
            dev: dev,
        })
    }
}

trait LuminositySensor<T> {
    fn read_ir(&mut self) -> Result<u16, T::Error>;
    fn read_full(&mut self) -> Result<u16, T::Error>;
}

impl<T> LuminositySensor for TSL2561LuminositySensor<T>
    where T: I2CDevice + Sized
{
    type Error = T::Error;

    fn read_ir(&mut self) -> Result<u16, T::Error>{
        read_raw_ir(&mut self.dev)
    }
    fn read_full(&mut self) -> Result<u16, T::Error> {
        read_raw_full(&mut self.dev)
    }
}


// ---------------------------------------------------------------------------------


#[cfg(test)]
mod tests {

    extern crate byteorder;
    extern crate rand;

    use super::{Command, Register, SamplingMode, BMP085SensorCoefficients, BMP085BarometerThermometer};
    use i2cdev::sensors::{Thermometer, Barometer};
    use i2cdev::core::I2CDevice;
    use self::byteorder::{BigEndian, ByteOrder};
    use self::rand::Rng;
    use std::io;

    pub struct MockTSL2561 {
        reg: Register,
        offset: usize,
        t_data: u16,
        p_data: u16,
        last_cmd: Command,
    }


    impl I2CDevice for MockBMP085 {
        type Error = io::Error;

        fn read(&mut self, data: &mut [u8]) -> Result<(), Self::Error> {
            let mut buf = [0; 3]; // array of size 3 for simpler offsets
            let reading = match self.last_cmd {
                Command::Bmp085ReadTempCmd => self.t_data,
                Command::Bmp085ReadPressureCmd => self.p_data, // Pressure command :)
            };

            match self.reg {
                Register::Tsl2561AC1 => BigEndian::write_i16(&mut buf, self.coeff.cal_ac1),
                Register::Tsl2561AC2 => BigEndian::write_i16(&mut buf, self.coeff.cal_ac2),
                Register::Tsl2561AC3 => BigEndian::write_i16(&mut buf, self.coeff.cal_ac3),
                Register::Tsl2561AC4 => BigEndian::write_u16(&mut buf, self.coeff.cal_ac4),
                Register::Tsl2561AC5 => BigEndian::write_u16(&mut buf, self.coeff.cal_ac5),
                Register::Tsl2561AC6 => BigEndian::write_u16(&mut buf, self.coeff.cal_ac6),
                Register::Tsl2561B1 => BigEndian::write_i16(&mut buf, self.coeff.cal_b1),
                Register::Tsl2561B2 => BigEndian::write_i16(&mut buf, self.coeff.cal_b2),
                Register::Tsl2561Mb => BigEndian::write_i16(&mut buf, self.coeff.cal_mb),
                Register::Tsl2561Mc => BigEndian::write_i16(&mut buf, self.coeff.cal_mc),
                Register::Tsl2561Md => BigEndian::write_i16(&mut buf, self.coeff.cal_md),
                Register::Bmp085Control => BigEndian::write_i16(&mut buf, 0),
                Register::Bmp085Data => BigEndian::write_u16(&mut buf, reading),
            };

            for (i, elem) in data.iter_mut().enumerate() {
                *elem = buf[i + self.offset]
            }
            Ok(())
        }

        fn write(&mut self, data: &[u8]) -> Result<(), Self::Error> {
            let d = [0, data[0]];
            self.offset = 0;
            self.reg = match BigEndian::read_u16(&d) {
                0xAA => Register::Tsl2561AC1,
                0xAC => Register::Tsl2561AC2,
                0xAE => Register::Tsl2561AC3,
                0xB0 => Register::Tsl2561AC4,
                0xB2 => Register::Tsl2561AC5,
                0xB4 => Register::Tsl2561AC6,
                0xB6 => Register::Tsl2561B1,
                0xB8 => Register::Tsl2561B2,
                0xBA => Register::Tsl2561Mb,
                0xBC => Register::Tsl2561Mc,
                0xBE => Register::Tsl2561Md,
                0xF4 => {
                    self.last_cmd = if data[1] == (Command::Bmp085ReadTempCmd as u8) {
                        Command::Bmp085ReadTempCmd
                    } else {
                        // don't do exact matching to prevent sampling mode influences
                        Command::Bmp085ReadPressureCmd
                    };
                    Register::Bmp085Control
                }
                0xF6 => Register::Bmp085Data,
                0xF7 => {
                    self.offset = 1;
                    Register::Bmp085Data
                }
                0xF8 => {
                    self.offset = 2;
                    Register::Bmp085Data
                }
                _ => unimplemented!(),
            };
            Ok(())
        }

        fn smbus_write_quick(&mut self, _bit: bool) -> Result<(), Self::Error> {
            unimplemented!()
        }

        fn smbus_read_block_data(&mut self, _register: u8) -> Result<Vec<u8>, Self::Error> {
            unimplemented!()
        }

        fn smbus_write_block_data(&mut self,
                                  _register: u8,
                                  _values: &[u8])
                                  -> Result<(), Self::Error> {
            unimplemented!()
        }

        fn smbus_process_block(&mut self,
                               _register: u8,
                               _values: &[u8])
                               -> Result<(), Self::Error> {
            unimplemented!()
        }

        fn smbus_read_i2c_block_data(&mut self,
                                     _register: u8,
                                     _len: u8)
                                     -> Result<Vec<u8>, Self::Error> {
            unimplemented!()
        }
    }

    fn new_i2c_mock(temperature: u16, pressure: u16) -> MockBMP085 {
        let coeff = BMP085SensorCoefficients {
            cal_ac1: 408i16,
            cal_ac2: -72i16,
            cal_ac3: -14383i16,
            cal_ac4: 32741u16,
            cal_ac5: 32757u16,
            cal_ac6: 23153u16,
            cal_b1: 6190i16,
            cal_b2: 4i16,
            cal_mb: -32768i16,
            cal_mc: -8711i16,
            cal_md: 2868i16,
        };
        MockBMP085 {
            coeff: coeff,
            reg: Register::Tsl2561AC1,
            offset: 0,
            t_data: temperature,
            p_data: pressure,
            last_cmd: Command::Bmp085ReadTempCmd,
        }
    }

    fn make_dev(i2cdev: MockBMP085) -> BMP085BarometerThermometer<MockBMP085> {
        BMP085BarometerThermometer::new(i2cdev, SamplingMode::UltraLowPower).unwrap()
    }

    #[test]
    fn test_basic_pressure_read() {
        let i2cdev = new_i2c_mock(27898, 23843);
        let mut dev = make_dev(i2cdev);
        assert_eq!(dev.pressure_kpa().unwrap(), 69.964);
    }

    #[test]
    #[should_panic(expected = "attempt to multiply with overflow")]
    fn test_zero_pressure_read() {
        let i2cdev = new_i2c_mock(0, 0);
        let mut dev = make_dev(i2cdev);
        assert_eq!(dev.pressure_kpa().unwrap(), 1.234);
    }

    #[test]
    fn test_basic_temp_read() {
        let i2cdev = new_i2c_mock(27898, 0);
        let mut dev = make_dev(i2cdev);
        assert_eq!(dev.temperature_celsius().unwrap(), 15.0);
    }

    #[test]
    fn test_zero_temp_read() {
        let i2cdev = new_i2c_mock(0, 0);
        let mut dev = make_dev(i2cdev);
        assert_eq!(dev.temperature_celsius().unwrap(), -139.2);
    }

    #[test]
    fn test_max_temp_read() {
        let i2cdev = new_i2c_mock(u16::max_value(), 0);
        let mut dev = make_dev(i2cdev);
        assert_eq!(dev.temperature_celsius().unwrap(), -139.2);
    }

    #[test]
    fn test_rand_temp_read() {
        let n = 2_000;
        let mut rng = rand::thread_rng();
        for i in 0..n {
            let i2cdev = new_i2c_mock(rng.gen::<u16>(), 0);
            let mut dev = make_dev(i2cdev);
            let _ = dev.temperature_celsius().unwrap();
        }

    }

}
