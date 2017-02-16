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


// -------------------------------------------------------------------------------

pub const TSL2561_I2C_ADDR: u16 = 0x39; //TSL2561 default address.


///
/// Gain mode
///
#[derive(Eq, PartialEq, Copy, Clone, Debug)]
pub enum Gain {
    Auto = 0,
    Low = 0x02,
    High = 0x12,
}

// -------------------------------------------------------------------------------

// BMP085 register addresses
enum Register {
    Tsl2561Enable = 0x80,
    Tsl2561Gain = 0x81,
    Tsl2561Full = 0x8C,
    Tsl2561IR = 0x8E,
    Tsl2561Data0 = 0x0C, // Data0/DataLow
    Tsl2561Data1 = 0x0D, // Data1/DataHigh
}

// BMP085 command register addresses
enum Command {
    Tsl2561Enable = 0x03,
}

// -------------------------------------------------------------------------------


// ---------------------------------------------------------------------------------

///
/// Reads the raw infrafred data...
///
fn enable_dev<E: Error>(dev: &mut I2CDevice<Error = E>) -> Result<(), E> {
    try!(dev.smbus_write_byte_data(Register::Tsl2561Enable as u8, Command::Tsl2561Enable as u8));
    Ok(())
}

///
/// Reads the raw infrafred data...
///
fn set_gain<E: Error>(dev: &mut I2CDevice<Error = E>, gain: &Gain) -> Result<(), E> {
    let r = try!(dev.smbus_write_byte_data(Register::Tsl2561Gain as u8, *gain as u8));
    thread::sleep(Duration::from_millis(400)); // sleep for 4.5 ms
    Ok(r)
}

///
/// Reads the raw full (IR + Lux) data...
///
fn read_raw_full<E: Error>(dev: &mut I2CDevice<Error = E>) -> Result<u16, E> {
    let full = try!(dev.smbus_read_word_data(Register::Tsl2561Full as u8)).to_le();
    Ok(full)
}

///
/// Reads the raw infrafred data...
///
fn read_raw_ir<E: Error>(dev: &mut I2CDevice<Error = E>) -> Result<u16, E> {
    let ir = try!(dev.smbus_read_word_data(Register::Tsl2561IR as u8)).to_le();
    Ok(ir)
}


// -------------------------------------------------------------------------------

///
/// The TSL2561 luminosity sensor
///
pub struct TSL2561LuminositySensor<T: I2CDevice + Sized> {
    pub dev: T,
    pub enabled: bool,
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
    pub fn new(dev: T) -> Result<TSL2561LuminositySensor<T>, T::Error> {
        Ok(TSL2561LuminositySensor {
            dev: dev,
            enabled: false,
        })
    }
}

pub trait LuminositySensor {
    type Error;
    fn read_lux(&mut self, gain: Gain) -> Result<f64, Self::Error>;
}

impl<T> LuminositySensor for TSL2561LuminositySensor<T>
    where T: I2CDevice + Sized
{
    type Error = T::Error;

    fn read_lux(&mut self, gain: Gain) -> Result<f64, T::Error> {
        if !self.enabled {
            try!(enable_dev(&mut self.dev));
            self.enabled = true;
        }
        let mut _gain = match gain {
            Gain::Auto | Gain::High => Gain::High,
            _ => Gain::Low,
        };

        try!(set_gain(&mut self.dev, &_gain));

        let _ambient = read_raw_full(&mut self.dev).unwrap();
        let _ambient = if (gain == Gain::Auto && _ambient >= u16::max_value()) {
            _gain = Gain::Low;
            try!(set_gain(&mut self.dev, &_gain));
            read_raw_full(&mut self.dev).unwrap()
        } else {
            _ambient
        };

        let _IR = read_raw_ir(&mut self.dev).unwrap();
        let (_ambient, _IR) = if (gain == Gain::Auto && _IR >= u16::max_value()) {
            _gain = Gain::Low;
            try!(set_gain(&mut self.dev, &_gain));
            let a = read_raw_full(&mut self.dev).unwrap();
            let i = read_raw_ir(&mut self.dev).unwrap();
            (a, i)
        } else {
            (_ambient, _IR)
        };

        let ambient = _ambient as f64;
        let IR = _IR as f64;

        println!("ambient={}, IR={}", ambient, IR);

        let (ambient, IR) = match _gain {
            Gain::High => (ambient * 16f64, IR * 16f64),
            _ => (ambient, IR),
        };

        let ratio = IR / ambient;

        let lux = if ratio >= 0f64 && ratio <= 0.52 {
            (0.0315 * ambient) - (0.0593 * ambient * ratio.powf(1.4))
        } else if ratio <= 0.65 {
            (0.0229 * ambient) - (0.0291 * IR)
        } else if ratio <= 0.80 {
            (0.0157 * ambient) - (0.018 * IR)
        } else if ratio <= 1.3 {
            (0.00338 * ambient) - (0.0026 * IR)
        } else {
            0f64
        };
        Ok(lux)
    }
}


// ---------------------------------------------------------------------------------


#[cfg(test)]
mod tests {

    extern crate byteorder;
    extern crate rand;

    use super::{Command, Register, TSL2561LuminositySensor};
    use i2cdev::core::I2CDevice;
    use self::byteorder::{BigEndian, ByteOrder};
    use self::rand::Rng;
    use std::io;

    pub struct MockTSL2561 {
        reg: Register,
        enabled: bool,
        ir_data: u16,
        full_data: u16,
    }


    impl I2CDevice for MockTSL2561 {
        type Error = io::Error;

        fn read(&mut self, data: &mut [u8]) -> Result<(), Self::Error> {
            let mut buf = [0; 3]; // array of size 3 for simpler offsets

            match self.reg {
                Register::Tsl2561Enable => BigEndian::write_u8(self.enabled as u8),
                Register::Tsl2561Gain => BigEndian::write_u16(self.gain_data),
                Register::Tsl2561Full => BigEndian::write_u16(self.full_data),
                Register::Tsl2561IR => BigEndian::write_u16(self.ir_data),
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
                0x80 => Register::Tsl2561Enable,
                0x81 => Register::Tsl2561Gain,
                0x8C => Register::Tsl2561Full,
                0x8E => Register::Tsl2561IR,
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

    fn new_i2c_mock(ir_data: u16, full_data: u16) -> MockTSL2561 {

        MockTSL2561 {
            reg: Register::Tsl2561Enable,
            enabled: false,
            ir_data: ir_data.to_be(),
            full_data: full_data.to_be(),
        }
    }

    fn make_dev(i2cdev: MockTSL2561) -> TSL2561LuminositySensor<MockTSL2561> {
        TSL2561LuminositySensor::new(i2cdev).unwrap()
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
    fn test_basic_lux_read() {
        let i2cdev = new_i2c_mock(27898, 0);
        let mut dev = make_dev(i2cdev);
        //assert_eq!(dev.temperature_celsius().unwrap(), 15.0);
    }

    #[test]
    fn test_zero_ir_read() {
        let i2cdev = new_i2c_mock(0, 0);
        let mut dev = make_dev(i2cdev);
        //assert_eq!(dev.temperature_celsius().unwrap(), -139.2);
    }

    #[test]
    fn test_max_ir_read() {
        let i2cdev = new_i2c_mock(u16::max_value(), 0);
        let mut dev = make_dev(i2cdev);
        //assert_eq!(dev.temperature_celsius().unwrap(), -139.2);
    }
}
