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
    Auto = 0xff,
    Low = 0x00,
    High = 0x10,
}

///
/// Integration times mode
///
#[derive(Eq, PartialEq, Copy, Clone, Debug)]
pub enum IntegrationTime {
    _13ms = 0xff,
    _101ms = 0x01,
    _402ms = 0x02,
}

#[derive(Eq, PartialEq, Copy, Clone, Debug)]
enum AGCHigh {
    _13ms = 4850,
    _101ms = 36000,
    _402ms = 63000,
}

#[derive(Eq, PartialEq, Copy, Clone, Debug)]
enum AGCLow {
    _13ms = 100,
    _101ms = 200,
    _402ms = 500,
}

#[derive(Eq, PartialEq, Copy, Clone, Debug)]
enum Clipping {
    _13ms = 4900,
    _101ms = 37000,
    _402ms = 65000,
}

enum LuxScale {
    Lux = 14,
    Ratio = 9,
    Channel = 10,
    ChannelTint0 = 0x7517,
    ChannelTint1 = 0x0FE7,
}

enum LuxModifier {
    K1t = 0x0040, // 0.125 * 2^RATIO_SCALE
    B1t = 0x01f2, // 0.0304 * 2^LUX_SCALE
    M1t = 0x01be, // 0.0272 * 2^LUX_SCALE
    K2t = 0x0080, // 0.250 * 2^RATIO_SCALE
    B2t = 0x0214, // 0.0325 * 2^LUX_SCALE
    M2t = 0x02d1, // 0.0440 * 2^LUX_SCALE
    K3t = 0x00c0, // 0.375 * 2^RATIO_SCALE
    B3t = 0x023f, // 0.0351 * 2^LUX_SCALE
    M3t = 0x037b, // 0.0544 * 2^LUX_SCALE
    K4t = 0x0100, // 0.50 * 2^RATIO_SCALE
    B4t = 0x0270, // 0.0381 * 2^LUX_SCALE
    M4t = 0x03fe, // 0.0624 * 2^LUX_SCALE
    K5t = 0x0138, // 0.61 * 2^RATIO_SCALE
    B5t = 0x016f, // 0.0224 * 2^LUX_SCALE
    M5t = 0x01fc, // 0.0310 * 2^LUX_SCALE
    K6t = 0x019a, // 0.80 * 2^RATIO_SCALE
    B6t = 0x00d2, // 0.0128 * 2^LUX_SCALE
    M6t = 0x00fb, // 0.0153 * 2^LUX_SCALE
    K7t = 0x029a, // 1.3 * 2^RATIO_SCALE
    B7t = 0x0018, // 0.00146 * 2^LUX_SCALE
    M7t = 0x0012, // 0.00112 * 2^LUX_SCALE
    //K8t = 0x029a, // 1.3 * 2^RATIO_SCALE
    B8t = 0x0000, // 0.000 * 2^LUX_SCALE
                  //M8t = 0x0000, // 0.000 * 2^LUX_SCALE
}

// -------------------------------------------------------------------------------

pub const TSL2561_COMMAND_BIT: u8 = 0x80;
pub const TSL2561_WORD_BIT: u8 = 0x20;


// TSL2561 register addresses
enum Register {
    Tsl2561Control = 0x00,
    Tsl2561Enable = 0x80,
    Tsl2561Full = 0x8C,
    Tsl2561IR = 0x8E,
    Tsl2561Timing = 0x01,
    Tsl2561Data0 = 0x0C, // Data0/DataLow
    Tsl2561Data1 = 0x0D, // Data1/DataHigh
    Tsl2561Id = 0x0A,
}

// TSL2561 command register addresses
enum Command {
    Tsl2561PowerOn = 0x03,
    Tsl2561PowerOff = 0x00,
}



// -------------------------------------------------------------------------------


// ---------------------------------------------------------------------------------

///
/// Enable device (Power on)
///
fn enable_dev<E: Error>(dev: &mut I2CDevice<Error = E>) -> Result<(), E> {
    dev.smbus_write_byte_data(TSL2561_COMMAND_BIT | Register::Tsl2561Control as u8,
                               Command::Tsl2561PowerOn as u8)?;
    Ok(())
}

///
/// Disable device (Power off)
///
fn disable_dev<E: Error>(dev: &mut I2CDevice<Error = E>) -> Result<(), E> {
    dev.smbus_write_byte_data(TSL2561_COMMAND_BIT | Register::Tsl2561Control as u8,
                               Command::Tsl2561PowerOff as u8)?;
    Ok(())
}

///
/// Set gain.
///
fn set_gain<E: Error>(dev: &mut I2CDevice<Error = E>,
                      gain: &Gain,
                      integration_time: &IntegrationTime)
                      -> Result<(), E> {
    let timing = (*integration_time as u8) | (*gain as u8);
    let r = dev.smbus_write_byte_data(TSL2561_COMMAND_BIT | Register::Tsl2561Timing as u8, timing)?;
    Ok(r)
}


///
/// Reads the raw full (IR + Lux) data...
///
fn read_raw_full<E: Error>(dev: &mut I2CDevice<Error = E>) -> Result<u16, E> {
    let full = dev.smbus_read_word_data(TSL2561_COMMAND_BIT | TSL2561_WORD_BIT |
                                        Register::Tsl2561Full as u8)?;
    Ok(full)
}

///
/// Reads the raw infrafred data...
///
fn read_raw_ir<E: Error>(dev: &mut I2CDevice<Error = E>) -> Result<u16, E> {
    let ir = dev.smbus_read_word_data(TSL2561_COMMAND_BIT | TSL2561_WORD_BIT |
                                      Register::Tsl2561IR as u8)?;
    Ok(ir)
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
    pub fn new(dev: T) -> Result<TSL2561LuminositySensor<T>, T::Error> {
        Ok(TSL2561LuminositySensor { dev: dev })
    }

    ///
    /// Read data from the sensors according to the params
    ///
    fn read_data(&mut self, gain: Gain, time: &IntegrationTime) -> Result<(u32, u32), T::Error> {
        let delay = match *time {
            IntegrationTime::_13ms => Duration::from_millis(15),
            IntegrationTime::_101ms => Duration::from_millis(120),
            IntegrationTime::_402ms => Duration::from_millis(450),
        };

        let (low, high) = match *time {
            IntegrationTime::_13ms => (AGCLow::_13ms as u16, AGCHigh::_13ms as u16),
            IntegrationTime::_101ms => (AGCLow::_101ms as u16, AGCHigh::_101ms as u16),
            IntegrationTime::_402ms => (AGCLow::_402ms as u16, AGCHigh::_402ms as u16),
        };


        let mut _gain = match gain {
            Gain::Auto | Gain::High => Gain::High,
            _ => Gain::Low,
        };

        enable_dev(&mut self.dev)?;
        let _ambient = read_raw_full(&mut self.dev)?;

        let ambient = if (gain == Gain::Auto) {
            if (_ambient >= high) {
                _gain = Gain::Low;
            } else if (_ambient <= low) {
                _gain = Gain::High;
            }
            set_gain(&mut self.dev, &_gain, &time)?;
            read_raw_full(&mut self.dev)?
        } else {
            _ambient
        };

        let IR = read_raw_ir(&mut self.dev)? as u32;
        let ambient = ambient as u32;
        disable_dev(&mut self.dev)?;
        Ok((IR, ambient))
    }
}

pub trait LuminositySensor {
    type Error;

    ///
    /// Reads the ambient light in lux from the sensor. Uses
    /// power management to enable/disable the device before / after
    /// reading.
    ///
    fn read_lux(&mut self, gain: Gain, time: IntegrationTime) -> Result<u32, Self::Error>;
}

impl<T> LuminositySensor for TSL2561LuminositySensor<T>
    where T: I2CDevice + Sized
{
    type Error = T::Error;


    fn read_lux(&mut self, gain: Gain, time: IntegrationTime) -> Result<u32, T::Error> {

        let (IR, ambient) = self.read_data(gain, &time)?;

        let (threshold, scale) = match time {
            IntegrationTime::_13ms => (Clipping::_13ms as u32, LuxScale::ChannelTint0 as u64),
            IntegrationTime::_101ms => (Clipping::_101ms as u32, LuxScale::ChannelTint1 as u64),
            IntegrationTime::_402ms => {
                (Clipping::_402ms as u32, (1 << LuxScale::Channel as u32) as u64)
            }
        } as (u32, u64);

        if IR > threshold || ambient > threshold {
            return Ok(u32::max_value());
        }

        let scale = match gain {
            Gain::Low => scale << 4u32,
            _ => scale,
        } as u32;

        let c0 = (ambient * scale) >> LuxScale::Channel as u32;
        let c1 = (IR * scale) >> LuxScale::Channel as u32;
        let ratio = if c0 != 0 {
            ((c1 << (LuxScale::Ratio as u32 + 1)) as f32 / c0 as f32).round() as u32
        } else {
            0u32
        };

        let (b, m) = if ratio >= 0 && ratio <= LuxModifier::K1t as u32 {
            (LuxModifier::B1t as u32, LuxModifier::M1t as u32)
        } else if (ratio <= LuxModifier::K2t as u32) {
            (LuxModifier::B2t as u32, LuxModifier::M2t as u32)
        } else if (ratio <= LuxModifier::K3t as u32) {
            (LuxModifier::B3t as u32, LuxModifier::M3t as u32)
        } else if (ratio <= LuxModifier::K4t as u32) {
            (LuxModifier::B4t as u32, LuxModifier::M4t as u32)
        } else if (ratio <= LuxModifier::K5t as u32) {
            (LuxModifier::B5t as u32, LuxModifier::M5t as u32)
        } else if (ratio <= LuxModifier::K6t as u32) {
            (LuxModifier::B6t as u32, LuxModifier::M6t as u32)
        } else if (ratio <= LuxModifier::K7t as u32) {
            (LuxModifier::B7t as u32, LuxModifier::M7t as u32)
        } else {
            (LuxModifier::B8t as u32, LuxModifier::B8t as u32)
        };

        println!("ambient={}, IR={}", ambient, IR);

        let lux: u32 = {
            let t: u32 = (c0 * b) - (c1 * m);
            let mut t: u32 = if t < 0 { 0 } else { t };
            t = t + (1 << LuxScale::Lux as u32 - 1);
            t >> LuxScale::Lux as u32
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
        // assert_eq!(dev.temperature_celsius().unwrap(), 15.0);
    }

    #[test]
    fn test_zero_ir_read() {
        let i2cdev = new_i2c_mock(0, 0);
        let mut dev = make_dev(i2cdev);
        // assert_eq!(dev.temperature_celsius().unwrap(), -139.2);
    }

    #[test]
    fn test_max_ir_read() {
        let i2cdev = new_i2c_mock(u16::max_value(), 0);
        let mut dev = make_dev(i2cdev);
        // assert_eq!(dev.temperature_celsius().unwrap(), -139.2);
    }
}
