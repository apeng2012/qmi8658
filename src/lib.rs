//! # Qmi8658 sensor driver.
//!
//! `embedded_hal` based driver with i2c access to QMI8656
//!
//! ### Misc
//! * [Data sheet](https://www.qstcorp.com/upload/pdf/202301/13-52-25%20QMI8658A%20Datasheet%20Rev%20A.pdf)
//!
//! To use this driver you must provide a concrete `embedded_hal` implementation.
//! This example uses `linux_embedded_hal`.
//!
//! **More Examples** can be found [here](https://github.com/juliangaal/mpu8656/tree/master/examples).
//! ```no_run
//! use qmi8658::*;
//! use linux_embedded_hal::{I2cdev, Delay};
//! use i2cdev::linux::LinuxI2CError;
//!
//! fn main() -> Result<(), Qmi8658Error<LinuxI2CError>> {
//!     let i2c = I2cdev::new("/dev/i2c-1")
//!         .map_err(Qmi8658Error::I2c)?;
//!
//!     let mut mpu = Qmi8658::new(i2c);
//!
//!     mpu.init()?;
//!
//!     loop {
//!         // get roll and pitch estimate
//!         let acc = mpu.get_acc_angles()?;
//!         println!("r/p: {:?}", acc);
//!
//!         // get sensor temp
//!         let temp = mpu.get_temp()?;
//!         printlnasd!("temp: {:?}c", temp);
//!
//!         // get gyro data, scaled with sensitivity
//!         let gyro = mpu.get_gyro()?;
//!         println!("gyro: {:?}", gyro);
//!
//!         // get accelerometer data, scaled with sensitivity
//!         let acc = mpu.get_acc()?;
//!         println!("acc: {:?}", acc);
//!     }
//! }
//! ```

#![no_std]

mod bits;
pub mod device;

use crate::device::*;
use embedded_hal::{
    blocking::delay::DelayMs,
    blocking::i2c::{Write, WriteRead},
};
use libm::{atan2f, powf, sqrtf};
use nalgebra::{Vector2, Vector3};

/// PI, f32
pub const PI: f32 = core::f32::consts::PI;

/// PI / 180, for conversion to radians
pub const PI_180: f32 = PI / 180.0;

/// All possible errors in this crate
#[derive(Debug)]
pub enum Qmi8658Error<E> {
    /// I2C bus error
    I2c(E),

    /// Invalid chip ID was read
    InvalidChipId(u8),
}

/// Handles all operations on/with Qmi8658
pub struct Qmi8658<I> {
    i2c: I,
    slave_addr: u8,
    acc_range: Option<AccelRange>, // None: not used
    acc_sensitivity: f32,
    gyro_range: Option<GyroRange>, // None: not used
    gyro_sensitivity: f32,
}

impl<I, E> Qmi8658<I>
where
    I: Write<Error = E> + WriteRead<Error = E>,
{
    /// Side effect free constructor with default sensitivies, no calibration
    pub fn new(i2c: I) -> Self {
        Qmi8658 {
            i2c,
            slave_addr: DEFAULT_SLAVE_ADDR,
            acc_range: Some(AccelRange::G2),
            acc_sensitivity: ACCEL_SENS.0,
            gyro_range: Some(GyroRange::D16),
            gyro_sensitivity: GYRO_SENS.0,
        }
    }

    /// custom sensitivity
    pub fn new_with_sens(i2c: I, arange: Option<AccelRange>, grange: Option<GyroRange>) -> Self {
        //let asens = ;
        Qmi8658 {
            i2c,
            slave_addr: DEFAULT_SLAVE_ADDR,
            acc_range: arange,
            acc_sensitivity: arange.unwrap_or_default().sensitivity(),
            gyro_range: grange,
            gyro_sensitivity: grange.unwrap_or_default().sensitivity(),
        }
    }

    /// Same as `new`, but the chip address can be specified (e.g. 0x69, if the A0 pin is pulled up)
    pub fn new_with_addr(i2c: I, slave_addr: u8) -> Self {
        Qmi8658 {
            i2c,
            slave_addr,
            acc_range: Some(AccelRange::G2),
            acc_sensitivity: ACCEL_SENS.0,
            gyro_range: Some(GyroRange::D16),
            gyro_sensitivity: GYRO_SENS.0,
        }
    }

    /// Combination of `new_with_sens` and `new_with_addr`
    pub fn new_with_addr_and_sens(
        i2c: I,
        slave_addr: u8,
        arange: Option<AccelRange>,
        grange: Option<GyroRange>,
    ) -> Self {
        Qmi8658 {
            i2c,
            slave_addr,
            acc_range: arange,
            acc_sensitivity: arange.unwrap_or_default().sensitivity(),
            gyro_range: grange,
            gyro_sensitivity: grange.unwrap_or_default().sensitivity(),
        }
    }

    /// Init wakes QMI8656 and verifies register addr, e.g. in i2c
    pub fn init(&mut self) -> Result<(), Qmi8658Error<E>> {
        self.verify()?;
        self.write_bit(CTRL1::ADDR, CTRL1::ADDR_AI, true)?;
        if let Some(arange) = self.acc_range {
            self.set_accel_range(arange)?;
            self.set_accel_odr(ODR_HZ::_3_NA)?;
            self.enable_accel(true)?;
        }
        if let Some(grange) = self.gyro_range {
            self.set_gyro_range(grange)?;
            self.set_gyro_odr(ODR_HZ::_31d25_28d025)?;
            self.enable_gyro(true)?;
        }
        Ok(())
    }

    /// Verifies device to address 0x68 with WHOAMI.addr() Register
    fn verify(&mut self) -> Result<(), Qmi8658Error<E>> {
        let address = self.read_byte(WHOAMI)?;
        if address != WHOAMI_ID {
            return Err(Qmi8658Error::InvalidChipId(address));
        }
        Ok(())
    }

    /// set gyro low pass filter mode
    pub fn set_gyro_lpf(&mut self, mode: LPF_MODE) -> Result<(), Qmi8658Error<E>> {
        self.write_bits(
            CTRL5::ADDR,
            CTRL5::G_LPF_MODE.bit,
            CTRL5::G_LPF_MODE.length,
            mode as u8,
        )?;

        Ok(())
    }

    /// get gyro low pass filter mode
    pub fn get_gyro_lpf(&mut self) -> Result<LPF_MODE, Qmi8658Error<E>> {
        let mode: u8 =
            self.read_bits(CTRL5::ADDR, CTRL5::G_LPF_MODE.bit, CTRL5::G_LPF_MODE.length)?;

        Ok(LPF_MODE::from(mode))
    }

    /// set accel low pass filter mode
    pub fn set_accel_lpf(&mut self, mode: LPF_MODE) -> Result<(), Qmi8658Error<E>> {
        self.write_bits(
            CTRL5::ADDR,
            CTRL5::A_LPF_MODE.bit,
            CTRL5::A_LPF_MODE.length,
            mode as u8,
        )?;

        Ok(())
    }

    /// get accel low pass filter mode
    pub fn get_accel_lpf(&mut self) -> Result<LPF_MODE, Qmi8658Error<E>> {
        let mode: u8 =
            self.read_bits(CTRL5::ADDR, CTRL5::A_LPF_MODE.bit, CTRL5::A_LPF_MODE.length)?;

        Ok(LPF_MODE::from(mode))
    }

    /// Set gyro range, and update sensitivity accordingly
    pub fn set_gyro_range(&mut self, range: GyroRange) -> Result<(), Qmi8658Error<E>> {
        self.write_bits(
            CTRL3::ADDR,
            CTRL3::G_FS_SEL.bit,
            CTRL3::G_FS_SEL.length,
            range as u8,
        )?;

        self.gyro_sensitivity = range.sensitivity();
        Ok(())
    }

    /// get current gyro range
    pub fn get_gyro_range(&mut self) -> Result<GyroRange, Qmi8658Error<E>> {
        let byte = self.read_bits(CTRL3::ADDR, CTRL3::G_FS_SEL.bit, CTRL3::G_FS_SEL.length)?;

        Ok(GyroRange::from(byte))
    }

    /// set accel range, and update sensitivy accordingly
    pub fn set_accel_range(&mut self, range: AccelRange) -> Result<(), Qmi8658Error<E>> {
        self.write_bits(
            CTRL2::ADDR,
            CTRL2::A_FS_SEL.bit,
            CTRL2::A_FS_SEL.length,
            range as u8,
        )?;

        self.acc_sensitivity = range.sensitivity();
        Ok(())
    }

    /// get current accel_range
    pub fn get_accel_range(&mut self) -> Result<AccelRange, Qmi8658Error<E>> {
        let byte = self.read_bits(CTRL2::ADDR, CTRL2::A_FS_SEL.bit, CTRL2::A_FS_SEL.length)?;

        Ok(AccelRange::from(byte))
    }

    /// set gyro ODR
    pub fn set_gyro_odr(&mut self, odr: ODR_HZ) -> Result<(), Qmi8658Error<E>> {
        self.write_bits(
            CTRL3::ADDR,
            CTRL3::G_ODR.bit,
            CTRL3::G_ODR.length,
            odr as u8,
        )?;

        Ok(())
    }

    /// get current gyro ODR
    pub fn get_gyro_odr(&mut self) -> Result<ODR_HZ, Qmi8658Error<E>> {
        let byte = self.read_bits(CTRL3::ADDR, CTRL3::G_ODR.bit, CTRL3::G_ODR.length)?;

        Ok(ODR_HZ::from(byte))
    }

    /// set accel ODR
    pub fn set_accel_odr(&mut self, odr: ODR_HZ) -> Result<(), Qmi8658Error<E>> {
        self.write_bits(
            CTRL2::ADDR,
            CTRL2::A_ODR.bit,
            CTRL2::A_ODR.length,
            odr as u8,
        )?;

        Ok(())
    }

    /// get current accel ODR
    pub fn get_accel_odr(&mut self) -> Result<ODR_HZ, Qmi8658Error<E>> {
        let byte = self.read_bits(CTRL2::ADDR, CTRL2::A_ODR.bit, CTRL2::A_ODR.length)?;

        Ok(ODR_HZ::from(byte))
    }

    /// set gyro en
    pub fn enable_gyro(&mut self, en: bool) -> Result<(), Qmi8658Error<E>> {
        self.write_bit(CTRL7::ADDR, CTRL7::G_EN, en)?;

        Ok(())
    }

    /// get current gyro en
    pub fn is_gyro_enable(&mut self) -> Result<bool, Qmi8658Error<E>> {
        let en = self.read_bit(CTRL7::ADDR, CTRL7::G_EN)?;

        Ok(en == 1)
    }

    /// set accel en
    pub fn enable_accel(&mut self, en: bool) -> Result<(), Qmi8658Error<E>> {
        self.write_bit(CTRL7::ADDR, CTRL7::A_EN, en)?;

        Ok(())
    }

    /// get current accel en
    pub fn is_accel_enable(&mut self) -> Result<bool, Qmi8658Error<E>> {
        let en = self.read_bit(CTRL7::ADDR, CTRL7::A_EN)?;

        Ok(en == 1)
    }

    /// reset device
    pub fn reset_device<D: DelayMs<u8>>(&mut self, delay: &mut D) -> Result<(), Qmi8658Error<E>> {
        self.write_byte(RESET, 0xb0)?;
        delay.delay_ms(30u8);
        Ok(())
    }

    /// Roll and pitch estimation from raw accelerometer readings
    /// NOTE: no yaw! no magnetometer present on QMI8656
    /// https://www.nxp.com/docs/en/application-note/AN3461.pdf equation 28, 29
    pub fn get_acc_angles(&mut self) -> Result<Vector2<f32>, Qmi8658Error<E>> {
        let acc = self.get_acc()?;

        Ok(Vector2::<f32>::new(
            atan2f(acc.y, sqrtf(powf(acc.x, 2.) + powf(acc.z, 2.))),
            atan2f(-acc.x, sqrtf(powf(acc.y, 2.) + powf(acc.z, 2.))),
        ))
    }

    /// Converts 2 bytes number in 2 compliment
    /// TODO i16?! whats 0x8000?!
    fn read_word_2c(&self, byte: &[u8]) -> i32 {
        let high: i32 = byte[1] as i32;
        let low: i32 = byte[0] as i32;
        let mut word: i32 = (high << 8) + low;

        if word >= 0x8000 {
            word = -((65535 - word) + 1);
        }

        word
    }

    /// Reads rotation (gyro/acc) from specified register
    fn read_rot(&mut self, reg: u8) -> Result<Vector3<f32>, Qmi8658Error<E>> {
        let mut buf: [u8; 6] = [0; 6];
        self.read_bytes(reg, &mut buf)?;

        Ok(Vector3::<f32>::new(
            self.read_word_2c(&buf[0..2]) as f32,
            self.read_word_2c(&buf[2..4]) as f32,
            self.read_word_2c(&buf[4..6]) as f32,
        ))
    }

    /// Accelerometer readings in g
    pub fn get_acc(&mut self) -> Result<Vector3<f32>, Qmi8658Error<E>> {
        let mut acc = self.read_rot(ACC_REGX_L)?;
        acc /= self.acc_sensitivity;

        Ok(acc)
    }

    /// Gyro readings in rad/s
    pub fn get_gyro(&mut self) -> Result<Vector3<f32>, Qmi8658Error<E>> {
        let mut gyro = self.read_rot(GYRO_REGX_L)?;

        gyro *= PI_180 / self.gyro_sensitivity;

        Ok(gyro)
    }

    /// Sensor Temp in degrees celcius
    pub fn get_temp(&mut self) -> Result<f32, Qmi8658Error<E>> {
        let mut buf: [u8; 2] = [0; 2];
        self.read_bytes(TEMP_OUT_L, &mut buf)?;

        Ok(buf[1] as f32 + (buf[0] as f32 / TEMP_SENSITIVITY))
    }

    /// Writes byte to register
    pub fn write_byte(&mut self, reg: u8, byte: u8) -> Result<(), Qmi8658Error<E>> {
        self.i2c
            .write(self.slave_addr, &[reg, byte])
            .map_err(Qmi8658Error::I2c)?;
        // delay disabled for dev build
        // TODO: check effects with physical unit
        // self.delay.delay_ms(10u8);
        Ok(())
    }

    /// Enables bit n at register address reg
    pub fn write_bit(&mut self, reg: u8, bit_n: u8, enable: bool) -> Result<(), Qmi8658Error<E>> {
        let mut byte: [u8; 1] = [0; 1];
        self.read_bytes(reg, &mut byte)?;
        bits::set_bit(&mut byte[0], bit_n, enable);
        self.write_byte(reg, byte[0])?;

        Ok(())
    }

    /// Write bits data at reg from start_bit to start_bit+length
    pub fn write_bits(
        &mut self,
        reg: u8,
        start_bit: u8,
        length: u8,
        data: u8,
    ) -> Result<(), Qmi8658Error<E>> {
        let mut byte: [u8; 1] = [0; 1];
        self.read_bytes(reg, &mut byte)?;
        bits::set_bits(&mut byte[0], start_bit, length, data);
        self.write_byte(reg, byte[0])?;
        Ok(())
    }

    /// Read bit n from register
    fn read_bit(&mut self, reg: u8, bit_n: u8) -> Result<u8, Qmi8658Error<E>> {
        let mut byte: [u8; 1] = [0; 1];
        self.read_bytes(reg, &mut byte)?;
        Ok(bits::get_bit(byte[0], bit_n))
    }

    /// Read bits at register reg, starting with bit start_bit, until start_bit+length
    pub fn read_bits(&mut self, reg: u8, start_bit: u8, length: u8) -> Result<u8, Qmi8658Error<E>> {
        let mut byte: [u8; 1] = [0; 1];
        self.read_bytes(reg, &mut byte)?;
        Ok(bits::get_bits(byte[0], start_bit, length))
    }

    /// Reads byte from register
    pub fn read_byte(&mut self, reg: u8) -> Result<u8, Qmi8658Error<E>> {
        let mut byte: [u8; 1] = [0; 1];
        self.i2c
            .write_read(self.slave_addr, &[reg], &mut byte)
            .map_err(Qmi8658Error::I2c)?;
        Ok(byte[0])
    }

    /// Reads series of bytes into buf from specified reg
    pub fn read_bytes(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Qmi8658Error<E>> {
        self.i2c
            .write_read(self.slave_addr, &[reg], buf)
            .map_err(Qmi8658Error::I2c)?;
        Ok(())
    }
}
