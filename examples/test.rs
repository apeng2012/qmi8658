use qmi8658::{device::*, *};

use i2cdev::linux::LinuxI2CError;
use linux_embedded_hal::{Delay, I2cdev};

fn main() -> Result<(), Qmi8658Error<LinuxI2CError>> {
    let i2c = I2cdev::new("/dev/i2c-1").map_err(Qmi8658Error::I2c)?;

    let mut delay = Delay;
    let mut mpu = Qmi8658::new(i2c);

    mpu.init()?;

    // Test power management
    println!("Test power management");

    // Test gyro config
    println!("Test gyro config");
    assert_eq!(mpu.get_gyro_range()?, GyroRange::D16);
    mpu.set_gyro_range(GyroRange::D512)?;
    assert_eq!(mpu.get_gyro_range()?, GyroRange::D512);

    // Test accel config
    println!("Test accel config");
    assert_eq!(mpu.get_accel_range()?, AccelRange::G2);
    mpu.set_accel_range(AccelRange::G4)?;
    assert_eq!(mpu.get_accel_range()?, AccelRange::G4);

    // accel_lpf
    println!("Test accel lpf");
    assert_eq!(mpu.get_accel_lpf()?, LPF_MODE::_2_66_ODR);
    mpu.set_accel_lpf(LPF_MODE::_3_63_ODR)?;
    assert_eq!(mpu.get_accel_lpf()?, LPF_MODE::_3_63_ODR);
    mpu.set_accel_lpf(LPF_MODE::_5_39_ODR)?;
    assert_eq!(mpu.get_accel_lpf()?, LPF_MODE::_5_39_ODR);
    mpu.set_accel_lpf(LPF_MODE::_13_37_ODR)?;
    assert_eq!(mpu.get_accel_lpf()?, LPF_MODE::_13_37_ODR);

    assert_ne!(mpu.get_temp()?, 36.53);

    // reset
    println!("Test reset");
    mpu.reset_device(&mut delay)?;
    assert_eq!(mpu.get_accel_lpf()?, LPF_MODE::_2_66_ODR);
    assert_eq!(mpu.get_accel_range()?, AccelRange::G2);
    assert_eq!(mpu.get_gyro_range()?, GyroRange::D16);

    println!("Test successful");
    Ok(())
}
