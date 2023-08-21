use i2cdev::linux::LinuxI2CError;
use linux_embedded_hal::I2cdev;
use qmi8658::*;

fn main() -> Result<(), Qmi8658Error<LinuxI2CError>> {
    let i2c = I2cdev::new("/dev/i2c-1").map_err(Qmi8658Error::I2c)?;

    let mut mpu = Qmi8658::new(i2c);

    mpu.init()?;

    loop {
        // get roll and pitch estimate
        let acc = mpu.get_acc_angles()?;
        println!("r/p: {:?}", acc);

        // get sensor temp
        let temp = mpu.get_temp()?;
        println!("temp: {:?}c", temp);

        // get gyro data, scaled with sensitivity
        let gyro = mpu.get_gyro()?;
        println!("gyro: {:?}", gyro);

        // get accelerometer data, scaled with sensitivity
        let acc = mpu.get_acc()?;
        println!("acc: {:?}", acc);
    }
}
