//! All device constants used in the driver, mostly register addresses.
//!
//! NOTE: Earlier revisions of Datasheet and Register Map has way more info about interrupt usage,
//! particularly rev 3.2
//!
//! #### Sources:
//! * Register map (rev 3.2): https://arduino.ua/docs/RM-MPU-6000A.pdf
//! * Datasheet (rev 3.2): https://www.cdiweb.com/datasheets/invensense/ps-mpu-6000a.pdf

/// Gyro Sensitivity
///
/// Measurements are scaled like this:
/// x * range/2**(resolution-1) or x / (2**(resolution-1) / range)
///
/// Sources:
///     * https://www.nxp.com/docs/en/application-note/AN3461.pdf
///     * https://theccontinuum.com/2012/09/24/arduino-imu-pitch-roll-from-accelerometer/
///     * https://makersportal.com/blog/2019/8/17/arduino-mpu6050-high-frequency-accelerometer-and-gyroscope-data-saver#accel_test
///     * https://github.com/kriswiner/MPU6050/wiki/2014-Invensense-Developer%27s-Conference
///     * rust MPU9250 driver on github
pub const GYRO_SENS: (f32, f32, f32, f32, f32, f32, f32, f32) =
    (2048., 1024., 512., 256., 128., 64., 32., 16.);

/// Accelerometer Sensitivity
///
/// Measurements are scaled like this:
///
/// x * range/2**(resolution-1) or x / (2**(resolution-1) / range)
/// Sources:
///     * https://www.nxp.com/docs/en/application-note/AN3461.pdf
///     * https://theccontinuum.com/2012/09/24/arduino-imu-pitch-roll-from-accelerometer/
///     * https://makersportal.com/blog/2019/8/17/arduino-mpu6050-high-frequency-accelerometer-and-gyroscope-data-saver#accel_test
///     * https://github.com/kriswiner/MPU6050/wiki/2014-Invensense-Developer%27s-Conference
///     * rust MPU9250 driver on github
pub const ACCEL_SENS: (f32, f32, f32, f32) = (16384., 8192., 4096., 2048.);
/// Temperature Sensitivity
pub const TEMP_SENSITIVITY: f32 = 256.;

/// Low Byte Register Gyro x orientation
pub const GYRO_REGX_L: u8 = 0x3b;
/// Low Byte Register Gyro y orientation
pub const GYRO_REGY_L: u8 = 0x3d;
/// Low Byte Register Gyro z orientation
pub const GYRO_REGZ_L: u8 = 0x3f;
/// Low Byte Register Calc roll
pub const ACC_REGX_L: u8 = 0x35;
/// Low Byte Register Calc pitch
pub const ACC_REGY_L: u8 = 0x37;
/// Low Byte Register Calc yaw
pub const ACC_REGZ_L: u8 = 0x39;
/// Low Byte Register Temperature
pub const TEMP_OUT_L: u8 = 0x33;
/// Slave address of Qmi8658
pub const DEFAULT_SLAVE_ADDR: u8 = 0x6b;
/// Device Identifier
pub const WHOAMI_ID: u8 = 0x05;
/// Internal register to check slave addr
pub const WHOAMI: u8 = 0x00;
/// Soft Reset Register
pub const RESET: u8 = 0x60;

/// Describes a bit block from bit number 'bit' to 'bit'+'length'
pub struct BitBlock {
    pub bit: u8,
    pub length: u8,
}

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]
/// Register 2: Serial Interface and Sensor Enable
pub struct CTRL1;

impl CTRL1 {
    /// Base Address
    pub const ADDR: u8 = 0x02;
    /// Enables 4-wire / 3-wire (0/1) SPI interface
    pub const SIM: u8 = 7;
    /// Serial interface (SPI, I2C, I3C) address non-increment / auto increment.
    pub const ADDR_AI: u8 = 6;
    /// Serial interface (SPI, I2C, I3C) read data Little-Endian / Big-Endian
    pub const BE: u8 = 5;
    /// INT2 pin output is enabled
    pub const INT2_EN: u8 = 4;
    /// INT1 pin output is enabled
    pub const INT1_EN: u8 = 3;
    /// FIFO interrupt is mapped to INT2 / INT1 pin
    pub const FIFO_INT_SEL: u8 = 2;
    /// Disable internal high-speed oscillator
    pub const SENSOR_DISABLE: u8 = 0;
}

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]
/// Register 3: Accelerometer Settings
pub struct CTRL2;

impl CTRL2 {
    /// Base Address
    pub const ADDR: u8 = 0x03;
    /// Accel self test bit
    pub const A_ST: u8 = 7;
    /// Accel Config FS_SEL
    pub const A_FS_SEL: BitBlock = BitBlock { bit: 6, length: 3 };
    /// Accel output data rate (ODR)
    pub const A_ODR: BitBlock = BitBlock { bit: 3, length: 4 };
}

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]
/// Register 4: Gyroscope Settings
pub struct CTRL3;

impl CTRL3 {
    pub const ADDR: u8 = 0x04;
    /// Gyro self test bit
    pub const G_ST: u8 = 7;
    /// Gyro Config FS_SEL
    pub const G_FS_SEL: BitBlock = BitBlock { bit: 6, length: 3 };
    /// Gyro output data rate (ODR)
    pub const G_ODR: BitBlock = BitBlock { bit: 3, length: 4 };
}

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]
/// Register 6: Sensor Data Processing Settings
pub struct CTRL5;

impl CTRL5 {
    /// Base Address
    pub const ADDR: u8 = 0x06;
    /// Gyro low-pass filter mode
    pub const G_LPF_MODE: BitBlock = BitBlock { bit: 6, length: 2 };
    /// Enable Gyroscope Low-Pass Filter with the mode given by G_LPF_MODE
    pub const G_LPF_EN: u8 = 4;
    /// Accel low-pass filter mode
    pub const A_LPF_MODE: BitBlock = BitBlock { bit: 2, length: 2 };
    /// Enable Accelerometer Low-Pass Filter with the mode given by A_LPF_MODE
    pub const A_LPF_EN: u8 = 0;
}

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]
/// Register 8: Enable Sensors and Configure Data Reads
pub struct CTRL7;

impl CTRL7 {
    /// Base Address
    pub const ADDR: u8 = 0x08;
    /// Enable SyncSample mode
    pub const SYNC_SAMPLE_EN: u8 = 7;
    /// DRDY(Data Ready) is disabled, is blocked from the INT2 pin
    pub const DRDY_DIS: u8 = 5;
    /// Gyroscope in Snooze Mode (only Drive enabled).
    pub const G_SN: u8 = 4;
    /// Enable Gyroscope
    pub const G_EN: u8 = 1;
    /// Enable Accelerometer
    pub const A_EN: u8 = 0;
}

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]
/// Register 9: Motion Detection Control
pub struct CTRL8;

impl CTRL8 {
    /// Base Address
    pub const ADDR: u8 = 0x09;
    /// use INT1 / STATUSINT.bit7 as CTRL9 handshake
    pub const HANDSHAKE_TYPE: u8 = 7;
    /// INT2 / INT1 is used for Activity Detection event interrupt
    pub const ACTIVITY_INT_SEL: u8 = 6;
    /// enable Pedometer engine
    pub const PEDO_EN: u8 = 4;
    /// enable Significant Motion engine
    pub const SIG_MOTION_EN: u8 = 3;
    /// enable No Motion engine
    pub const NO_MOTION_EN: u8 = 2;
    /// enable Any Motion engine
    pub const ANY_MOTION_EN: u8 = 1;
    /// enable Tap engine
    pub const TAP_EN: u8 = 0;
}

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]
/// Register 45: Sensor Data Available and Lock Register
pub struct STATUS_INT;

impl STATUS_INT {
    /// Base Address
    pub const ADDR: u8 = 0x2d;
    /// Indicates CTRL9 Command was done
    pub const CMD_DONE: u8 = 7;
    /// if SYNC_SAMPLE_EN: Sensor Data is locked else: INT1 level
    pub const LOCKED: u8 = 1;
    /// if SYNC_SAMPLE_EN: Sensor Data is available for reading else: INT2 level
    pub const AVAIL: u8 = 0;
}

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]
/// Register 46: Output Data Status Register
pub struct STATUS0;

impl STATUS0 {
    /// Base Address
    pub const ADDR: u8 = 0x2e;
    /// Gyroscope new data available
    pub const G_DA: u8 = 1;
    /// Accelerometer new data available
    pub const A_DA: u8 = 0;
}

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]
/// Register 47: Miscellaneous Status
pub struct STATUS1;

impl STATUS1 {
    /// Base Address
    pub const ADDR: u8 = 0x2f;
    /// Significant-Motion was detected
    pub const SIG_MOTION: u8 = 7;
    /// No-Motion was detected
    pub const NO_MOTION: u8 = 6;
    /// Any-Motion was detected
    pub const ANY_MOTION: u8 = 5;
    /// step was detected
    pub const PEDOMETER: u8 = 4;
    /// WoM was detected
    pub const WOM: u8 = 2;
    /// Tap was detected
    pub const TAP: u8 = 1;
}

/// Defines accelerometer range/sensivity
#[derive(Debug, Eq, PartialEq, Copy, Clone, Default)]
pub enum AccelRange {
    /// 2G
    #[default]
    G2 = 0,
    /// 4G
    G4,
    /// 8G
    G8,
    /// 16G
    G16,
}

/// Defines gyro range/sensitivity
#[derive(Debug, Eq, PartialEq, Copy, Clone, Default)]
pub enum GyroRange {
    /// 16 degrees
    #[default]
    D16 = 0,
    /// 32 degrees
    D32,
    /// 64 degrees
    D64,
    /// 128 degrees
    D128,
    /// 256 degrees
    D256,
    /// 512 degrees
    D512,
    /// 1024 degrees
    D1024,
    /// 2048 degrees
    D2048,
}

impl From<u8> for GyroRange {
    fn from(range: u8) -> Self {
        match range {
            0 => GyroRange::D16,
            1 => GyroRange::D32,
            2 => GyroRange::D64,
            3 => GyroRange::D128,
            4 => GyroRange::D256,
            5 => GyroRange::D512,
            6 => GyroRange::D1024,
            7 => GyroRange::D2048,
            _ => GyroRange::D16,
        }
    }
}

impl From<u8> for AccelRange {
    fn from(range: u8) -> Self {
        match range {
            0 => AccelRange::G2,
            1 => AccelRange::G4,
            2 => AccelRange::G8,
            3 => AccelRange::G16,
            _ => AccelRange::G2,
        }
    }
}

impl AccelRange {
    // Converts accelerometer range to correction/scaling factor, see register sheet
    pub(crate) fn sensitivity(&self) -> f32 {
        match &self {
            AccelRange::G2 => ACCEL_SENS.0,
            AccelRange::G4 => ACCEL_SENS.1,
            AccelRange::G8 => ACCEL_SENS.2,
            AccelRange::G16 => ACCEL_SENS.3,
        }
    }
}

impl GyroRange {
    // Converts gyro range to correction/scaling factor, see register sheet
    pub(crate) fn sensitivity(&self) -> f32 {
        match &self {
            GyroRange::D16 => GYRO_SENS.0,
            GyroRange::D32 => GYRO_SENS.1,
            GyroRange::D64 => GYRO_SENS.2,
            GyroRange::D128 => GYRO_SENS.3,
            GyroRange::D256 => GYRO_SENS.4,
            GyroRange::D512 => GYRO_SENS.5,
            GyroRange::D1024 => GYRO_SENS.6,
            GyroRange::D2048 => GYRO_SENS.7,
        }
    }
}

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
/// Output Data Rate.
pub enum ODR_HZ {
    /// N/A 7174.4 Hz
    _NA_7174d4 = 0,
    /// N/A 3587.2 Hz
    _NA_3587d2 = 1,
    /// N/A 1793.6 Hz
    _NA_1793d6 = 2,
    /// 1000 Hz 896.8 Hz
    _1000_896d8 = 3,
    /// 500 Hz 448.4 Hz
    _500_448d4 = 4,
    /// 250 Hz 224.2 Hz
    _250_224d2 = 5,
    /// 125 Hz 112.1 Hz
    _125_112d1 = 6,
    /// 62.5 Hz 56.05 Hz
    _62d5_56d05 = 7,
    /// 31.25 Hz 28.025 Hz
    _31d25_28d025 = 8,
    /// 128 Hz N/A
    _128_NA = 12,
    /// 21 Hz N/A
    _21_NA = 13,
    /// 11 Hz N/A
    _11_NA = 14,
    /// 3 Hz N/A
    _3_NA = 15,
}

impl From<u8> for ODR_HZ {
    fn from(range: u8) -> Self {
        match range {
            0 => ODR_HZ::_NA_7174d4,
            1 => ODR_HZ::_NA_3587d2,
            2 => ODR_HZ::_NA_1793d6,
            3 => ODR_HZ::_1000_896d8,
            4 => ODR_HZ::_500_448d4,
            5 => ODR_HZ::_250_224d2,
            6 => ODR_HZ::_125_112d1,
            7 => ODR_HZ::_62d5_56d05,
            8 => ODR_HZ::_31d25_28d025,
            12 => ODR_HZ::_128_NA,
            13 => ODR_HZ::_21_NA,
            14 => ODR_HZ::_11_NA,
            15 => ODR_HZ::_3_NA,
            _ => ODR_HZ::_NA_7174d4,
        }
    }
}

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
/// Low-Pass Filter
pub enum LPF_MODE {
    /// 2.66% of ODR
    _2_66_ODR = 0,
    /// 3.63% of ODR
    _3_63_ODR = 1,
    /// 5.39% of ODR
    _5_39_ODR = 2,
    /// 13.37% of ODR
    _13_37_ODR = 3,
}

impl From<u8> for LPF_MODE {
    fn from(range: u8) -> Self {
        match range {
            0 => LPF_MODE::_2_66_ODR,
            1 => LPF_MODE::_3_63_ODR,
            2 => LPF_MODE::_5_39_ODR,
            3 => LPF_MODE::_13_37_ODR,
            _ => LPF_MODE::_2_66_ODR,
        }
    }
}
