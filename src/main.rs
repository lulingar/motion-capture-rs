use anyhow::Result;
use esp_idf_svc::hal::{
    delay::FreeRtos,
    gpio::PinDriver,
    i2c::{I2cConfig, I2cDriver},
    peripherals::Peripherals,
    units::FromValueType,
};
use mpu6050_dmp::{
    address::Address,
    sensor::Mpu6050,
    quaternion::Quaternion,
    yaw_pitch_roll::YawPitchRoll,
};

//use rand::prelude::*;

fn main() -> Result<()> {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("Hello, world!");

    let peripherals = Peripherals::take().unwrap();

    let i2c_config = I2cConfig::new().baudrate(400.kHz().into());
    let sda_imu = peripherals.pins.gpio1;
    let scl_imu = peripherals.pins.gpio0;
    let i2c_imu = I2cDriver::new(peripherals.i2c0, sda_imu, scl_imu, &i2c_config)?;

    let mut flag = PinDriver::output(peripherals.pins.gpio21)?;
    flag.set_low()?;

    let mut delay = FreeRtos;
    let mut sensor = Mpu6050::new(i2c_imu, Address::default()).unwrap();
    log::info!("Sensor Initialized");
    sensor.initialize_dmp(&mut delay).unwrap();
    log::info!("DMP Initialized");

    loop {
        let len = sensor.get_fifo_count().unwrap();
        if len > 0 {
            log::info!("Buflen is {}", len);
        }
        if len >= 28 {
            let mut buf = [0; 28];
            let buf = sensor.read_fifo(&mut buf).unwrap();
            let quat = Quaternion::from_bytes(&buf[..16]).unwrap();
            let ypr = YawPitchRoll::from(quat);
            log::info!("{:.5?}; {:.5?}; {:.5?};", ypr.yaw, ypr.pitch, ypr.roll);
            let _ = flag.toggle();
        }
    }
}
