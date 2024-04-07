use anyhow::Result;
use std::time::Instant;
use esp_idf_svc::hal::{
    delay::FreeRtos,
    gpio::PinDriver,
    i2c::{I2cConfig, I2cDriver},
    peripherals::Peripherals,
    units::FromValueType,
};
use mpu6050::{
    Mpu6050,
    Mpu6050Error,
    device::{ AccelRange, GyroRange, ACCEL_HPF },
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
    let mut mpu = Mpu6050::new(i2c_imu);

    /* Supply chain issue:
     * https://forum.arduino.cc/t/mpu-6050-a-module-problems-who-am-i-reports-0x98-not-0x68-as-it-should-fake-mpu-6050/861956/20
     */
    match mpu.init(&mut delay) {
        Ok(_) => (),
        Err(error) => match error {
            Mpu6050Error::I2c(_) => log::error!("I2C initalization failed."),
            Mpu6050Error::InvalidChipId(read) => log::warn!("Maybe counterfeit chip? GottenID {:#x}.", read),
        }
    };

    mpu.set_accel_range(AccelRange::G4).unwrap();
    mpu.set_gyro_range(GyroRange::D250).unwrap();
    mpu.set_accel_hpf(ACCEL_HPF::_RESET).unwrap();

    let mut tic = Instant::now();
    loop {
        let toc = Instant::now();
        //let dt = toc - tic;
        tic = toc;
        flag.set_high()?;
        let acc = mpu.get_acc().unwrap();
        flag.set_low()?;
        let gyr = mpu.get_gyro().unwrap();
        /*
        println!("dt: {:?} acc: {:+.3} {:+.3} {:+.3} gyro: {:+.3} {:+.3} {:+.3}",
        dt, acc.x, acc.y, acc.z, gyr.x, gyr.y, gyr.z);
        */
        acc + gyr;
    }
}
