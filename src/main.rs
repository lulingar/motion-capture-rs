use core::time::Duration;
use std::time::Instant;
use anyhow::Result;
use esp_idf_svc::hal::{
    delay::FreeRtos,
    gpio::PinDriver,
    gpio::{Gpio20, Gpio21, Output},
    i2c::{I2cConfig, I2cDriver},
    peripherals::Peripherals,
    units::FromValueType,
};
//use esp_idf_svc::sys::EspError;
use esp_idf_svc::timer::EspTaskTimerService;
use mpu6050::{
    Mpu6050,
    Mpu6050Error,
    device::{ AccelRange, GyroRange, ACCEL_HPF },
    PI_180,
};
use imu_fusion::{Fusion, FusionAhrsSettings, FusionVector};

//use rand::prelude::*;

fn main() -> Result<()> {
    // It is necessary to call this function once. Otherwise, some patches to the runtime
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

    let mut flag_total = PinDriver::output(peripherals.pins.gpio21)?;
    let mut flag_compute = PinDriver::output(peripherals.pins.gpio20)?;
    flag_total.set_low()?;
    flag_compute.set_low()?;

    let mut delay = FreeRtos;
    let mut mpu = Mpu6050::new(i2c_imu);

    /* Supply chain issue:
     * https://forum.arduino.cc/t/mpu-6050-a-module-problems-who-am-i-reports-0x98-not-0x68-as-it-should-fake-mpu-6050/861956/20
     */
    match mpu.init(&mut delay) {
        Ok(_) => (),
        Err(error) => match error {
            Mpu6050Error::I2c(_) => log::error!("I2C initialization failed."),
            Mpu6050Error::InvalidChipId(read) => log::warn!("Maybe counterfeit chip? GottenID {:#x}.", read),
        }
    };

    mpu.set_accel_range(AccelRange::G4).unwrap();
    mpu.set_gyro_range(GyroRange::D250).unwrap();
    mpu.set_accel_hpf(ACCEL_HPF::_RESET).unwrap();

    /*
    loop {
        let mut tic = Instant::now();
        let toc = Instant::now();
        //let dt = toc - tic;
        tic = toc;
    }
    */
    const IMU_SAMPLE_PERIOD: Duration = Duration::from_millis(5);
    const IMU_SAMPLE_FREQ: f32 = 1000_f32 / IMU_SAMPLE_PERIOD.subsec_millis() as f32;
    let ahrs_settings = FusionAhrsSettings::new();
    let mut fusion = Fusion::new(IMU_SAMPLE_FREQ as u32, ahrs_settings);
    let mut pos = FusionVector::zero();
    let mut vel = FusionVector::zero();
    let start_time = Instant::now();

    let timer_service = EspTaskTimerService::new().unwrap();
    let callback_timer = timer_service.timer(move || acq_cmp(
        &mut mpu,
        &mut fusion,
        start_time,
        &IMU_SAMPLE_PERIOD,
        &mut pos, &mut vel,
        &mut flag_compute,
        &mut flag_total,
    ))?;
    callback_timer.every(IMU_SAMPLE_PERIOD).unwrap();

    loop {
        FreeRtos::delay_ms(100);
    }
}

fn acq_cmp(
    mpu: &mut Mpu6050<I2cDriver>,
    fusion: &mut Fusion,
    start_time: Instant,
    sampling_period: &Duration,
    pos_vector: &mut FusionVector,
    vel_vector: &mut FusionVector,
    flag_compute: &mut PinDriver<Gpio20, Output>,
    flag_total: &mut PinDriver<Gpio21, Output>,
) {
    flag_total.set_high().unwrap();
    flag_compute.set_high().unwrap();
    let ts = start_time.elapsed().as_secs_f32();

    // Gets acceleration in units of earth gravity
    let acc = mpu.get_acc().unwrap();
    // Gets angular rotation in degrees/sec
    let gyr = mpu.get_gyro().unwrap() / PI_180;

    let gyro_vector = FusionVector::new(gyr.x, gyr.y, gyr.z);
    let acceleration_vector = FusionVector::new(acc.x, acc.y, acc.z);
    fusion.update_no_mag(gyro_vector, acceleration_vector, ts);
    // Gets heading in units of degrees
    let euler = fusion.euler();
    let earth_acc = fusion.earth_acc();
    *vel_vector += earth_acc * 9.81 * sampling_period.as_secs_f32();
    *pos_vector += *vel_vector * sampling_period.as_secs_f32();
    flag_compute.set_low().unwrap();

    /*
    println!("ts: {:5.5}, pitch:{:+.3} roll:{:+.3} yaw:{:+.3}, px:{:+.3}, py:{:+.3}, pz:{:+.3}",
             ts,
             euler.angle.pitch, euler.angle.roll, euler.angle.yaw,
             pos_vector.x, pos_vector.y, pos_vector.z);
    */
    println!("ts: {:5.5}, pitch:{:+.3} roll:{:+.3} yaw:{:+.3} ae_x:{:+.3} ae.y:{:+.3} ae.z:{:+.3}",
             ts,
             euler.angle.pitch, euler.angle.roll, euler.angle.yaw,
             earth_acc.x, earth_acc.y, earth_acc.z,
    );
    flag_total.set_low().unwrap();
}
