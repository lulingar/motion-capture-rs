use core::time::Duration;
use std::num::NonZeroU32;
use std::time::Instant;

use anyhow::{anyhow, Result};
use esp_idf_svc::hal::spi::{SpiDeviceDriver, SpiDriverConfig};
use esp_idf_svc::hal::task::notification::Notification;
use esp_idf_svc::hal::{
    delay::FreeRtos,
    gpio::{AnyOutputPin, PinDriver},
    peripherals::Peripherals,
    units::FromValueType,
    //sys::EspError,
    spi::config::Config as SpiConfig,
};
use esp_idf_svc::timer::EspTaskTimerService;
use mpu9250::{ Mpu9250, MpuConfig };

use imu_fusion::{FusionMatrix, FusionVector};
mod imu_tracker;
use imu_tracker::ImuTracker;

// The constant `CONFIG` is auto-generated by `toml_config`.
#[toml_cfg::toml_config]
pub struct Config {
    #[default("localhost")]
    mqtt_host: &'static str,
    #[default("1883")]
    mqtt_port: &'static str,
    #[default("")]
    mqtt_user: &'static str,
    #[default("")]
    mqtt_pass: &'static str,
    #[default("")]
    mqtt_id: &'static str,
    #[default("")]
    wifi_ssid: &'static str,
    #[default("")]
    wifi_psk: &'static str,
}

fn main() -> Result<()> {
    // It is necessary to call this function once. Otherwise, some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take()?;

    let mut delay = FreeRtos;

    let sclk = peripherals.pins.gpio6;
    let miso = peripherals.pins.gpio2;
    let mosi = peripherals.pins.gpio7;
    let cs = PinDriver::output(peripherals.pins.gpio10)?;
    let spi = SpiDeviceDriver::new_single(
        peripherals.spi2,
        sclk,
        mosi,
        Some(miso),
        Option::<AnyOutputPin>::None,
        &SpiDriverConfig::new(),
        &SpiConfig::default().baudrate(1.MHz().into()),
    )?;
    let gyro_rate = mpu9250::GyroTempDataRate::DlpfConf(mpu9250::Dlpf::_0);
    let mut imu = Mpu9250::imu(
        spi,
        cs,
        &mut delay,
        MpuConfig::imu()
            .gyro_temp_data_rate(gyro_rate)
            .sample_rate_divisor(3)
    ).expect("Could not initialize IMU");

    let who_am_i = imu.who_am_i().expect("Could not read WHO_AM_I");
    println!("WHO_AM_I: 0x{:x}", who_am_i);

    let mut flag_serialize = PinDriver::output(peripherals.pins.gpio21)?;
    let mut flag_acquire = PinDriver::output(peripherals.pins.gpio20)?;
    flag_serialize.set_low()?;
    flag_acquire.set_low()?;

    // Configures the notification
    let notification = Notification::new();
    let notifier = notification.notifier();

    const IMU_SAMPLE_PERIOD: Duration = Duration::from_millis(5);

    let timer_service = EspTaskTimerService::new()?;
    let callback_timer = timer_service.timer(move || unsafe {
        notifier.notify_and_yield(NonZeroU32::new(1).unwrap());
    })?;
    callback_timer.every(IMU_SAMPLE_PERIOD)?;

    let mut last = Instant::now();
    let mut id = 0u32;

    /* Test calibration function. As expected, it cannot account for earth's gravity
    let offsets: [f32; 3] = imu.calibrate_at_rest(&mut delay).map_err(|err| anyhow!("Error: {:?}", err))?;
    println!("Offsets: {:?}", offsets);
    */
    let acc_misalignment = FusionMatrix::new(0.998154, 4.21399e-09, 1.36475e-09,
                                                        4.21466e-09, 0.997542, -2.99281e-09,
                                                        1.2859e-09, -3.01287e-09, 0.987841);
    let acc_offset = FusionVector::new(0.0246591f32, -0.00429982f32, 0.137597f32);
    let acc_sensitivity = FusionVector::ones();
    let gyr_offset = FusionVector::new(1.275, 1.902, -1.202);
    let mut tracker = ImuTracker::new(IMU_SAMPLE_PERIOD, Instant::now(), 2000.0f32,
                                      acc_misalignment, acc_sensitivity, acc_offset, gyr_offset);

    loop {
        notification.wait(esp_idf_svc::hal::delay::BLOCK);
        flag_acquire.set_high()?;
        /*
        let all = imu
            .unscaled_all::<[i16; 3]>()
            .expect("Unable to read from IMU!");
        */
        // TODO read up on the "turbofish" operator below
        let all = imu.all::<[f32; 3]>().map_err(|err| anyhow!("Error: {:?}", err))?;
        let now = Instant::now();
        flag_acquire.set_low()?;
        let imu_gyro = FusionVector::new(all.gyro[0], all.gyro[1],all.gyro[2]) * (180. / core::f32::consts::PI);
        let imu_accel = FusionVector::new(all.accel[0],all.accel[1],all.accel[2]) * (1. / mpu9250::G);
        tracker.update(now, imu_accel, imu_gyro);
        let usecs = now.duration_since(last).as_micros();
        last = now;
        if id % 20 == 0 {
            println!(
                "{},{},{},{},{},{},{},{},{}",
                id,
                usecs,
                all.accel[0],
                all.accel[1],
                all.accel[2],
                all.gyro[0],
                all.gyro[1],
                all.gyro[2],
                all.temp
            );
        }
        id += 1;
    }
}
