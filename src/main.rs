mod imu_tracker;

use core::time::Duration;
use std::num::NonZeroU32;
use std::time::Instant;
use anyhow::Result;
use esp_idf_svc::hal::{
    delay::FreeRtos,
    gpio::{AnyOutputPin, PinDriver},
    i2c::{I2cConfig, I2cDriver},
    peripherals::Peripherals,
    //sys::EspError,
    units::FromValueType,
};
use esp_idf_svc::hal::spi::{SpiDeviceDriver, SpiDriverConfig};
use esp_idf_svc::hal::spi::config::Config;
use esp_idf_svc::hal::task::notification::Notification;
use esp_idf_svc::timer::EspTaskTimerService;
use imu_fusion::FusionVector;
use mpu9250::{ImuMeasurements, Mpu9250};
use crate::imu_tracker::ImuTracker;


fn main() -> Result<()> {
    // It is necessary to call this function once. Otherwise, some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("Hello, world!");

    let mut delay = FreeRtos;
    let peripherals = Peripherals::take().unwrap();

    let i2c_config = I2cConfig::new().baudrate(400.kHz().into());
    let sda = peripherals.pins.gpio1;
    let scl = peripherals.pins.gpio0;
    let _i2c = I2cDriver::new(peripherals.i2c0, sda, scl, &i2c_config)?;

    let sclk = peripherals.pins.gpio6;
    let miso = peripherals.pins.gpio2;
    let mosi = peripherals.pins.gpio7;
    let cs = PinDriver::output(peripherals.pins.gpio10)?;
    let spi_config = Config::default().baudrate(1.MHz().into());
    let spi = SpiDeviceDriver::new_single(
        peripherals.spi2,
        sclk,
        mosi,
        Some(miso),
        Option::<AnyOutputPin>::None,
        &SpiDriverConfig::new(),
        &spi_config
    )?;
    let mut imu = Mpu9250::imu_default(spi, cs, &mut delay).expect("Could not init IMU!");
    let who_am_i = imu.who_am_i().expect("could not read WHO_AM_I");
    println!("WHO_AM_I: 0x{:x}", who_am_i);

    /*
    let acc_offsets: [f32; 3] = if let Ok(offsets) = imu.calibrate_at_rest(&mut delay) {
        offsets
    } else {
        log::warn!("Could not calibrate offsets!");
        [0f32; 3]
    };
    println!("Calibration is {:?}", acc_offsets);
    */

    let mut flag_total = PinDriver::output(peripherals.pins.gpio21)?;
    let mut flag_compute = PinDriver::output(peripherals.pins.gpio20)?;
    flag_total.set_low()?;
    flag_compute.set_low()?;

    // Configures the notification
    let notification = Notification::new();
    let notifier = notification.notifier();
    
    const IMU_SAMPLE_PERIOD: Duration = Duration::from_millis(5);
    let mut tracker = ImuTracker::new(IMU_SAMPLE_PERIOD, Instant::now());

    let timer_service = EspTaskTimerService::new().unwrap();
    let callback_timer = timer_service.timer(move || unsafe {
        notifier.notify_and_yield(NonZeroU32::new(1).unwrap());
    })?;
    callback_timer.every(IMU_SAMPLE_PERIOD)?;

    const REPORT_PERIOD: Duration = Duration::from_millis(200);
    const SKIP_COUNT: u32 = REPORT_PERIOD.as_millis() as u32 / (IMU_SAMPLE_PERIOD.as_millis() as u32);
    let mut skip = 0;
    
    loop {
        notification.wait(esp_idf_svc::hal::delay::BLOCK);
        
        flag_total.set_high().unwrap();
        flag_compute.set_high().unwrap();        
        let all: ImuMeasurements<[f32;3]> = imu.all().expect("Unable to read from IMU!");
        let imu_gyro = FusionVector::new(all.gyro[0], all.gyro[1],all.gyro[2]) * (180f32 / std::f32::consts::PI);
        let imu_accel = FusionVector::new(all.accel[0],all.accel[1],all.accel[2]);
        tracker.update(imu_accel, imu_gyro);
        flag_compute.set_low().unwrap();

        let deviation = 100f32 * (tracker.latest_sampling_deviation() - 1f32);
        skip += 1;
        if skip == SKIP_COUNT {
            skip = 0;
            //println!("... px:{:+.3}, py:{:+.3}, pz:{:+.3}",
            //         pos_vector.x, pos_vector.y, pos_vector.z);
            println!("dt: {:5.2}ms ({:.1}%), temp: {:.1}C, pitch:{:+.1} roll:{:+.1} yaw:{:+.1} ae_x:{:+.3} ae.y:{:+.3} ae.z:{:+.3}",
                     1000f32*tracker.latest_delta, deviation,
                     all.temp,
                     tracker.euler.angle.pitch, tracker.euler.angle.roll, tracker.euler.angle.yaw,
                     tracker.accel.x, tracker.accel.y, tracker.accel.z,
            );
        }
        flag_total.set_low().unwrap();
    }
}
