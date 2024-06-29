use core::time::Duration;
use std::time::Instant;
use imu_fusion::{Fusion, FusionAhrsSettings, FusionConvention, FusionEuler, FusionMatrix, FusionQuaternion, FusionVector};
use mpu9250::GyroScale;

pub struct ImuTracker {
    time: Instant,
    pub fusion: Fusion,
    pub euler: FusionEuler,
    pub latest_delta: f32,
    pub earth_accel: FusionVector,
    pub linear_accel: FusionVector,
    pub velocity: FusionVector,
    pub position: FusionVector,
}

impl ImuTracker {
    pub fn new(sampling_period: Duration, now: Instant, gyr_range: f32,
               acc_misalignment: FusionMatrix, acc_offset: FusionVector,
               acc_sensitivity: FusionVector, gyr_offset: FusionVector) -> Self {
        // Set the gyroscope range in degrees/s

        let sampling_freq: f32 = 1000_f32 / sampling_period.subsec_millis() as f32;

        // Set AHRS algorithm settings
        let mut ahrs_settings = FusionAhrsSettings::new();
        ahrs_settings.convention = FusionConvention::NWU;

        /* A gain of 0.5 resulted in a strange, long transient that made the
         * computed euler angles drift toward zero after a rotation was over.
         */
        ahrs_settings.gain = 0f32;

        ahrs_settings.acc_rejection = 10.0f32;
        ahrs_settings.recovery_trigger_period = 5;// * sampling_freq as i32;
        ahrs_settings.gyr_range = gyr_range;

        let mut fusion = Fusion::new(sampling_freq as u32, ahrs_settings);
        fusion.acc_misalignment = acc_misalignment;
        fusion.acc_sensitivity = acc_sensitivity;
        fusion.acc_offset = acc_offset;
        fusion.gyr_offset = gyr_offset;

        Self {
            time: now,
            fusion,
            euler: FusionEuler::zero(),
            latest_delta: 0f32,
            earth_accel: FusionVector::zero(),
            linear_accel: FusionVector::zero(),
            velocity: FusionVector::zero(),
            position: FusionVector::zero(),
        }
    }

    pub fn update(&mut self, time: Instant, imu_accel: FusionVector, imu_gyro: FusionVector) {
        // Gets: acceleration in units of standard gravity
        //       angular rotation in degrees/sec
        let delta = time.duration_since(self.time).as_secs_f32();
        self.time = time;
        self.latest_delta = delta;
        self.fusion.update_no_mag_by_duration_seconds(imu_gyro, imu_accel, delta);

        self.compute(imu_accel, delta);
    }

    pub fn compute(&mut self, imu_accel: FusionVector, delta_t: f32) {
        // Gets heading in units of degrees
        self.euler = self.fusion.euler();
    }

}
    }
}