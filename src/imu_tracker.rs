use core::time::Duration;
use imu_fusion::{Fusion, FusionAhrsSettings, FusionEuler, FusionVector};
use std::time::Instant;

pub struct ImuTracker {
    pub sampling_period: Duration,
    time: Instant,
    pub euler: FusionEuler,
    pub accel: FusionVector,
    pub velocity: FusionVector,
    pub position: FusionVector,
    fusion: Fusion,
    pub latest_delta: f32,
}

impl ImuTracker {
    pub fn new(sampling_period: Duration, now: Instant) -> Self {
        let sampling_freq: f32 = 1000_f32 / sampling_period.subsec_millis() as f32;
        let ahrs_settings = FusionAhrsSettings::new();
        let fusion = Fusion::new(sampling_freq as u32, ahrs_settings);

        Self {
            sampling_period,
            time: now,
            euler: FusionEuler::zero(),
            accel: FusionVector::zero(),
            velocity: FusionVector::zero(),
            position: FusionVector::zero(),
            fusion,
            latest_delta: 0f32,
        }
    }

    pub fn update(&mut self, time: Instant, imu_accel: FusionVector, imu_gyro: FusionVector) {
        // Gets: acceleration in units of standard gravity
        //       angular rotation in degrees/sec
        let delta = time.duration_since(self.time).as_secs_f32();
        self.time = time;
        self.latest_delta = delta;

        //fusion.update_no_mag(gyro_vector, acceleration_vector, timestamp);
        self.fusion
            .update_no_mag_by_duration_seconds(imu_gyro, imu_accel, delta);

        // Gets heading in units of degrees
        self.euler = self.fusion.euler();
        self.accel = self.fusion.earth_acc();
    }

    pub fn latest_sampling_deviation(&self) -> f32 {
        self.latest_delta / self.sampling_period.as_secs_f32()
    }
}