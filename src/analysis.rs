use imu_fusion::FusionVector;
use std::{collections::VecDeque, f32::consts::PI};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MovementDirection {
    Horizontal,
    Vertical,
    Diagonal,
}

pub struct Analysis {
    measurements: VecDeque<FusionVector>,
    smoothing_window_size: usize,
    detection_window_size: usize,
    acceleration_threshold: f32,
    angle_low_threshold: f32,
    angle_high_threshold: f32,
    prev_direction: Option<MovementDirection>,
}

impl Default for Analysis {
    fn default() -> Self {
        let diagonal_low = PI / 4.0 - PI / 8.0;
        let diagonal_high = 1.1 * PI / 4.0;
        return Analysis::new(100, 30, 1.5, diagonal_low, diagonal_high);
    }
}

impl Analysis {
    pub fn new(
        smoothing_window_size: usize,
        detection_window_size: usize,
        acceleration_threshold: f32,
        angle_low_threshold: f32,
        angle_high_threshold: f32,
    ) -> Analysis {
        assert!(smoothing_window_size > 0);
        assert!(detection_window_size > 0);
        assert!(detection_window_size > smoothing_window_size);

        Analysis {
            measurements: VecDeque::with_capacity(smoothing_window_size),
            smoothing_window_size,
            detection_window_size,
            acceleration_threshold,
            angle_low_threshold,
            angle_high_threshold,
            prev_direction: None,
        }
    }

    pub fn add_measurement(&mut self, earth_acc: FusionVector) -> Option<MovementDirection>  {
        if self.measurements.len() >= self.smoothing_window_size {
            self.measurements.pop_front();
        }

        self.measurements.push_back(earth_acc);
        return self.compute_direction();
    }

    fn compute_direction(&mut self) -> Option<MovementDirection> {
        if self.measurements.len() < self.smoothing_window_size {
            return None;
        }

        let (x, y) = self.compute_average_detection_accel();

        return self.next_direction(x, y);
    }

    fn next_direction(&mut self, x_accel: f32, y_accel: f32) -> Option<MovementDirection> {
        if self.prev_direction.is_some() && self.below_acceleration_threshold(x_accel, y_accel) {
            self.prev_direction = None;
            return None;
        } else if self.prev_direction.is_some() {
            return self.prev_direction;
        }

        let angle = f32::atan2(x_accel, y_accel);

        if !self.below_acceleration_threshold(x_accel, y_accel) {
            self.prev_direction =
                if self.angle_low_threshold < angle && angle < self.angle_high_threshold {
                    Some(MovementDirection::Diagonal)
                } else if angle < self.angle_low_threshold {
                    Some(MovementDirection::Horizontal)
                } else {
                    Some(MovementDirection::Vertical)
                };
        }
        return self.prev_direction;
    }

    fn below_acceleration_threshold(&self, x_accel: f32, y_accel: f32) -> bool {
        return x_accel < self.acceleration_threshold && y_accel < self.acceleration_threshold;
    }

    fn compute_average_detection_accel(&self) -> (f32, f32) {
        let smoothing_vector = self.compute_smoothing_vector();

        let mut sum_horizontal = 0.0;
        let mut sum_vertical = 0.0;

        for m in self
            .measurements
            .iter()
            .skip(self.measurements.len() - self.smoothing_window_size)
        {
            let m = *m - smoothing_vector;
            let horizontal = (m.x * m.x + m.y * m.y).sqrt(); // compute euclidean norm of x and y component
            let vertical = m.z.abs();

            sum_horizontal += horizontal;
            sum_vertical += vertical;
        }
        let mean_horizontal = sum_horizontal / self.smoothing_window_size as f32;
        let mean_vertical = sum_vertical / self.smoothing_window_size as f32;
        return (mean_horizontal, mean_vertical);
    }

    fn compute_smoothing_vector(&self) -> FusionVector {
        let mut sum_x = 0.0;
        let mut sum_y = 0.0;
        let mut sum_z = 0.0;
        for m in &self.measurements {
            sum_x += m.x;
            sum_y += m.y;
            sum_z += m.z;
        }
        let num_elems = self.measurements.len() as f32;
        return FusionVector::new(sum_x / num_elems, sum_y / num_elems, sum_z / num_elems);
    }
}
