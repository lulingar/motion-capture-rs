use imu_fusion::FusionVector;
use std::{collections::VecDeque, f32::consts::PI};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MovementDirection {
    Horizontal,
    Vertical,
    Diagonal,
}

pub struct Analysis {
    smoothing: Smoothing,
    movement_detection: MovementDetection,
}

struct Smoothing {
    measurements: VecDeque<FusionVector>,
    smoothing_window_size: usize,
}

impl Smoothing {
    // Adds a measurement and returns smoothed value
    fn add_measurement(&mut self, linear_acceleration: FusionVector) -> FusionVector {
        if self.measurements.len() >= self.smoothing_window_size {
            self.measurements.pop_front();
        }
        let sv = self.compute_smoothing_vector();
        self.measurements.push_back(linear_acceleration);
        return linear_acceleration - sv;
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

struct MovementDetection {
    horizontal_measurements: VecDeque<f32>,
    vertical_measurements: VecDeque<f32>,
    detection_window_size: usize,
    acceleration_threshold: f32,
    angle_low_threshold: f32,
    angle_high_threshold: f32,
    prev_direction: Option<MovementDirection>,
}

impl MovementDetection {
    fn add_measurement(&mut self, x: f32, y: f32) -> Option<MovementDirection> {
        if self.horizontal_measurements.len() >= self.detection_window_size {
            self.horizontal_measurements.pop_front();
            self.vertical_measurements.pop_front();
        }

        self.horizontal_measurements.push_back(x);
        self.vertical_measurements.push_back(y);

        self.compute_direction()
    }

    fn compute_direction(&mut self) -> Option<MovementDirection> {
        if self.horizontal_measurements.len() < self.detection_window_size {
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
        let mean_horizontal = self.horizontal_measurements.iter().sum::<f32>() / self.horizontal_measurements.len() as f32;
        let mean_vertical = self.horizontal_measurements.iter().sum::<f32>() / self.horizontal_measurements.len() as f32;

        return (mean_horizontal, mean_vertical);
    }

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
            smoothing: Smoothing {
                measurements: VecDeque::with_capacity(smoothing_window_size),
                smoothing_window_size,
            },
            movement_detection: MovementDetection {
                horizontal_measurements: VecDeque::with_capacity(detection_window_size),
                vertical_measurements: VecDeque::with_capacity(detection_window_size),
                detection_window_size,
                acceleration_threshold,
                angle_low_threshold,
                angle_high_threshold,
                prev_direction: None,
            },
        }
    }

    pub fn add_measurement(&mut self, linear_acceleration: FusionVector) -> Option<MovementDirection> {
        let smoothed = self.smoothing.add_measurement(linear_acceleration);

        let x = (smoothed.x * smoothed.x + smoothed.y * smoothed.y).sqrt(); // compute euclidean norm of x and y component
        let y = smoothed.z.abs();

        return self.movement_detection.add_measurement(x, y);
    }

}
