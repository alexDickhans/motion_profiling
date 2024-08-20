use alloc::vec;
use alloc::vec::Vec;
use core::cmp::{max};
use core::time::Duration;
use interp::interp;
use uom::si::angular_velocity::radian_per_second;
use uom::si::f64::{AngularVelocity, Velocity};
use uom::si::velocity::meter_per_second;
use crate::bezier::Bezier;
use crate::combined_mp::CombinedMP;
use crate::motion_profile::{MotionCommand, MotionProfile};
use crate::path::{Path};


pub struct MotionProfile2d {
    path: Vec<Bezier<50>>,
    /// (time, velocity, t)
    time_to_velocity: (Vec<f64>, Vec<f64>, Vec<f64>),
}

impl MotionProfile2d {
    fn new(path: Path, track_width: f64) -> Self {
        let mut res = Self {
            path: path.segments.into_iter().filter_map(|path_segment| path_segment.try_into().ok()).collect(),
            time_to_velocity: (vec![], vec![], vec![])
        };

        res.compute(path.start_speed, path.end_speed, track_width);
        
        res
    }

    fn limited_speed(curvature: f64, track_width: f64) -> f64 {
        if curvature == 0.0 || !curvature.is_finite() {
            1.0
        } else {
            1.0 / (1.0 + (curvature * 0.5) * track_width).abs()
        }
    }

    fn limit_speed(unlimited_speed: Vec<(f64, f64, f64, f64)>) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
        let t_arr = unlimited_speed.iter().map(|speeds| speeds.3).collect();
        let time = vec![];
        let mut velocity = vec![unlimited_speed[0].1];

        for item in 1..unlimited_speed.len() {
            let delta_distance = (unlimited_speed[item].0 - unlimited_speed[item - 1].0).abs();
            velocity.push(
                (unlimited_speed[item - 1].1.powi(2) + 2.0 * unlimited_speed[item - 1].2 * delta_distance).sqrt().min(unlimited_speed[item].1));
        }

        (time, velocity, t_arr)
    }
    
    fn compute(&mut self, start_speed: f64, end_speed: f64, track_width: f64) {
        // distance, speed, accel, t
        let mut unlimited_speed = vec![(0.0, start_speed.min(Self::limited_speed(self.get_bezier(0.0).unwrap().get_curvature(0.0), track_width)), 0.0, 0.0)];

        let mut distance = 0.0;

        for (index, bezier) in self.path.iter().enumerate() {
            let length = bezier.get_length();
            let count = max(5, (length/0.02) as usize);

            for i in 1..=count {
                let t = i as f64 / count as f64;

                let curvature = bezier.get_curvature(t).min(bezier.vel);

                unlimited_speed.push((distance + bezier.get_distance_by_t(t), Self::limited_speed(curvature, track_width), bezier.accel, index as f64 + t));
            }

            distance += length;
        }

        assert!(!unlimited_speed.is_empty());

        let len = unlimited_speed.len()-1;

        unlimited_speed[len].1 = end_speed.min(unlimited_speed[len].1);

        let left_pass = unlimited_speed.clone();
        let mut right_pass = unlimited_speed.clone();
        right_pass.reverse();

        assert_eq!(left_pass.len(), unlimited_speed.len());
        assert_eq!(right_pass.len(), unlimited_speed.len());

        let left_pass_limited = Self::limit_speed(left_pass);
        let mut right_pass_limited = Self::limit_speed(right_pass);
        right_pass_limited.0.reverse();
        right_pass_limited.1.reverse();
        right_pass_limited.2.reverse();

        assert_eq!(right_pass_limited.2.len(), unlimited_speed.len());
        assert_eq!(right_pass_limited.1.len(), unlimited_speed.len());

        self.time_to_velocity.2 = right_pass_limited.2;

        for (i, unlimited) in unlimited_speed.iter_mut().enumerate() {
            unlimited.1 = left_pass_limited.1[i].min(right_pass_limited.1[i]);
        }

        let mut time = 0.0;

        for i in 1..unlimited_speed.len() {
            // add stuff to current time
            let delta_distance = unlimited_speed[i].0 - unlimited_speed[i-1].0;
            let a =
                (unlimited_speed[i].1.powi(2) - unlimited_speed[i-1].1.powi(2)) / (2.0 * delta_distance);

            if a.abs() > 0.0001 {
                time += (unlimited_speed[i].1 - unlimited_speed[i-1].1) / a;
            } else {
                time += delta_distance / unlimited_speed[i].1;
            }

            self.time_to_velocity.0.push(time);
            self.time_to_velocity.1.push(unlimited_speed[i].1);
        }
    }

    fn get_bezier(&self, t: f64) -> Option<&Bezier<50>> {
        self.path.get(t.floor() as usize)
    }
}

impl MotionProfile for MotionProfile2d {
    fn duration(&self) -> Duration {
        Duration::from_secs_f64(*self.time_to_velocity.0.last().expect("Array should not be empty"))
    }

    fn get(&mut self, time: Duration) -> Option<MotionCommand> {
        if self.duration() < time {
            None
        } else {
            let t = interp(&self.time_to_velocity.0, &self.time_to_velocity.2, time.as_secs_f64());
            let bezier = self.get_bezier(t)?;
            let t_local = t % 1.0;

            let desired_velocity = Velocity::new::<meter_per_second>(interp(&self.time_to_velocity.0, &self.time_to_velocity.1, time.as_secs_f64()));
            let curvature = bezier.get_curvature(t_local);

            Some(
                MotionCommand {
                    desired_velocity,
                    desired_angular: AngularVelocity::new::<radian_per_second>(desired_velocity.get::<meter_per_second>() * curvature),
                    desired_pose: bezier.get(t_local),
                }
            )
        }
    }
}

impl CombinedMP<MotionProfile2d> {
    pub fn try_new_2d(mut path: Path, track_width: f64) -> Option<Self> {
        let mut paths = Vec::new();

        let mut segment_iter = path.segments.into_iter();

        let mut segments = vec![segment_iter.next()?];

        let last_inverted = segments[0].inverted;

        for segment in segment_iter {
            if (segment.inverted ^ last_inverted) || segment.stop_end {
                if !segments.is_empty() {
                    paths.push(Path {
                        start_speed: path.start_speed,
                        end_speed: 0.0,
                        segments: segments.drain(..).collect(),
                        commands: vec![],
                    });
                    path.start_speed = 0.0;
                }
            }
            segments.push(segment);
        }

        paths.push(Path {
            start_speed: path.start_speed,
            end_speed: path.end_speed,
            segments: segments.drain(..).collect(),
            commands: vec![],
        });

        Some(Self::new(paths.drain(..).map(|path: Path| MotionProfile2d::new(path, track_width)).collect()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::time::Duration;
    use crate::path::{Constraints, PathSegment, Point};

    fn create_mock_path() -> Path {
        // Create a mock path with dummy segments
        Path {
            start_speed: 1.0,
            end_speed: 0.5,
            segments: vec![PathSegment{ inverted: false, stop_end: false, path: vec![Point{ x: 0.0, y: 0.0 }, Point{ x: 1.0, y: 0.0 }, Point{ x: 0.0, y: 1.0 }, Point{ x: 1.0, y: 1.0 }], constraints: Constraints { velocity: 1.0, accel: 1.0 } }], // You may need to customize this for your actual tests
            commands: vec![],
        }
    }

    #[test]
    fn test_new_motion_profile_2d() {
        let path = create_mock_path();
        let track_width = 1.0;
        let profile = MotionProfile2d::new(path, track_width);

        assert!(!profile.path.is_empty(), "Path should not be empty after initialization");
        assert!(!profile.time_to_velocity.0.is_empty(), "Initial time_to_velocity vector should not be empty");
    }

    #[test]
    fn test_limited_speed_no_curvature() {
        let track_width = 1.0;
        let curvature = 0.0;
        let speed = MotionProfile2d::limited_speed(curvature, track_width);

        assert_eq!(speed, 1.0, "Speed should be 1.0 when curvature is zero");
    }

    #[test]
    fn test_limited_speed_with_curvature() {
        let track_width = 1.0;
        let curvature = 0.5;
        let speed = MotionProfile2d::limited_speed(curvature, track_width);

        assert!(speed < 1.0, "Speed should be less than 1.0 when curvature is positive");
    }

    #[test]
    fn test_compute_with_simple_path() {
        let path = create_mock_path();
        let track_width = 1.0;
        let profile = MotionProfile2d::new(path, track_width);

        // Test that the computation is correctly run and values are calculated
        assert!(!profile.time_to_velocity.0.is_empty(), "Computed time array should not be empty");
        assert!(!profile.time_to_velocity.1.is_empty(), "Computed velocity array should not be empty");
        assert!(!profile.time_to_velocity.2.is_empty(), "Computed t array should not be empty");
    }

    #[test]
    fn test_get_duration() {
        let path = create_mock_path();
        let track_width = 1.0;
        let profile = MotionProfile2d::new(path, track_width);

        let duration = profile.duration();

        assert!(duration.as_secs_f64() > 0.0, "Duration should be greater than zero after computation");
    }

    #[test]
    fn test_get_motion_command_at_time() {
        let path = create_mock_path();
        let track_width = 1.0;
        let mut profile = MotionProfile2d::new(path, track_width);

        let time = Duration::from_secs_f64(0.5);
        let command = profile.get(time);

        assert!(command.is_some(), "MotionCommand should be returned for valid time input");
    }

    #[test]
    fn test_get_motion_command_out_of_bounds() {
        let path = create_mock_path();
        let track_width = 1.0;
        let mut profile = MotionProfile2d::new(path, track_width);

        let time = Duration::from_secs_f64(10.0); // Assuming time exceeds duration
        let command = profile.get(time);

        assert!(command.is_none(), "MotionCommand should return None for time exceeding the duration");
    }

    #[test]
    fn test_try_new_2d_combined_motion_profile() {
        let path = create_mock_path();
        let track_width = 1.0;
        let combined_profile = CombinedMP::try_new_2d(path, track_width);

        assert!(combined_profile.is_some(), "Combined motion profile should be created successfully");
    }
}
