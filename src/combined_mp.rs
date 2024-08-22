use crate::motion_profile::{MotionCommand, MotionProfile};
use alloc::vec::Vec;
use core::time::Duration;

pub struct CombinedMP<T: MotionProfile> {
    motion_profiles: Vec<T>,
}

impl<T: MotionProfile> CombinedMP<T> {
    pub fn new(motion_profiles: Vec<T>) -> Self {
        Self { motion_profiles }
    }

    pub fn len(&self) -> usize {
        self.motion_profiles.iter().len()
    }
}

impl<T: MotionProfile> MotionProfile for CombinedMP<T> {
    fn duration(&self) -> Duration {
        self.motion_profiles
            .iter()
            .map(|profile| profile.duration())
            .sum()
    }

    fn get(&mut self, t: Duration) -> Option<MotionCommand> {
        if t < Duration::new(0, 0) || t > self.duration() {
            None
        } else {
            let mut accumulated_t = Duration::new(0, 0);

            self.motion_profiles
                .iter_mut()
                .find(|profile| {
                    let duration = profile.duration();
                    if accumulated_t + duration >= t {
                        true
                    } else {
                        accumulated_t += duration;
                        false
                    }
                })?
                .get(t - accumulated_t)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec;
    use core::time::Duration;
    use nalgebra::Vector3;
    use uom::si::angular_velocity::radian_per_second;
    use uom::si::f64::{AngularVelocity, Velocity};
    use uom::si::velocity::meter_per_second;

    // Mock implementation of MotionCommand for testing
    #[derive(Debug, PartialEq)]
    struct MockMotionCommand {
        pub value: i32,
    }

    // Mock implementation of the MotionProfile trait for testing
    struct MockMotionProfile {
        duration: Duration,
        command: MockMotionCommand,
    }

    impl MotionProfile for MockMotionProfile {
        fn duration(&self) -> Duration {
            self.duration
        }

        fn get(&mut self, t: Duration) -> Option<MotionCommand> {
            if t <= self.duration {
                Some(MotionCommand {
                    desired_velocity: Velocity::new::<meter_per_second>(t.as_secs_f64()),
                    desired_angular: AngularVelocity::new::<radian_per_second>(0.0),
                    desired_pose: Vector3::new(self.command.value as f64, 0.0, 0.0),
                })
            } else {
                None
            }
        }
    }

    #[test]
    fn test_combined_motion_profile_duration() {
        let profiles = vec![
            MockMotionProfile {
                duration: Duration::new(2, 0),
                command: MockMotionCommand { value: 10 },
            },
            MockMotionProfile {
                duration: Duration::new(3, 0),
                command: MockMotionCommand { value: 20 },
            },
        ];

        let combined_mp = CombinedMP::new(profiles);

        assert_eq!(combined_mp.duration(), Duration::new(5, 0));
    }

    #[test]
    fn test_combined_motion_profile_get() {
        let profiles = vec![
            MockMotionProfile {
                duration: Duration::new(2, 0),
                command: MockMotionCommand { value: 10 },
            },
            MockMotionProfile {
                duration: Duration::new(3, 0),
                command: MockMotionCommand { value: 20 },
            },
        ];

        let mut combined_mp = CombinedMP::new(profiles);

        // Test within the first profile
        assert_eq!(
            combined_mp.get(Duration::new(1, 0)),
            Some(MotionCommand {
                desired_velocity: Velocity::new::<meter_per_second>(1.0),
                desired_angular: AngularVelocity::new::<radian_per_second>(0.0),
                desired_pose: Vector3::new(10.0, 0.0, 0.0),
            })
        );

        // Test at the boundary between profiles
        assert_eq!(
            combined_mp.get(Duration::new(2, 0)),
            Some(MotionCommand {
                desired_velocity: Velocity::new::<meter_per_second>(2.0),
                desired_angular: AngularVelocity::new::<radian_per_second>(0.0),
                desired_pose: Vector3::new(10.0, 0.0, 0.0),
            })
        );

        // Test within the second profile
        assert_eq!(
            combined_mp.get(Duration::new(4, 0)),
            Some(MotionCommand {
                desired_velocity: Velocity::new::<meter_per_second>(2.0),
                desired_angular: AngularVelocity::new::<radian_per_second>(0.0),
                desired_pose: Vector3::new(20.0, 0.0, 0.0),
            })
        );

        // Test outside the duration
        assert_eq!(combined_mp.get(Duration::new(6, 0)), None);
    }

    #[test]
    fn test_combined_motion_profile_invalid_times() {
        let profiles = vec![
            MockMotionProfile {
                duration: Duration::new(2, 0),
                command: MockMotionCommand { value: 10 },
            },
            MockMotionProfile {
                duration: Duration::new(3, 0),
                command: MockMotionCommand { value: 20 },
            },
        ];

        let mut combined_mp = CombinedMP::new(profiles);

        // Test a negative time
        assert_eq!(
            combined_mp.get(Duration::new(0, 0)),
            Some(MotionCommand {
                desired_velocity: Velocity::new::<meter_per_second>(0.0),
                desired_angular: AngularVelocity::new::<radian_per_second>(0.0),
                desired_pose: Vector3::new(10.0, 0.0, 0.0),
            })
        );

        // Test a time greater than the total duration
        assert_eq!(combined_mp.get(Duration::new(10, 0)), None);
    }
}
