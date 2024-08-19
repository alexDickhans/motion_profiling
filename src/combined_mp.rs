use alloc::vec::Vec;
use core::time::Duration;
use crate::motion_profile::{MotionCommand, MotionProfile};

pub struct CombinedMP<T: MotionProfile> {
    motion_profiles: Vec<T>
}

impl<T: MotionProfile> CombinedMP<T> {
    pub fn new(motion_profiles: Vec<T>) -> Self {
        Self {
            motion_profiles
        }
    }
}

impl<T: MotionProfile> MotionProfile for CombinedMP<T> {
    fn get_duration(&self) -> Duration {
        self.motion_profiles.iter().map(|profile| profile.get_duration()).sum()
    }

    fn get(&mut self, t: Duration) -> Option<MotionCommand> {
        if t < Duration::new(0, 0) || t > self.get_duration() {
            None
        } else {
            let mut accumulated_t = Duration::new(0, 0);

            self.motion_profiles.iter_mut().find(|profile| {
                let duration = profile.get_duration();
                if accumulated_t + duration >= t {
                    true
                } else {
                    accumulated_t += duration;
                    false
                }
            })?.get(t - accumulated_t)
        }
    }
}
