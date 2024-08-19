use alloc::vec;
use core::time::Duration;
use crate::combined_mp::CombinedMP;
use crate::motion_profile::{MotionCommand, MotionProfile};
use crate::path::Path;

pub struct MotionProfile2d {
    path: Path,
}

impl MotionProfile2d {
    fn new(path: Path) -> Self {
        Self {
            path
        }
    }
}

impl MotionProfile for MotionProfile2d {
    fn get_duration(&self) -> Duration {
        Duration::from_millis((self.path.segments.last().unwrap().path.iter().last().unwrap().y * 1000.0) as _)
    }

    fn get(&mut self, _t: Duration) -> Option<MotionCommand> {
        todo!()
    }
}

impl CombinedMP<MotionProfile2d> {
    pub fn new_2d(path: Path) -> Self {
        Self::new(vec![MotionProfile2d::new(path)])
    }
}