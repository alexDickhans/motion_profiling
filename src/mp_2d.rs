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
        todo!()
    }

    fn get(&mut self, _t: Duration) -> Option<MotionCommand> {
        todo!()
    }
}

impl CombinedMP<MotionProfile2d> {
    pub fn new_2d(path: Path) -> Self {
        todo!()
    }
}