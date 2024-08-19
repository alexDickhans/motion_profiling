use core::time::Duration;

use uom::si::f64::{AngularVelocity, Velocity};

use crate::localization::localization::StateRepresentation;

pub struct MotionCommand {
    pub desired_velocity: Velocity,
    pub desired_angular: AngularVelocity,
    pub desired_pose: StateRepresentation,
}

pub trait MotionProfile {
    fn get_duration(&self) -> Duration;
    fn get(&mut self, t: Duration) -> MotionCommand;
}

pub fn add(left: u64, right: u64) -> u64 {
    left + right
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        let result = add(2, 2);
        assert_eq!(result, 4);
    }
}
