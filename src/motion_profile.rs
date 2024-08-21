use core::time::Duration;
use nalgebra::Vector3;
use uom::si::f64::{AngularVelocity, Velocity};

#[derive(Debug, PartialOrd, PartialEq)]
pub struct MotionCommand {
    pub desired_velocity: Velocity,
    pub desired_angular: AngularVelocity,
    pub desired_pose: Vector3<f64>,
}

pub trait MotionProfile {
    fn duration(&self) -> Duration;
    fn get(&mut self, t: Duration) -> Option<MotionCommand>;
}
