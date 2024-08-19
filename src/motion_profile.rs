use core::time::Duration;
use nalgebra::Vector3;
use uom::si::f64::{AngularVelocity, Velocity};

pub struct MotionCommand {
    desired_velocity: Velocity,
    desired_angular: AngularVelocity,
    desired_pose: Vector3<f64>,
}

pub trait MotionProfile {
    fn get_duration(&self) -> Duration;
    fn get(&mut self, t: Duration) -> Option<MotionCommand>;
}