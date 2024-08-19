use alloc::string::String;
use alloc::vec::Vec;
use nalgebra::Vector2;
use serde::{Deserialize, Serialize};

#[derive(Debug)]
#[cfg_attr(feature = "serde_support", derive(Serialize, Deserialize))]
pub struct Point {
    x: f64,
    y: f64,
}

impl Into<Vector2<f64>> for Point {
    fn into(self) -> Vector2<f64> {
        Vector2::new(self.x, self.y)
    }
}

#[derive(Debug)]
#[cfg_attr(feature = "serde_support", derive(Serialize, Deserialize))]
pub struct PathSegment {
    inverted: bool,
    stop_end: bool,
    path: Vec<Point>,
}

#[derive(Debug)]
#[cfg_attr(feature = "serde_support", derive(Serialize, Deserialize))]
pub struct Command {
    t: f64,
    name: String,
}

#[derive(Debug)]
#[cfg_attr(feature = "serde_support", derive(Serialize, Deserialize))]
pub struct Path {
    start_speed: f64,
    end_speed: f64,
    segments: Vec<PathSegment>,
    commands: Vec<Command>,
}
