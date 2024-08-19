use alloc::string::String;
use alloc::vec::Vec;
use nalgebra::Vector2;
#[cfg(feature="serde_support")]
use serde::{Deserialize, Serialize};

#[derive(Debug)]
#[cfg_attr(feature = "serde_support", derive(Serialize, Deserialize))]
pub struct Point {
    pub x: f64,
    pub y: f64,
}

impl Into<Vector2<f64>> for Point {
    fn into(self) -> Vector2<f64> {
        Vector2::new(self.x, self.y)
    }
}

#[derive(Debug)]
#[cfg_attr(feature = "serde_support", derive(Serialize, Deserialize))]
pub struct PathSegment {
    pub inverted: bool,
    pub stop_end: bool,
    pub path: Vec<Point>,
}

#[derive(Debug)]
#[cfg_attr(feature = "serde_support", derive(Serialize, Deserialize))]
pub struct Command {
    pub t: f64,
    pub name: String,
}

#[derive(Debug)]
#[cfg_attr(feature = "serde_support", derive(Serialize, Deserialize))]
pub struct Path {
    pub start_speed: f64,
    pub end_speed: f64,
    pub segments: Vec<PathSegment>,
    pub commands: Vec<Command>,
}
