use alloc::string::String;
use alloc::vec::Vec;
use nalgebra::Vector2;
#[cfg(feature="serde_support")]
use serde::{Deserialize, Serialize};

#[derive(Debug, Copy, Clone, PartialEq)]
#[cfg_attr(feature = "serde_support", derive(Serialize, Deserialize))]
pub struct Point {
    pub x: f64,
    pub y: f64,
}

impl From<Point> for Vector2<f64> {
    fn from(value: Point) -> Self {
        Vector2::new(value.x, value.y)
    }
}

#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde_support", derive(Serialize, Deserialize))]
pub struct Constraints {
    pub velocity: f64,
    pub accel: f64,
}

#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde_support", derive(Serialize, Deserialize))]
pub struct PathSegment {
    pub inverted: bool,
    pub stop_end: bool,
    pub path: Vec<Point>,
    pub constraints: Constraints,
}

#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde_support", derive(Serialize, Deserialize))]
pub struct Command {
    pub t: f64,
    pub name: String,
}

#[derive(Debug, PartialEq)]
#[cfg_attr(feature = "serde_support", derive(Serialize, Deserialize))]
pub struct Path {
    pub start_speed: f64,
    pub end_speed: f64,
    pub segments: Vec<PathSegment>,
    pub commands: Vec<Command>,
}

#[cfg(test)]
mod tests {
    use alloc::vec;
    use super::*;
    #[cfg(feature = "serde_support")]
    use serde_json;

    #[test]
    fn test_point_to_vector2_conversion() {
        let point = Point { x: 3.0, y: 4.0 };
        let vector: Vector2<f64> = point.into();

        assert_eq!(vector.x, 3.0);
        assert_eq!(vector.y, 4.0);
    }

    #[test]
    fn test_constraints_initialization() {
        let constraints = Constraints {
            velocity: 5.0,
            accel: 1.5,
        };

        assert_eq!(constraints.velocity, 5.0);
        assert_eq!(constraints.accel, 1.5);
    }

    #[test]
    fn test_path_segment_initialization() {
        let point1 = Point { x: 0.0, y: 0.0 };
        let point2 = Point { x: 1.0, y: 1.0 };
        let points = vec![point1, point2];

        let constraints = Constraints {
            velocity: 2.5,
            accel: 1.2,
        };

        let path_segment = PathSegment {
            inverted: false,
            stop_end: true,
            path: points.clone(),
            constraints: constraints.clone(),
        };

        assert_eq!(path_segment.inverted, false);
        assert_eq!(path_segment.stop_end, true);
        assert_eq!(path_segment.path.len(), 2);
        assert_eq!(path_segment.constraints.velocity, 2.5);
        assert_eq!(path_segment.constraints.accel, 1.2);
    }

    #[test]
    fn test_command_initialization() {
        let command = Command {
            t: 10.0,
            name: String::from("TestCommand"),
        };

        assert_eq!(command.t, 10.0);
        assert_eq!(command.name, "TestCommand");
    }

    #[test]
    fn test_path_initialization() {
        let point1 = Point { x: 0.0, y: 0.0 };
        let point2 = Point { x: 2.0, y: 2.0 };
        let points = vec![point1, point2];

        let constraints = Constraints {
            velocity: 3.0,
            accel: 2.0,
        };

        let segment = PathSegment {
            inverted: false,
            stop_end: false,
            path: points.clone(),
            constraints: constraints.clone(),
        };

        let command = Command {
            t: 5.0,
            name: String::from("Start"),
        };

        let path = Path {
            start_speed: 1.0,
            end_speed: 0.0,
            segments: vec![segment.clone()],
            commands: vec![command.clone()],
        };

        assert_eq!(path.start_speed, 1.0);
        assert_eq!(path.end_speed, 0.0);
        assert_eq!(path.segments.len(), 1);
        assert_eq!(path.commands.len(), 1);
    }

    #[cfg(feature = "serde_support")]
    #[test]
    fn test_serialize_deserialize_point() {
        let point = Point { x: 1.0, y: 2.0 };
        let serialized = serde_json::to_string(&point).unwrap();
        let deserialized: Point = serde_json::from_str(&serialized).unwrap();

        assert_eq!(point, deserialized);
    }

    #[cfg(feature = "serde_support")]
    #[test]
    fn test_serialize_deserialize_constraints() {
        let constraints = Constraints {
            velocity: 5.0,
            accel: 2.5,
        };
        let serialized = serde_json::to_string(&constraints).unwrap();
        let deserialized: Constraints = serde_json::from_str(&serialized).unwrap();

        assert_eq!(constraints, deserialized);
    }

    #[cfg(feature = "serde_support")]
    #[test]
    fn test_serialize_deserialize_path() {
        let point1 = Point { x: 0.0, y: 0.0 };
        let point2 = Point { x: 2.0, y: 2.0 };
        let points = vec![point1, point2];

        let constraints = Constraints {
            velocity: 3.0,
            accel: 2.0,
        };

        let segment = PathSegment {
            inverted: false,
            stop_end: false,
            path: points.clone(),
            constraints: constraints.clone(),
        };

        let command = Command {
            t: 5.0,
            name: String::from("Start"),
        };

        let path = Path {
            start_speed: 1.0,
            end_speed: 0.0,
            segments: vec![segment.clone()],
            commands: vec![command.clone()],
        };

        let serialized = serde_json::to_string(&path).unwrap();
        let deserialized: Path = serde_json::from_str(&serialized).unwrap();

        assert_eq!(path.start_speed, deserialized.start_speed);
        assert_eq!(path.end_speed, deserialized.end_speed);
        assert_eq!(path.segments.len(), deserialized.segments.len());
        assert_eq!(path.commands.len(), deserialized.commands.len());
    }
}