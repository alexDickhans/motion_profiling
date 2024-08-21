use crate::bezier::BezierConstructionError::{TooFewPoints, TooManyPoints};
use crate::path::PathSegment;
use core::cmp::Ordering;
use interp::interp;
use nalgebra::{
    Matrix2x4, Matrix3x4, Matrix4, RowVector2, RowVector3, RowVector4, Vector2, Vector3, Vector4,
};

const BEZIER_MATRIX: Matrix4<f64> = Matrix4::new(
    -1.0, 3.0, -3.0, 1.0, 3.0, -6.0, 3.0, 0.0, -3.0, 3.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
);

const BEZIER_D_MATRIX: Matrix3x4<f64> = Matrix3x4::new(
    -3.0, 9.0, -9.0, 3.0, 6.0, -12.0, 6.0, 0.0, -3.0, 3.0, 0.0, 0.0,
);

const BEZIER_DD_MATRIX: Matrix2x4<f64> =
    Matrix2x4::new(-6.0, 18.0, -18.0, 6.0, 6.0, -12.0, 6.0, 0.0);

#[derive(Debug, Copy, Clone)]
pub enum BezierConstructionError {
    TooFewPoints,
    TooManyPoints,
}

pub struct Bezier<const D: usize> {
    points: (Vector2<f64>, Vector2<f64>, Vector2<f64>, Vector2<f64>),
    dist_to_t: ([f64; D], [f64; D]),
    pub vel: f64,
    pub accel: f64,
}

impl<const D: usize> Bezier<D> {
    pub fn new(
        points: (Vector2<f64>, Vector2<f64>, Vector2<f64>, Vector2<f64>),
        vel: f64,
        accel: f64,
    ) -> Self {
        let mut res = Self {
            points,
            dist_to_t: ([0.0; D], [0.0; D]),
            vel,
            accel,
        };

        res.compute_dist_to_t();

        res
    }

    fn compute_dist_to_t(&mut self) {
        let mut mag_sum = 0.0;

        for i in 0..D {
            let t = i as f64 / (D - 1) as f64;
            mag_sum += self.get_d(t).magnitude() / D as f64;
            self.dist_to_t.0[i] = t;
            self.dist_to_t.1[i] = mag_sum;
        }
    }

    pub fn get_length(&self) -> f64 {
        self.dist_to_t.1.last().unwrap().clone()
    }

    pub fn get(&self, t: f64) -> Vector3<f64> {
        let t_vector = RowVector4::new(t.powi(3), t.powi(2), t, 1.0);
        let x = t_vector
            * BEZIER_MATRIX
            * Vector4::new(
                self.points.0.x,
                self.points.1.x,
                self.points.2.x,
                self.points.3.x,
            );
        let y = t_vector
            * BEZIER_MATRIX
            * Vector4::new(
                self.points.0.y,
                self.points.1.y,
                self.points.2.y,
                self.points.3.y,
            );

        let d = self.get_d(t);

        Vector3::new(x.sum(), y.sum(), d.y.atan2(d.x))
    }

    pub fn get_d(&self, t: f64) -> Vector2<f64> {
        let t_vector = RowVector3::new(t.powi(2), t, 1.0);
        let x = t_vector
            * BEZIER_D_MATRIX
            * Vector4::new(
                self.points.0.x,
                self.points.1.x,
                self.points.2.x,
                self.points.3.x,
            );
        let y = t_vector
            * BEZIER_D_MATRIX
            * Vector4::new(
                self.points.0.y,
                self.points.1.y,
                self.points.2.y,
                self.points.3.y,
            );

        Vector2::new(x.sum(), y.sum())
    }

    pub fn get_dd(&self, t: f64) -> Vector2<f64> {
        let t_vector = RowVector2::new(t, 1.0);
        let x = t_vector
            * BEZIER_DD_MATRIX
            * Vector4::new(
                self.points.0.x,
                self.points.1.x,
                self.points.2.x,
                self.points.3.x,
            );
        let y = t_vector
            * BEZIER_DD_MATRIX
            * Vector4::new(
                self.points.0.y,
                self.points.1.y,
                self.points.2.y,
                self.points.3.y,
            );

        Vector2::new(x.sum(), y.sum())
    }

    pub fn get_curvature(&self, t: f64) -> f64 {
        cross_2d(self.get_d(t), self.get_dd(t)) / self.get_d(t).magnitude().powi(3)
    }

    pub fn get_distance_by_t(&self, t: f64) -> f64 {
        interp(&self.dist_to_t.0, &self.dist_to_t.1, t)
    }

    pub fn get_t_by_distance(&self, dist: f64) -> f64 {
        interp(&self.dist_to_t.1, &self.dist_to_t.0, dist)
    }
}

impl<const D: usize> TryFrom<PathSegment> for Bezier<D> {
    type Error = BezierConstructionError;

    fn try_from(path: PathSegment) -> Result<Self, Self::Error> {
        match path.path.len().cmp(&4usize) {
            Ordering::Less => Err(TooFewPoints),
            Ordering::Equal => Ok(Self::new(
                (
                    path.path[0].into(),
                    path.path[1].into(),
                    path.path[2].into(),
                    path.path[3].into(),
                ),
                path.constraints.velocity,
                path.constraints.accel,
            )),
            Ordering::Greater => Err(TooManyPoints),
        }
    }
}

fn cross_2d(a: Vector2<f64>, b: Vector2<f64>) -> f64 {
    a.x * b.y - a.y * b.x
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::path::{Constraints, Point};
    use alloc::vec;
    use nalgebra::Vector2;

    #[test]
    fn test_bezier_new() {
        let points = (
            Vector2::new(0.0, 0.0),
            Vector2::new(1.0, 2.0),
            Vector2::new(2.0, 3.0),
            Vector2::new(3.0, 0.0),
        );
        let bezier = Bezier::<50>::new(points, 1.0, 0.5);

        assert_eq!(bezier.points.0, Vector2::new(0.0, 0.0));
        assert_eq!(bezier.vel, 1.0);
        assert_eq!(bezier.accel, 0.5);
        assert!(
            !bezier.dist_to_t.0.is_empty(),
            "dist_to_t.0 should not be empty after initialization"
        );
        assert!(
            !bezier.dist_to_t.1.is_empty(),
            "dist_to_t.1 should not be empty after initialization"
        );
    }

    #[test]
    fn test_bezier_get() {
        let points = (
            Vector2::new(0.0, 0.0),
            Vector2::new(0.25, 0.0),
            Vector2::new(0.75, 0.0),
            Vector2::new(1.0, 0.0),
        );
        let bezier = Bezier::<50>::new(points, 1.0, 0.5);

        let point_at_t = bezier.get(0.5);

        // Check that the result is a Vector3 with meaningful x, y, and angle values.
        assert_eq!(point_at_t.len(), 3);
        assert!(point_at_t.x.is_finite(), "X should be finite");
        assert!(point_at_t.y.is_finite(), "Y should be finite");
        assert!(point_at_t.z.is_finite(), "Angle should be finite");
        assert_eq!(point_at_t.x, 0.5);
        assert_eq!(point_at_t.y, 0.0);
        assert_eq!(point_at_t.z, 0.0);
    }

    #[test]
    fn test_bezier_get_d() {
        let points = (
            Vector2::new(0.0, 0.0),
            Vector2::new(1.0, 2.0),
            Vector2::new(2.0, 3.0),
            Vector2::new(3.0, 0.0),
        );
        let bezier = Bezier::<50>::new(points, 1.0, 0.5);

        let d_vector = bezier.get_d(0.5);

        assert!(d_vector.x.is_finite(), "X derivative should be finite");
        assert!(d_vector.y.is_finite(), "Y derivative should be finite");
    }

    #[test]
    fn test_bezier_get_dd() {
        let points = (
            Vector2::new(0.0, 0.0),
            Vector2::new(1.0, 2.0),
            Vector2::new(2.0, 3.0),
            Vector2::new(3.0, 0.0),
        );
        let bezier = Bezier::<50>::new(points, 1.0, 0.5);

        let dd_vector = bezier.get_dd(0.5);

        assert!(
            dd_vector.x.is_finite(),
            "X second derivative should be finite"
        );
        assert!(
            dd_vector.y.is_finite(),
            "Y second derivative should be finite"
        );
    }

    #[test]
    fn test_bezier_get_curvature() {
        let points = (
            Vector2::new(0.0, 0.0),
            Vector2::new(1.0, 2.0),
            Vector2::new(2.0, 3.0),
            Vector2::new(3.0, 0.0),
        );
        let bezier = Bezier::<50>::new(points, 1.0, 0.5);

        let curvature = bezier.get_curvature(0.5);

        assert!(curvature.is_finite(), "Curvature should be finite");
    }

    #[test]
    fn test_bezier_get_length() {
        let points = (
            Vector2::new(0.0, 0.0),
            Vector2::new(0.25, 0.0),
            Vector2::new(0.75, 0.0),
            Vector2::new(1.0, 0.0),
        );
        let bezier = Bezier::<50>::new(points, 1.0, 0.5);

        let length = bezier.get_length();
        let correct_length = 1.0;

        assert!(length.is_finite(), "Length should be finite");
        assert!((length - correct_length).abs() < correct_length * 0.01);
    }

    #[test]
    fn test_bezier_get_t_by_distance() {
        let points = (
            Vector2::new(0.0, 0.0),
            Vector2::new(0.25, 0.0),
            Vector2::new(0.75, 0.0),
            Vector2::new(1.0, 0.0),
        );
        let bezier = Bezier::<50>::new(points, 1.0, 0.5);

        let distance = bezier.get_t_by_distance(0.5);
        let correct_distance = 0.5;

        assert!(distance.is_finite(), "Distance should be finite");
        assert!(distance >= 0.0, "Distance should be non-negative");
        assert!(
            (distance - correct_distance).abs() < correct_distance * 0.05,
            "calculated: {}, correct: {}",
            distance,
            correct_distance
        );
    }

    #[test]
    fn test_bezier_get_distance_by_t() {
        let points = (
            Vector2::new(0.0, 0.0),
            Vector2::new(1.0, 2.0),
            Vector2::new(2.0, 3.0),
            Vector2::new(3.0, 0.0),
        );
        let bezier = Bezier::<50>::new(points, 1.0, 0.5);

        let distance = bezier.get_distance_by_t(0.5);

        assert!(distance.is_finite(), "Distance should be finite");
        assert!(distance >= 0.0, "Distance should be non-negative");
    }

    #[test]
    fn test_bezier_try_from_valid() {
        let path_segment = PathSegment {
            path: vec![
                Point { x: 0.0, y: 0.0 },
                Point { x: 1.0, y: 2.0 },
                Point { x: 2.0, y: 3.0 },
                Point { x: 3.0, y: 0.0 },
            ],
            constraints: Constraints {
                velocity: 1.0,
                accel: 1.0,
            },
            inverted: false,
            stop_end: false,
        };

        let bezier_result = Bezier::<50>::try_from(path_segment);

        assert!(
            bezier_result.is_ok(),
            "Should successfully create a Bezier from PathSegment"
        );
    }

    #[test]
    fn test_bezier_try_from_too_few_points() {
        let path_segment = PathSegment {
            path: vec![Point { x: 0.0, y: 0.0 }, Point { x: 1.0, y: 2.0 }],
            constraints: Constraints {
                velocity: 1.0,
                accel: 1.0,
            },
            inverted: false,
            stop_end: false,
        };

        let bezier_result = Bezier::<50>::try_from(path_segment);

        assert!(matches!(
            bezier_result,
            Err(BezierConstructionError::TooFewPoints)
        ));
    }

    #[test]
    fn test_bezier_try_from_too_many_points() {
        let path_segment = PathSegment {
            path: vec![
                Point { x: 0.0, y: 0.0 },
                Point { x: 1.0, y: 2.0 },
                Point { x: 2.0, y: 3.0 },
                Point { x: 3.0, y: 0.0 },
                Point { x: 4.0, y: 1.0 },
            ],
            constraints: Constraints {
                velocity: 1.0,
                accel: 1.0,
            },
            inverted: false,
            stop_end: false,
        };

        let bezier_result = Bezier::<50>::try_from(path_segment);

        assert!(matches!(
            bezier_result,
            Err(BezierConstructionError::TooManyPoints)
        ));
    }
}
