use crate::intersect_trait::{Intersect, SurfaceSide};
use crate::point3d::Point3D;
use crate::ray3d::Ray3D;
use crate::vector3d::Vector3D;

/// Describes a plane based on a [`Point3D`] (`P`) contained in the plane and
/// a [`Vector3D`] (`N`) normal to the plane.
///
/// A vector of these characteristics can also be described by an equation
/// ```math
/// 0 = Ax+By+Cz+D
/// ```
/// Where $`A = \vec{N}_x`$, $`B = \vec{N}_y`$, $`C = \vec{N}_z`$ and $`D = \vec{N}\cdot\vec{P}`$,
pub struct Plane3D {
    // A point contained in the plane
    //point: Point3D,
    /// Normal to the plane
    pub normal: Vector3D,

    /// The D coefficient in the equation explained earlier.    
    pub d: f64,
}

impl Plane3D {
    /// Creates a new plane
    pub fn new(point: Point3D, normal: Vector3D) -> Self {
        let mut normal = normal;
        normal.normalize();
        Self {
            normal,
            // point,
            d: normal * point,
        }
    }

    pub fn test_point(&self, point: Point3D) -> bool {
        (self.normal * point - self.d).abs() < f64::EPSILON
    }
}

impl Intersect for Plane3D {
    fn intersect(&self, ray: &Ray3D) -> Option<(f64, Vector3D, SurfaceSide)> {
        let den = self.normal * ray.direction;
        // They do not intercect
        if den.abs() < f64::EPSILON {
            return None;
        }
        let t = (self.d - self.normal * ray.origin) / den;
        if t < 0. {
            None
        } else {
            // return
            let (side, normal) = SurfaceSide::get_side(self.normal, ray.direction);
            Some((t, normal, side))
        }
    }
}

#[cfg(test)]
mod testing {
    use super::*;

    #[test]
    fn test_new() {
        let p = Point3D::new(1.2, 2.2, 2.1);
        let mut n = Vector3D::new(1.2, 3.1, 2.3);
        n.normalize();
        let plane = Plane3D::new(p, n);
        //assert!((plane.d - n*p).abs()<0.00000000001);
        assert_eq!(plane.d, n * p);
    }

    #[test]
    fn test_plane_intersect() {
        let p = Point3D::new(1.2, 2.2, 2.1);
        let mut n = Vector3D::new(0., 0., 1.);
        n.normalize();
        let plane = Plane3D::new(p, n);
        let ray = Ray3D {
            origin: Point3D::new(2.1, -3.1, 100.),
            direction: Vector3D::new(0., 0., -1.),
        };
        if let Some((t, normal, side)) = plane.intersect(&ray) {
            assert_eq!(side, SurfaceSide::Front);
            assert_eq!(normal, plane.normal);
            assert_eq!(t, 100. - p.z);
        } else {
            panic!("Did not intersect!")
        }
    }
}