use crate::ray3d::Ray3D;
use crate::vector3d::Vector3D;

/// Is the [`Ray3D`] intersecting from the Front or Back side?
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum SurfaceSide {
    Front,
    Back,
}

impl SurfaceSide {
    /// Given a `normal` at the intersection point, this function
    /// checks whether a [`Ray3D`] is arriving from the front or the back
    /// of the surface
    pub fn get_side(normal: Vector3D, ray_dir: Vector3D) -> (Self, Vector3D) {
        let dot = normal * ray_dir;
        debug_assert!((1.0 - normal.length()).abs() < 0.00000001);

        if dot < 0. {
            (Self::Front, normal)
        } else if dot > 0. {
            (Self::Back, normal * -1.)
        } else {
            // It is perpendicular, meaning that the dot product
            // gives us no information. We need a small hack: Deviate
            // the direction a bit, and try again

            // Let's get a vector that is normal to the direction.
            let normal_dir = if ray_dir.x.abs() > f64::EPSILON {
                Vector3D::new(-ray_dir.y / ray_dir.x, 1., 0.)
            } else if ray_dir.y.abs() > f64::EPSILON {
                Vector3D::new(1., -ray_dir.x / ray_dir.y, 0.)
            } else if ray_dir.z.abs() > f64::EPSILON {
                Vector3D::new(1., 0., -ray_dir.x / ray_dir.z)
            } else {
                panic!("Direction of the given Ray3D is a Zero vector");
            };

            let proxy_dir = ray_dir + normal_dir * 0.001;

            // Try again
            Self::get_side(normal, proxy_dir)
        }
    }
}

pub trait Intersect {

    /// The name of the figure. Useful for debugging.
    const ID : &'static str;

    /// Intercects a [`Ray3D]` traveling forward with an object, returning the distance
    /// `t` and the normal [`Vector3D`] at that point. If the distance
    /// is negative (i.e., the object is behind the plane), it should return
    /// [`None`].
    ///
    /// Do not asume that the ray will be normalized
    fn intersect(&self, ray: &Ray3D) -> Option<f64>;

    /// Gets the normal at the intersection, flipping it if necessary
    fn normal_at_intersection(&self, ray: &Ray3D, t: f64)->(Vector3D, SurfaceSide);

    /// Checks whether a certain object is infinite
    /// 
    /// defaults to `false`
    fn is_infinite(&self)->bool{
        false
    }
}

#[cfg(test)]
mod testing {
    use super::*;

    #[test]
    fn test_get_side() {
        // Normal points up
        let normal = Vector3D::new(0., 0., 1.);

        // Test 1: Hitting from above (should return front)
        let v = Vector3D::new(-1., 0., -1.);
        if let (SurfaceSide::Front, normal) = SurfaceSide::get_side(normal, v) {
            assert_eq!(normal, Vector3D::new(0., 0., 1.));
        } else {
            panic!("Expecting ray to come from the Front!")
        }

        // Test 2: Hitting from below (should return Back)
        let v = Vector3D::new(-1., 0., 1.);
        if let (SurfaceSide::Back, normal) = SurfaceSide::get_side(normal, v) {
            assert_eq!(normal, Vector3D::new(0., 0., -1.));
        } else {
            panic!("Expecting ray to come from the Back!")
        }

        // Test 3: Hitting NORMAL from above (should return front)
        let v = Vector3D::new(0., 0., -1.);
        if let (SurfaceSide::Front, _normal) = SurfaceSide::get_side(normal, v) {
            assert_eq!(normal, Vector3D::new(0., 0., 1.));
        } else {
            panic!("Expecting ray to come from the Front!")
        }

        // Test 4: Hitting NORMAL from below (should return Back)
        let v = Vector3D::new(0., 0., 1.);
        if let (SurfaceSide::Back, normal) = SurfaceSide::get_side(normal, v) {
            assert_eq!(normal, Vector3D::new(0., 0., -1.));
        } else {
            panic!("Expecting ray to come from the Front!")
        }
    }
}
