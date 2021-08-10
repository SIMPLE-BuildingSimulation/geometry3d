use crate::disk3d::Disk3D;
use crate::intersect_trait::{Intersect, IntersectionInfo};
use crate::point3d::Point3D;
use crate::ray3d::Ray3D;
use crate::transform::Transform;
use crate::vector3d::Vector3D;
use crate::{Float, PI};
use std::rc::Rc;

/// Represents a solid angle pointing from any [`Point3D`] in the scene towards
/// a certain `direction` (i.e., a [`Vector3D`]). So, this is not really a surface
/// because it is at an infinite distance from any point.
pub struct DistantSource3D {
    /// The direction pointing to the source
    pub direction: Vector3D,

    /// The solid angle in Sr
    pub omega: Float,

    /// The angle in Radians
    pub angle: Float,

    /// Cos half angle
    cos_half_alpha: Float,

    /// Tan of half angle
    tan_half_alpha: Float,

    /// A pointer to the [`Transform`] associated with this [`Sphere3D`]
    transform: Option<Rc<Transform>>,

    // Does the transform change the hand-ness of the coordinate system?
    // transform_reverses: bool,
}

impl DistantSource3D {
    /// Creates a new [`DistantSource3D`] geometry.
    ///
    /// # Inputs:
    /// * direction: A [`Vector3D`] pointing to the source
    /// * angle: The flat angle in Radians (e.g., a hemisphere would be $`\pi`$)
    pub fn new(direction: Vector3D, angle: Float) -> Self {
        let tan_half_alpha = (angle / 2.0).tan();
        let omega = tan_half_alpha * tan_half_alpha * PI;
        Self {
            omega,
            angle,
            tan_half_alpha,
            cos_half_alpha: (angle / 2.).cos(),
            direction: direction.get_normalized(),
            // transform_reverses: false,
            transform: None,
        }
    }

    /// Creates a proxy [`Disk3D`] that can be used for sampling and
    /// other operations
    pub fn get_proxy_disk(&self, t: Float) -> Disk3D {
        // Simulate a Disk3D and get that value.
        let center = (self.direction * t).as_point3d();
        let normal = self.direction;
        let radius = t * self.tan_half_alpha;

        Disk3D::new(center, normal, radius)
    }
}

impl Intersect for DistantSource3D {
    fn id(&self) -> &'static str {
        "source"
    }

    fn area(&self) -> Float {
        Float::MAX
    }

    fn transform(&self) -> &Option<Rc<Transform>> {
        &self.transform
    }

    fn intersect_local_ray(
        &self,
        ray: &Ray3D,
        o_error: Point3D,
        d_error: Point3D,
    ) -> Option<IntersectionInfo> {
        let phit = self.simple_intersect_local_ray(ray, o_error, d_error)?;
        let t = 10.;
        let phi = 0.5; // I don't think this value is relevant
        let disk = self.get_proxy_disk(t);
        // Get basic info and update
        let mut info = disk.intersection_info(ray, phit, phi)?;
        info.p = phit;
        Some(info)
    }

    fn simple_intersect_local_ray(
        &self,
        ray: &Ray3D,
        _o_error: Point3D,
        _d_error: Point3D,
    ) -> Option<Point3D> {
        // intersects as long as it is in the same direction...
        let cos_angle = ray.direction.get_normalized() * self.direction;
        if cos_angle >= self.cos_half_alpha {
            Some(ray.project(Float::MAX))
        } else {
            None
        }
    }

    // fn normal_at_intersection(&self, ray: &Ray3D, _t: Float) -> (Vector3D, SurfaceSide) {
    //     (ray.direction * -1., SurfaceSide::Front)
    // }

    // fn is_infinite(&self) -> bool {
    //     self.angle > 0.95 * PI
    // }

    // fn intersection_info(&self, ray: &Ray3D, _t: Float) -> IntersectionInfo {
    //     let t = 10.;
    //     let disk = self.get_proxy_disk(t);
    //     disk.intersection_info(ray, t)
    // }
}
