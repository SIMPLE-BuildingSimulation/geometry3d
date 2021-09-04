/*
MIT License

Copyright (c) 2021 Germán Molina

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

use crate::{Float, PI};
use std::rc::Rc;

use crate::intersect_trait::{Intersect, IntersectionInfo, SurfaceSide};
use crate::plane3d::Plane3D;
use crate::point3d::Point3D;
use crate::ray3d::Ray3D;
use crate::transform::Transform;
use crate::vector3d::Vector3D;

/// A disk of radius `radius` whose normal points in the
/// `normal` direction, located at a height `height`. Can have a
pub struct Disk3D {
    /// The location of the center of the [`Disk3D`]
    centre: Point3D,

    /// The normal of the [`Disk3D`]
    normal: Vector3D,

    /// The external radius of the [`Disk3D`]
    radius: Float,

    /// The radius of the inner hole of the [`Disk3D`]
    inner_radius: Float,

    /// The axis that marks `phi` as Zero
    phi_zero: Vector3D,

    /// The angle described by the [`Disk3D`]. A value of
    /// `2.*PI` implies a full [`Disk3D`]; a value of less than
    /// that, a partial [`Disk3D`].
    phi_max: Float,

    /// A pointer to the [`Transform`] associated with this [`Sphere3D`]
    transform: Option<Rc<Transform>>,
    // Does the transform change the hand-ness of the coordinate system?
    // transform_reverses: bool,
}

impl Disk3D {
    pub fn new(centre: Point3D, normal: Vector3D, radius: Float) -> Self {
        Self::new_detailed(
            centre,
            normal,
            radius,
            0.,
            normal.get_perpendicular().unwrap(),
            360.,
            None,
        )
    }

    pub fn new_detailed(
        centre: Point3D,
        normal: Vector3D,
        radius: Float,
        inner_radius: Float,
        phi_zero: Vector3D,
        phi_max: Float,
        transform: Option<Rc<Transform>>,
    ) -> Self {
        // Normalize
        let normal = normal.get_normalized();
        // Check phi_zero
        if normal.is_parallel(phi_zero) {
            panic!(
                "when creating a 'disk': normal ({}) is parallel to phi_zero ({})",
                normal, phi_zero
            );
        }
        // Project phi_zero over the plane, so they are perpendicular.
        let phi_zero = (phi_zero - normal * (normal * phi_zero)).get_normalized();
        debug_assert!((phi_zero * normal).abs() < Float::EPSILON);

        // check radius consistency
        if radius <= inner_radius {
            panic!(
                "when creating a 'disk': radius ({}) is smaller or equal than inner_radius ({})",
                radius, inner_radius
            );
        }
        if radius < 0. {
            panic!("when creating a 'disk': radius ({}) is negative", radius);
        }
        if inner_radius < 0. {
            panic!(
                "when creating a 'disk': inner_radius ({}) is negative",
                inner_radius
            );
        }

        // Convert phi_max
        let phi_max = phi_max.clamp(0., 360.).to_radians();

        // Check transform
        // let transform_reverses = match &transform{
        //     Some(t)=>t.changes_hands(),
        //     None =>false,
        // };

        Self {
            centre,
            normal,
            radius,
            inner_radius,
            phi_zero,
            phi_max,
            transform,
            // transform_reverses,
        }
    }

    /// Intersects a [`Ray3D`] in local coordinates with the [`Disk3D`]. Returns the
    /// a [`Point3D`] of intersection and `phi`, the parameter indicating
    /// the polar position of the point of intersection.
    pub fn basic_intersection(
        &self,
        ray: &Ray3D,
        _o_error: Point3D,
        _d_error: Point3D,
    ) -> Option<(Point3D, Float)> {
        let disk_plane = Plane3D::new(self.centre, self.normal);
        let t = disk_plane.intersect(ray)?;

        // if it intersects the plane, and the point is within the radius
        let phit = ray.project(t);
        let r_squared = (phit - self.centre).length_squared();
        if r_squared > self.radius * self.radius
            || r_squared < self.inner_radius * self.inner_radius
        {
            return None;
        }

        // calc phi
        let zxn = self.phi_zero.cross(self.normal);
        let r = phit - self.centre;
        let x = r * self.phi_zero;
        let y = -r * zxn;

        let mut phi = y.atan2(x);
        if phi < 0. {
            phi += 2. * PI;
        }
        // return
        if phi > self.phi_max {
            None
        } else {
            Some((phit, phi))
        }
    }

    pub fn intersection_info(
        &self,
        ray: &Ray3D,
        phit: Point3D,
        phi: Float,
    ) -> Option<IntersectionInfo> {
        let r = phit - self.centre;
        let rhit = r.length();
        // Calculate (u,v)
        let u = phi / self.phi_max;
        let v = (self.radius - rhit) / (self.radius - self.inner_radius);

        // Calcuate first derivatives
        let zxn = self.phi_zero.cross(self.normal);
        let rhit_sin_phi = -r * zxn;
        let rhit_cos_phi = r * self.phi_zero;
        let dpdu = self.phi_zero * (-rhit_sin_phi)
            + zxn * (rhit_cos_phi / rhit * (self.inner_radius - self.radius));
        let dpdv = self.phi_zero * (-rhit_cos_phi)
            + zxn * (rhit_sin_phi / rhit * (self.radius - self.inner_radius));

        let normal = dpdv.cross(dpdu).get_normalized();
        let (normal, side) = SurfaceSide::get_side(normal, ray.direction);

        // We know some things are Zero... so let's not call the `IntersectionInfo::new(..)` function.
        Some(IntersectionInfo {
            p: phit,
            u,
            v,
            dpdu,
            dpdv,
            dndu: Vector3D::new(0., 0., 0.),
            dndv: Vector3D::new(0., 0., 0.),
            normal,
            side,
        })
    }
}

impl Intersect for Disk3D {
    fn id(&self) -> &'static str {
        "disk"
    }

    // fn normal_at_intersection(&self, ray: &Ray3D, t: Float) -> (Vector3D, SurfaceSide) {
    //     SurfaceSide::get_side(self.normal, ray.direction)
    // }

    fn area(&self) -> Float {
        self.phi_max * 0.5 * (self.radius * self.radius - self.inner_radius * self.inner_radius)
    }

    fn transform(&self) -> &Option<Rc<Transform>> {
        &self.transform
    }

    fn simple_intersect_local_ray(
        &self,
        ray: &Ray3D,
        o_error: Point3D,
        d_error: Point3D,
    ) -> Option<Point3D> {
        let (phit, _) = self.basic_intersection(ray, o_error, d_error)?;
        Some(phit)
    }

    fn intersect_local_ray(
        &self,
        ray: &Ray3D,
        o_error: Point3D,
        d_error: Point3D,
    ) -> Option<IntersectionInfo> {
        let (phit, phi) = self.basic_intersection(ray, o_error, d_error)?;

        self.intersection_info(ray, phit, phi)
    }
}

#[cfg(test)]
mod testing {
    use super::*;
    // use crate::intersect_trait::SurfaceSide;
    use crate::PI;

    #[test]
    fn test_disk_area() {
        let r = 4.212;
        let d = Disk3D::new(Point3D::new(0., 0., 0.), Vector3D::new(0., 0., 1.), r);
        assert_eq!(d.area(), PI * r * r);
    }
}
