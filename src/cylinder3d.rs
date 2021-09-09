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

use crate::round_error::ApproxFloat;
use crate::{Float, PI};

use crate::intersect_trait::{Intersect, IntersectionInfo};
use crate::point3d::Point3D;
use crate::ray3d::Ray3D;
use crate::transform::Transform;
use crate::vector3d::Vector3D;
use std::rc::Rc;

/// A Cylinder of radius `radius` aligned witht the Z axis, starting at a
/// Z value of `zmin` and ending at `zmax`.
///
/// It can be an 'open' cylinder, by setting a `phi_max` less than `2.*PI`
pub struct Cylinder3D {
    /// The radius of the `Cylinder3D`
    radius: Float,

    /// The Z coordinate of the lower end of the [`Cylinder3D`]
    zmin: Float,

    /// The Z coordinate of the upper end of the [`Cylinder3D`]
    zmax: Float,

    /// The range of longitudes allowed (2*PI radians in a full [`Cylinder3D`], less in a partial one)
    phi_max: Float,

    /// A pointer to the [`Transform`] associated with this [`Cylinder3D`]
    transform: Option<Rc<Transform>>,
    // Does the transform change the hand-ness of the coordinate system?
    // transform_reverses: bool,
}

impl Cylinder3D {
    pub fn new(p0: Point3D, p1: Point3D, radius: Float) -> Self {
        Self::new_partial(p0, p1, radius, 360.)
    }

    pub fn new_partial(p0: Point3D, p1: Point3D, radius: Float, phi_max: Float) -> Self {
        let l = p1 - p0;
        let (x, y, z) = (l.x, l.y, l.z);

        let mut transform = Transform::translate(p0.x, p0.y, p0.z);

        // Rotate...?
        let rot_y_degrees = (x * x + y * y).sqrt().atan2(z).to_degrees();
        let rot_z_degrees = y.atan2(x).to_degrees();

        transform *= Transform::rotate_y(rot_y_degrees);
        transform *= Transform::rotate_z(rot_z_degrees);

        Self::new_transformed(radius, 0., l.length(), phi_max, Some(Rc::new(transform)))
    }

    /// The angle `phi_max` is in degrees
    pub fn new_transformed(
        radius: Float,
        zmin: Float,
        zmax: Float,
        phi_max: Float,
        transform: Option<Rc<Transform>>,
    ) -> Self {
        if zmin > zmax {
            panic!(
                "when creating a 'cylinder': given zmin > zmax (i.e., {} > {})",
                zmin, zmax
            );
        }
        let mut phi_max = phi_max;        
        if !(-Float::EPSILON..=360. + Float::EPSILON).contains(&phi_max){
            panic!("when creating a 'cylinder': given phi_max is not between 0 and 360 degrees (it was {})", phi_max);
        }
        phi_max = phi_max.clamp(0., 360.).to_radians();

        // let transform_reverses = match &transform {
        //     Some(e) => e.changes_hands(),
        //     None => false,
        // };

        Self {
            radius,
            zmin,
            zmax,
            phi_max,
            transform,
            // transform_reverses,
        }
    }

    pub fn basic_intersection(
        &self,
        ray: &Ray3D,
        o_error: Point3D,
        d_error: Point3D,
    ) -> Option<(Point3D, Float)> {
        // decompose ray
        let dx = ApproxFloat::from_value_and_error(ray.direction.x, d_error.x);
        let dy = ApproxFloat::from_value_and_error(ray.direction.y, d_error.y);
        // let dz = ApproxFloat::from_value_and_error(ray.direction.z, d_error.z);
        let ox = ApproxFloat::from_value_and_error(ray.origin.x, o_error.x);
        let oy = ApproxFloat::from_value_and_error(ray.origin.y, o_error.y);
        // let oz = ApproxFloat::from_value_and_error(ray.origin.z, o_error.z);

        let a = dx * dx + dy * dy;
        let b = (dx * ox + dy * oy) * 2.;
        let c = ox * ox + oy * oy - self.radius * self.radius;
        let (t0, t1) = ApproxFloat::solve_quadratic(a, b, c)?;
        debug_assert!(t1.as_float() >= t0.as_float());
        // t0 < t1... so, check if they are possitive
        if t1.low <= 0.0 {
            return None;
        }

        // We now know that t1 hits... check t0
        let (mut thit, hit_is_t1) = if t0.low > 0. {
            // if t0 is a valid hit, keep that
            (t0, false)
        } else {
            // else, use t1 (which we know works...)
            (t1, true)
        };

        // We might try to do the same with 'thit' = t1, later
        let calc_phit_and_phi = |thit: ApproxFloat| -> (Point3D, Float) {
            // Calculate point of intersection.
            let mut phit = ray.project(thit.as_float());
            // refine in order to avoid error accumulation
            let hit_rad = (phit.x * phit.x + phit.y * phit.y).sqrt();
            phit.x *= self.radius / hit_rad;
            phit.y *= self.radius / hit_rad;

            // calc phi
            let mut phi = phit.y.atan2(phit.x);
            if phi < 0. {
                phi += 2. * PI;
            }
            (phit, phi)
        };

        let (mut phit, mut phi) = calc_phit_and_phi(thit);

        // Check intersection against clipping parameters...
        // it is possible that the first hit misses, but the second
        // does not
        if phit.z < self.zmin || // zmin is limiting but 'thit' missed it
            phit.z > self.zmax || // zmax is limiting but 'thit' missed it
            phi > self.phi_max
        // 'thit' missed due to the phi limitation
        {
            // if this was already t1, then we missed the sphere
            if hit_is_t1 {
                return None;
            }

            // else, try with t1.
            thit = t1;

            // recalculate
            let (new_phit, new_phi) = calc_phit_and_phi(thit);

            if new_phit.z < self.zmin || // zmin is limiting but 'thit' missed it
                new_phit.z > self.zmax || // zmax is limiting but 'thit' missed it
                new_phi > self.phi_max
            // 'thit' missed due to the phi limitation
            {
                return None;
            }
            // update values
            phit = new_phit;
            phi = new_phi;
        }

        Some((phit, phi))
    }

    pub fn intersection_info(
        &self,
        ray: &Ray3D,
        phit: Point3D,
        phi: Float,
    ) -> Option<IntersectionInfo> {
        // Calculate (u,v)
        let u = phi / self.phi_max;
        let v = (phit.z - self.zmin) / (self.zmax - self.zmin);

        // Calcuate first derivatives
        let dpdu = Vector3D::new(-self.phi_max * phit.y, self.phi_max * phit.x, 0.);
        let dpdv = Vector3D::new(0., 0., self.zmax - self.zmin);

        // Calculate second derivatives
        let d2p_duu = Vector3D::new(phit.x, phit.y, 0.) * (-self.phi_max * self.phi_max);
        let d2p_dvv = Vector3D::new(0., 0., 0.);
        let d2p_duv = Vector3D::new(0., 0., 0.);

        // return
        Some(IntersectionInfo::new(
            ray, phit, u, v, dpdu, dpdv, d2p_duu, d2p_dvv, d2p_duv,
        ))
    }
}

impl Intersect for Cylinder3D {
    fn id(&self) -> &'static str {
        "cylinder"
    }

    fn area(&self) -> Float {
        debug_assert!(self.zmax > self.zmin);
        (self.zmax - self.zmin) * self.radius * self.phi_max
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
        // Do the first part
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
    use crate::PI;
    // use crate::intersect_trait::SurfaceSide;

    #[test]
    fn test_cylinder_area() {
        let r = 2.1;
        let l = 3.2;
        let full_area = 2. * PI * r * l;

        // Full cylinder
        let c = Cylinder3D::new_transformed(r, 0., l, 360., None);
        assert_eq!(c.area(), full_area);

        // Half a cylinder due to phi
        let c = Cylinder3D::new_transformed(r, 0., l, 180., None);
        assert_eq!(c.area(), full_area / 2.);
    }
}
