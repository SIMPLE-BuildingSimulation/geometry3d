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


use crate::{
    Point3D,
    Ray3D,
    Transform,
    Vector3D,
    Float,
};


/// Is the [`Ray3D`] intersecting from the Front or Back side?
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum SurfaceSide {
    Front,
    Back,
    NonApplicable,
}

impl SurfaceSide {
    /// Given a `normal` at the intersection point, this function
    /// checks whether a [`Ray3D`] is arriving from the front or the back
    /// of the surface
    pub fn get_side(normal: Vector3D, ray_dir: Vector3D) -> (Vector3D, Self) {
        debug_assert!(normal.length().abs() > 0.00000001);
        // eprintln!("raydir = {}", ray_dir);
        let dot = normal * ray_dir;

        if dot < 0. {
            (normal, Self::Front)
        } else if dot > 0. {
            (normal * -1., Self::Back)
        } else {   
            return (Vector3D::new(0., 0., 0.), Self::NonApplicable)                     
        }
    }
}



/// Contains more detailed information about the
/// what is happening at the surface in the intersection point
pub struct IntersectionInfo {
    /// The point of intersection
    pub p: Point3D,

    // pub perror: Point3D,
    /// The normal [`Vector3D`] at the interaction.
    /// Must have the value of (dpdu x dpdv).normalize() ... use
    /// ShadingInfo::new(..) and this is done automatically
    pub normal: Vector3D,

    /// The side in which the ray intersected
    pub side: SurfaceSide,

    /// Partial derivative of the position `p` with respect to `u`
    pub dpdu: Vector3D,

    /// Partial derivative of the position `p` with respect to `v`
    pub dpdv: Vector3D,

    /// Partial derivative of the normal `n` with respect to u
    pub dndu: Vector3D,

    /// Partial derivative of the normal `n` with respect to v
    pub dndv: Vector3D,

    /// The position `u` of the intersection point
    pub u: Float,

    /// The position `v` of the intersection point
    pub v: Float,
}

impl IntersectionInfo {
    pub fn new(
        ray: &Ray3D,
        p: Point3D,
        u: Float,
        v: Float,
        dpdu: Vector3D,
        dpdv: Vector3D,
        d2p_duu: Vector3D,
        d2p_dvv: Vector3D,
        d2p_duv: Vector3D,
    ) -> Self {
        /* Weingarten equations */
        let big_e = dpdu * dpdu;
        let big_f = dpdu * dpdv;
        let big_g = dpdv * dpdv;
        let normal = dpdv.cross(dpdu).get_normalized();
        let (normal, side) = SurfaceSide::get_side(normal, ray.direction);

        let e = normal * d2p_duu;
        let f = normal * d2p_duv;
        let g = normal * d2p_dvv;

        let inv_big_egf2 = 1. / (big_e * big_g - big_f * big_f);
        let dndu = dpdu * ((f * big_f - e * big_g) * inv_big_egf2)
            + dpdv * ((e * big_f - f * big_e) * inv_big_egf2);
        let dndv = dpdu * ((g * big_f - f * big_g) * inv_big_egf2)
            + dpdv * ((f * big_f - g * big_e) * inv_big_egf2);

        // return
        Self {
            p,
            normal,
            side,
            dpdu,
            dpdv,
            dndu,
            dndv,
            u,
            v,
        }
    }

    pub fn transform(&self, transform: &Transform) -> Self {
        Self {
            p: transform.transform_pt(self.p),
            u: self.u,
            v: self.v,
            dpdu: transform.transform_vec(self.dpdu),
            dpdv: transform.transform_vec(self.dpdv),
            dndu: transform.transform_vec(self.dndu),
            dndv: transform.transform_vec(self.dndv),
            normal: transform.transform_normal(self.normal),
            side: self.side,
        }
    }

    pub fn inv_transform(&self, transform: &Transform) -> Self {
        Self {
            p: transform.inv_transform_pt(self.p),
            u: self.u,
            v: self.v,
            dpdu: transform.inv_transform_vec(self.dpdu),
            dpdv: transform.inv_transform_vec(self.dpdv),
            dndu: transform.inv_transform_vec(self.dndu),
            dndv: transform.inv_transform_vec(self.dndv),
            normal: transform.inv_transform_normal(self.normal),
            side: self.side,
        }
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
        if let (normal, SurfaceSide::Front) = SurfaceSide::get_side(normal, v) {
            assert_eq!(normal, Vector3D::new(0., 0., 1.));
        } else {
            panic!("Expecting ray to come from the Front!")
        }

        // Test 2: Hitting from below (should return Back)
        let v = Vector3D::new(-1., 0., 1.);
        if let (normal, SurfaceSide::Back) = SurfaceSide::get_side(normal, v) {
            assert_eq!(normal, Vector3D::new(0., 0., -1.));
        } else {
            panic!("Expecting ray to come from the Back!")
        }

        // Test 3: Hitting NORMAL from above (should return front)
        let v = Vector3D::new(0., 0., -1.);
        if let (_normal, SurfaceSide::Front) = SurfaceSide::get_side(normal, v) {
            assert_eq!(normal, Vector3D::new(0., 0., 1.));
        } else {
            panic!("Expecting ray to come from the Front!")
        }

        // Test 4: Hitting NORMAL from below (should return Back)
        let v = Vector3D::new(0., 0., 1.);
        if let (normal, SurfaceSide::Back) = SurfaceSide::get_side(normal, v) {
            assert_eq!(normal, Vector3D::new(0., 0., -1.));
        } else {
            panic!("Expecting ray to come from the Front!")
        }
    }
}