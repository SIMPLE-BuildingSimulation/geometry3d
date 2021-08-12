use crate::point3d::Point3D;
use crate::ray3d::Ray3D;
use crate::transform::Transform;
use crate::vector3d::Vector3D;
use crate::Float;
use std::rc::Rc;

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
    pub fn get_side(normal: Vector3D, ray_dir: Vector3D) -> (Vector3D, Self) {
        debug_assert!(normal.length().abs() > 0.00000001);
        let dot = normal * ray_dir;

        if dot < 0. {
            (normal, Self::Front)
        } else if dot > 0. {
            (normal * -1., Self::Back)
        } else {
            println!("overtrying get_normal... side... Normal = {}, raydir = {} | N*ray_dir = {}", normal, ray_dir, normal * ray_dir);
            // It is perpendicular, meaning that the dot product
            // gives us no information. We need a small hack: Deviate
            // the direction a bit, and try again

            // Let's get a vector that is normal to the direction.
            let normal_dir = if ray_dir.x.abs() > Float::EPSILON {
                Vector3D::new(-ray_dir.y / ray_dir.x, 1., 0.)
            } else if ray_dir.y.abs() > Float::EPSILON {
                Vector3D::new(1., -ray_dir.x / ray_dir.y, 0.)
            } else if ray_dir.z.abs() > Float::EPSILON {
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
            u,
            v,
            dpdu,
            dpdv,
            dndu,
            dndv,
            normal,
            side,
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

pub trait Intersect {
    /// The name of the figure. Useful for debugging.
    fn id(&self) -> &'static str;

    /// Gets the area of the object
    fn area(&self) -> Float;

    /// Borrows the [`Transform`]
    fn transform(&self) -> &Option<Rc<Transform>>;

    /// Intersects an object with a [`Ray3D]` (IN LOCAL COORDINATES) traveling forward, returning the distance
    /// `t` and the normal [`Vector3D`] at that point. If the distance
    /// is negative (i.e., the object is behind the plane), it should return
    /// [`None`].    
    ///
    ///  The steps should be:
    /// * Calculate the point of intersection
    /// * Calculate normal and side of intersection
    /// * Calculate (u,v)
    /// * Calculate first derivative (i.e., dp_du, dp_dv)
    /// * Calculate second derivative (i.e., d2p_duu, d2p_dvv, d2p_duv)
    /// * Call `IntersectionInfo::new(ray,p, u, v, dpdu, dpdv, d2p_duu, d2p_dvv, d2p_duv)`
    fn intersect_local_ray(
        &self,
        ray: &Ray3D,
        o_error: Point3D,
        d_error: Point3D,
    ) -> Option<IntersectionInfo>;

    /// Like [`intersect_local_ray`] but simplified because there is not need
    /// for calcuating the paramtrisized elements
    fn simple_intersect_local_ray(
        &self,
        ray: &Ray3D,
        o_error: Point3D,
        d_error: Point3D,
    ) -> Option<Point3D>;

    /// Intersects an object with a [`Ray3D]` (IN WORLD COORDINATES) traveling forward, returning the distance
    /// `t` and the normal [`Vector3D`] at that point. If the distance
    /// is negative (i.e., the object is behind the plane), it should return
    /// [`None`]. Returns detailed [`IntersectionInfo`] about the intersaction .
    fn intersect(&self, ray: &Ray3D) -> Option<IntersectionInfo> {
        // Transform ray into object space, if needed
        let (local_ray, o_error, d_error) = if let Some(t) = self.transform() {
            t.inv_transform_ray(ray)
        } else {
            let t = Transform::new();
            t.inv_transform_ray(ray)
        };

        // Intersect, and return transformed back... if needed
        match self.intersect_local_ray(&local_ray, o_error, d_error) {
            None => None,
            Some(info) => {
                if let Some(t) = self.transform() {
                    Some(info.transform(t))
                } else {
                    Some(info)
                }
            }
        }
    }

    /// Intersects an object with a [`Ray3D]` (IN WORLD COORDINATES) traveling forward, returning the distance
    /// `t` and the normal [`Vector3D`] at that point. If the distance
    /// is negative (i.e., the object is behind the plane), it should return
    /// [`None`]. Returns only the point of intersection.
    fn simple_intersect(&self, ray: &Ray3D) -> Option<Point3D> {
        // Transform ray into object space, if needed
        let (local_ray, o_error, d_error) = if let Some(t) = self.transform() {
            t.inv_transform_ray(ray)
        } else {
            let t = Transform::new();
            t.inv_transform_ray(ray)
        };

        // Intersect, and return transformed back... if needed
        match self.simple_intersect_local_ray(&local_ray, o_error, d_error) {
            None => None,
            Some(phit) => {
                if let Some(t) = self.transform() {
                    Some(t.transform_pt(phit))
                } else {
                    Some(phit)
                }
            }
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
