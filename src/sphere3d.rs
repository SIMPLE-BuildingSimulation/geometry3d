use crate::{Float, PI};
use std::rc::Rc;

use crate::intersect_trait::{Intersect, IntersectionInfo};
use crate::point3d::Point3D;
use crate::ray3d::Ray3D;
use crate::round_error::ApproxFloat;
use crate::transform::Transform;
use crate::vector3d::Vector3D;
// use crate::utils::solve_quadratic;

/// A Full or Partial sphere in three dimensions, centered at the origin
///
/// Everything is in Local coordinates. Where $`\phi`$ describes the
/// longitude (i.e., it goes from 0 to 360 degrees); and
/// $`\theta`$ defines the latitude (i.e., it goes from 0 to 190 degrees).
pub struct Sphere3D {
    /// The radius of the `Sphere3D`
    pub radius: Float,

    /// The position of the lower clipping plane on the Z axis
    zmin: Float,

    /// The posiftion of the upper clipping plane on the Z axis
    zmax: Float,

    /// The range of longitudes allowed for the sphere
    phi_max: Float,

    /// The range of Latitudes comprised within the allowed `zmin` and `zmax`
    delta_theta: Float,

    /// The minimum latitude allowed by `zmin`
    theta_min: Float,

    /// A pointer to the [`Transform`] associated with this [`Sphere3D`]
    transform: Option<Rc<Transform>>,

    // Does the transform change the hand-ness of the coordinate system?
    // transform_reverses: bool,
}

impl Sphere3D {
    /// Creates a new full [`Sphere3D`] of a certain `radius` and
    /// position its its `centre`.
    ///
    /// If the given `centre` is the origin
    /// (i.e., `Vector3D(0., 0., 0.)`), no [`Transform`] will be attached
    /// to this [`Sphere3D`]. If this is not the case, a `translate` [`Transform`]
    /// will be attached to it.
    pub fn new(radius: Float, centre: Point3D) -> Self {
        Self::new_partial(radius, centre, -2. * radius, 2. * radius, 360.)
    }

    /// Gets the [`Point3D`] of the [`Sphere3D`] that represents its
    /// centre, after transformation.
    pub fn centre(&self)-> Point3D {
        let o = Point3D::new(0.,0.,0.);
        match &self.transform{
            None=>o,
            Some(t)=>t.transform_pt(o)
        }
    }

    /// Creates a new full [`Sphere3D`] of a certain `radius` and
    /// transformed according to a [`Transform`].         
    pub fn new_transformed(radius: Float, transform: Option<Rc<Transform>>) -> Self {
        Self::new_partial_transformed(radius, -2. * radius, 2. * radius, 360., transform)
    }

    /// Creates a new potentially partial [`Sphere3D`] of a certain `radius` and
    /// position its its `centre`. It allows specifying the `zmin` and `zmax`
    /// as well as the `phi_max` (in degrees). The expected values for `phi_max`
    /// should go between 0 and 360 degress; while those for  
    /// `zmin` and `zmax` should go between `0` and plus or minus `radius`.
    ///
    /// If the given `centre` is the origin
    /// (i.e., `Vector3D(0., 0., 0.)`), no [`Transform`] will be attached
    /// to this [`Sphere3D`]. If this is not the case, a `translate` [`Transform`]
    /// will be attached to it.
    pub fn new_partial(
        radius: Float,
        centre: Point3D,
        zmin: Float,
        zmax: Float,
        phi_max: Float,
    ) -> Self {
        // Process transformation
        let mut transform: Option<Rc<Transform>> = None;
        if !centre.is_zero() {
            let tr = Transform::translate(centre.x, centre.y, centre.z);
            transform = Some(Rc::new(tr));
        }

        Self::new_partial_transformed(radius, zmin, zmax, phi_max, transform)
    }

    /// Creates a new potentially partial [`Sphere3D`] of a certain `radius` and
    /// transformed according to a [`Transform`]. It allows specifying the `zmin` and `zmax`
    /// as well as the `phi_max` (in degrees). The expected values for `phi_max`
    /// should go between 0 and 360 degress; while those for  
    /// `zmin` and `zmax` should go between `0` and plus or minus `radius`.         
    pub fn new_partial_transformed(
        radius: Float,
        zmin: Float,
        zmax: Float,
        phi_max: Float,
        transform: Option<Rc<Transform>>,
    ) -> Self {
        let mut zmin = zmin;
        let mut zmax = zmax;
        if zmin > zmax {
            panic!(
                "when creating a 'sphere': given zmin > zmax (i.e., {} > {})",
                zmin, zmax
            );
        }
        zmin = zmin.clamp(-radius, radius);
        zmax = zmax.clamp(-radius, radius);
        let mut theta_min = (zmin / radius).clamp(-1., 1.).acos();
        let mut theta_max = (zmax / radius).clamp(-1., 1.).acos();
        if theta_min > theta_max {
            std::mem::swap(&mut theta_min, &mut theta_max);
        }
        let mut phi_max = phi_max;
        if phi_max < -Float::EPSILON || phi_max > 360. + Float::EPSILON {
            panic!("when creating a 'sphere': given phi_max is not between 0 and 360 degrees (it was {})", phi_max);
        }
        phi_max = phi_max.clamp(0., 360.).to_radians();

        // let transform_reverses = match &transform {
        //     Some(t) => t.changes_hands(),
        //     None => false,
        // };

        Self {
            radius,
            zmin,
            zmax,
            phi_max,
            theta_min,
            delta_theta: theta_max - theta_min,
            // transform_reverses,
            transform: transform,
        }
    }

    // pub fn basic_intersection (
    //     &self,
    //     ray: &Ray3D,
    //     _o_error: Point3D,
    //     _d_error: Point3D,
    // ) -> Option<(Point3D, Float)> {
        
    //     let a = ray.direction * ray.direction;
    //     let b = 2.*(ray.origin * ray.direction);
    //     let c = ray.origin * ray.origin - self.radius*self.radius;

    //     let (t0,t1)=solve_quadratic(a,b,c)?;
    //     #[cfg(debug_assertions)]
    //     if t0.is_nan() || t1.is_nan() || t0.is_infinite() || t1.is_infinite(){
    //         panic!("After solve_quadratic in Sphere intersection: t0 = {}, t1 = {}", t0, t1);
    //     }
    //     debug_assert!(t1 >= t0 );
    //     // t0 < t1... so, check if they are possitive
    //     if t1 <= 0.0 {
    //         return None;
    //     }

    //     // We now know that t1 hits... check t0
    //     let (mut thit, hit_is_t1) = if t0 > 0. {
    //         // if t0 is a valid hit, keep that
    //         (t0, false)
    //     } else {
    //         // else, use t1 (which we know works...)
    //         (t1, true)
    //     };

    //     // We might try to do the same with 'thit' = t1, later
    //     let calc_phit_and_phi = |thit: Float| -> (Point3D, Float) {
    //         // Calculate point of intersection.
    //         let mut phit = ray.project(thit);
    //         // refine in order to avoid error accumulation
    //         phit *= self.radius / phit.as_vector3d().length();

    //         // Avoid a singularity on top of the sphere
    //         let limit = 1e-5 * self.radius;
    //         if phit.x.abs() < limit && phit.y.abs() < limit {
    //             phit.x = limit
    //         }

    //         // calc phi
    //         let mut phi = phit.y.atan2(phit.x);
    //         if phi < 0. {
    //             phi += 2. * PI;
    //         }
    //         (phit, phi)
    //     };
    //     let (mut phit, mut phi) = calc_phit_and_phi(thit);

    //     // Check intersection against clipping parameters...
    //     // it is possible that the first hit misses, but the second
    //     // does not
    //     if (self.zmin > -self.radius && phit.z < self.zmin) || // zmin is limiting but 'thit' missed it
    //         (self.zmax <  self.radius && phit.z > self.zmax) || // zmax is limiting but 'thit' missed it
    //         phi > self.phi_max
    //     // 'thit' missed due to the phi limitation
    //     {
    //         // if this was already t1, then we missed the sphere
    //         if hit_is_t1 {
    //             return None;
    //         }
    //         // else, try with t1.
    //         thit = t1;

    //         // recalculate
    //         let (new_phit, new_phi) = calc_phit_and_phi(thit);

    //         if (self.zmin > -self.radius && new_phit.z < self.zmin) || // zmin is limiting but 'thit' missed it
    //             (self.zmax <  self.radius && new_phit.z > self.zmax) || // zmax is limiting but 'thit' missed it
    //             new_phi > self.phi_max
    //         // 'thit' missed due to the phi limitation
    //         {
    //             return None;
    //         }
    //         // update values
    //         phit = new_phit;
    //         phi = new_phi;
    //     }

    //     Some((phit, phi))
    // }

    fn approx_basic_intersection(
        &self,
        ray: &Ray3D,
        o_error: Point3D,
        d_error: Point3D,
    ) -> Option<(Point3D, Float)> {
        // decompose ray
        let dx = ApproxFloat::from_value_and_error(ray.direction.x, d_error.x);
        let dy = ApproxFloat::from_value_and_error(ray.direction.y, d_error.y);
        let dz = ApproxFloat::from_value_and_error(ray.direction.z, d_error.z);
        let ox = ApproxFloat::from_value_and_error(ray.origin.x, o_error.x);
        let oy = ApproxFloat::from_value_and_error(ray.origin.y, o_error.y);
        let oz = ApproxFloat::from_value_and_error(ray.origin.z, o_error.z);

        
        let a = dx * dx + dy * dy + dz * dz;
        let b = (ox * dx + oy * dy + oz * dz) * 2.;
        let c = ox * ox + oy * oy + oz * oz - self.radius * self.radius;
        
        let (t0, t1) = ApproxFloat::solve_quadratic(a, b, c)?;
        #[cfg(debug_assertions)]
        if t0.as_float().is_nan() || t1.as_float().is_nan() || t0.as_float().is_infinite() || t1.as_float().is_infinite(){
            panic!("After solve_quadratic in Sphere intersection: t0.as_float() = {}, t1.as_float() = {}", t0.as_float(), t1.as_float());
        }        
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
            phit *= self.radius / phit.as_vector3d().length();

            // Avoid a singularity on top of the sphere
            let limit = 1e-5 * self.radius;
            if phit.x.abs() < limit && phit.y.abs() < limit {
                phit.x = limit
            }

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
        if (self.zmin > -self.radius && phit.z < self.zmin) || // zmin is limiting but 'thit' missed it
            (self.zmax <  self.radius && phit.z > self.zmax) || // zmax is limiting but 'thit' missed it
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

            if (self.zmin > -self.radius && new_phit.z < self.zmin) || // zmin is limiting but 'thit' missed it
                (self.zmax <  self.radius && new_phit.z > self.zmax) || // zmax is limiting but 'thit' missed it
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
        // auxiliar data
        let x = phit.x;
        let y = phit.y;
        let z = phit.z;
        // let zrad = (phit.x*phit.x + phit.y*phit.y).sqrt();
        // let inv_zrad = 1./zrad;

        let cos_theta = (z / self.radius).clamp(-1., 1.);
        let theta = cos_theta.acos();
        let sin_theta = theta.sin();
        let one_over_r_sin_theta = 1. / self.radius / sin_theta;
        let cos_phi = x * one_over_r_sin_theta;
        let sin_phi = y * one_over_r_sin_theta;

        // Calculate (u,v)
        let u = phi / self.phi_max;
        let v = (theta - self.theta_min) / self.delta_theta;

        // Calcuate first derivatives
        let dpdu = Vector3D::new(-self.phi_max * y, self.phi_max * x, 0.);
        let dpdv =
            Vector3D::new(z * cos_phi, z * sin_phi, -self.radius * sin_theta) * self.delta_theta;

        // Calculate second derivatives
        let d2p_duu = Vector3D::new(x, y, 0.) * (-self.phi_max * self.phi_max);
        let d2p_duv = Vector3D::new(-sin_phi, cos_phi, 0.) * (self.delta_theta * z * self.phi_max);
        let d2p_dvv = Vector3D::new(x, y, z) * (-self.delta_theta * self.delta_theta);

        // return
        Some(IntersectionInfo::new(
            ray, phit, u, v, dpdu, dpdv, d2p_duu, d2p_dvv, d2p_duv,
        ))
    }
}

impl Intersect for Sphere3D {
    fn id(&self) -> &'static str {
        "sphere"
    }

    fn area(&self) -> Float {
        self.phi_max * self.radius * (self.zmax - self.zmin)
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
        let (phit, _) = self.approx_basic_intersection(ray, o_error, d_error)?;
        Some(phit)
    }

    fn intersect_local_ray(
        &self,
        ray: &Ray3D,
        o_error: Point3D,
        d_error: Point3D,
    ) -> Option<IntersectionInfo> {
        let (phit, phi) = self.approx_basic_intersection(ray, o_error, d_error)?;

        self.intersection_info(ray, phit, phi)
    }

    // fn normal_at_intersection(&self, ray: &Ray3D, t: Float) -> (Vector3D, SurfaceSide) {
    //     let inter_point = ray.project(t);
    //     let mut r = inter_point.as_vector3d().get_normalized();
    //     SurfaceSide::get_side(r, ray.direction)
    // }
}

#[cfg(test)]
mod testing {
    use super::*;
    use crate::intersect_trait::SurfaceSide;

    #[test]
    fn test_new() {
        let centre = Point3D::new(2.1, 1.2, -2.);
        let s = Sphere3D::new(PI, centre);

        // assert_eq!(s.centre, centre);

        assert!((s.radius - PI).abs() < Float::EPSILON);
        assert!((s.radius * s.radius - PI * PI).abs() < Float::EPSILON);
        // assert!((s.centre_squared - centre * centre).abs() < Float::EPSILON);
    }

    #[test]
    fn test_sphere_centre() {
        let centre = Point3D::new(2.1, 1.2, -2.);
        let s = Sphere3D::new(PI, centre);
        assert_eq!(centre, s.centre());
        
    }


    #[test]
    fn test_sphere_area() {
        
        let r = 123.512;
        let full_area = 4.*PI*r*r;

        // Full sphere
        let s = Sphere3D::new_partial(r, Point3D::new(0., 0., 0.), -r, r, 360.);
        assert_eq!(s.area(), full_area);


        // Half sphere, due to Z
        let s = Sphere3D::new_partial(r, Point3D::new(0., 0., 0.), 0., r, 360.);
        assert_eq!(s.area(), full_area / 2.);
        let s = Sphere3D::new_partial(r, Point3D::new(0., 0., 0.), -r, 0., 360.);
        assert_eq!(s.area(), full_area / 2.);

        // Half sphere due to Phi
        let s = Sphere3D::new_partial(r, Point3D::new(0., 0., 0.), -r, r, 180.);
        assert_eq!(s.area(), full_area / 2.);

        // Quarter sphere due to Phi
        let s = Sphere3D::new_partial(r, Point3D::new(0., 0., 0.), -r, r, 90.);
        assert_eq!(s.area(), full_area / 4.);
        

    }

    

    #[test]
    fn test_sphere_intersect() {
        const ALLOWED_ERROR: Float = 0.00001;

        fn get_intersect(r: Float, centre: Point3D, p: Point3D) -> Option<Point3D> {
            let x_prime = p.x - centre.x;
            let z_prime = p.z - centre.z;
            // Check if we hit it at all.
            if z_prime.abs() >= r {
                return None;
            }
            let r_prime = (r * r - z_prime * z_prime).sqrt();
            if x_prime >= r_prime {
                return None;
            }

            // let y_plus =  centre.y + (r_prime*r_prime-x_prime*x_prime).sqrt();
            let y_minus = centre.y - (r_prime * r_prime - x_prime * x_prime).sqrt();
            Some(Point3D::new(p.x, y_minus, p.z))
        }

        fn floats_are_close(f1: Float, f2: Float) -> Result<(), String> {
            let d = (f1 - f2).abs();
            if d > ALLOWED_ERROR {
                return Err(format!(
                    "floats 'f1' ({}) and 'f2' ({}) are separated by {}... more than allowed",
                    f1, f2, d
                ));
            }
            Ok(())
        }

        fn points_are_close(p1: Point3D, p2: Point3D) -> Result<(), String> {
            let d = (p1 - p2).length();
            if d > ALLOWED_ERROR {
                return Err(format!(
                    "points 'p1' ({}) and 'p2' ({}) are separated by {}... more than allowed",
                    p1, p2, d
                ));
            }
            Ok(())
        }
        

        fn vectors_are_close(p1: Vector3D, p2: Vector3D) -> Result<(), String> {
            let d = (p1 - p2).length();
            if d > ALLOWED_ERROR {
                return Err(format!(
                    "points 'v1' ({}) and 'v2' ({}) are separated by {}... more than allowed",
                    p1, p2, d
                ));
            }
            Ok(())
        }

        // Test 1: Towards the centre outside
        let r = 1.4;
        let centre = Point3D::new(0.0, 0.0, 0.0);
        let origin = Point3D::new(0.0, -10.0, 0.0);
        let sphere = Sphere3D::new(r, centre);
        let exp = get_intersect(r, centre, origin);
        let ray = Ray3D {
            origin,
            direction: Vector3D::new(0., 1., 0.),
        };
        if let Some(info) = sphere.intersect(&ray) {
            let t = (info.p - ray.origin).length();

            //let (normal, side) = sphere.normal_at_intersection(&ray, t);
            let exp_t = 10. - r;
            floats_are_close(t, exp_t).unwrap();
            points_are_close(exp.unwrap(), info.p).unwrap();
            vectors_are_close(info.normal, Vector3D::new(0., -1., 0.)).unwrap();
            assert!(matches![info.side, SurfaceSide::Front]);
        } else {
            panic!("Wrong intersection!!")
        }

        // Test 2: Towards the centre from inside
        let r = 2.1;
        let centre = Point3D::new(0.0, 0.0, 0.0);
        let origin = Point3D::new(0.0, 0.0, 0.0);
        let sphere = Sphere3D::new(r, centre);
        let exp = get_intersect(r, centre, origin);
        let ray = Ray3D {
            origin,
            direction: Vector3D::new(0., -1., 0.),
        };
        if let Some(info) = sphere.intersect(&ray) {
            let t = (info.p - ray.origin).length();
            let exp_t = r;
            floats_are_close(t, exp_t).unwrap();
            points_are_close(exp.unwrap(), info.p).unwrap();
            vectors_are_close(info.normal, Vector3D::new(0., 1., 0.)).unwrap();
            assert!(matches![info.side, SurfaceSide::Back]);
        } else {
            panic!("Wrong intersection!!")
        }

        // Test 3: Towards the centre outside
        let r = 1.123;
        let centre = Point3D::new(3.0, 0.0, -2.0);
        let origin = Point3D::new(3.0, -10.0, -2.0);
        let sphere = Sphere3D::new(r, centre);
        let exp = get_intersect(r, centre, origin);
        let ray = Ray3D {
            origin,
            direction: Vector3D::new(0., 1., 0.),
        };
        if let Some(info) = sphere.intersect(&ray) {
            let t = (info.p - ray.origin).length();
            let exp_t = 10. - r;
            floats_are_close(t, exp_t).unwrap();
            points_are_close(exp.unwrap(), info.p).unwrap();
            vectors_are_close(info.normal, Vector3D::new(0., -1., 0.)).unwrap();
            assert!(matches![info.side, SurfaceSide::Front]);
        } else {
            panic!("Wrong intersection!!")
        }

        // Test 4: Towards the centre from inside
        let r = 1.2;
        let centre = Point3D::new(7.1, 0.0, 2.0);
        let origin = Point3D::new(7.1, 0.0, 2.0);
        let sphere = Sphere3D::new(r, centre);
        let exp = get_intersect(r, centre, origin);
        let ray = Ray3D {
            origin,
            direction: Vector3D::new(0., -1., 0.),
        };
        if let Some(info) = sphere.intersect(&ray) {
            let t = (info.p - ray.origin).length();
            let exp_t = r;
            floats_are_close(t, exp_t).unwrap();
            points_are_close(exp.unwrap(), info.p).unwrap();
            vectors_are_close(info.normal, Vector3D::new(0., 1., 0.)).unwrap();
            assert!(matches![info.side, SurfaceSide::Back]);
        } else {
            panic!("Wrong intersection!!")
        }

        //

        // Test 5: Removed from the center, from outside to inside
        let r = 2.;
        let centre = Point3D::new(1.0, 0.0, 1.0);
        let origin = Point3D::new(2.0, -10.0, 2.0);
        let sphere = Sphere3D::new(r, centre);
        let exp = get_intersect(r, centre, origin);
        let ray = Ray3D {
            origin,
            direction: Vector3D::new(0., 1., 0.),
        };
        if let Some(info) = sphere.intersect(&ray) {            
            points_are_close(exp.unwrap(), info.p).unwrap();
            assert!(matches![info.side, SurfaceSide::Front]);
            assert!(info.normal.z > 0.);
        } else {
            panic!("Wrong intersection!!")
        }

        // Test 6: Towards the centre from inside
        let r = 3.;
        let centre = Point3D::new(1.0, 0.0, -2.0);
        let origin = Point3D::new(1.0, 0.0, -0.05);
        let sphere = Sphere3D::new(r, centre);
        let exp = get_intersect(r, centre, origin);
        let ray = Ray3D {
            origin,
            direction: Vector3D::new(0., -1., 0.),
        };
        if let Some(info) = sphere.intersect(&ray) {
            points_are_close(exp.unwrap(), info.p).unwrap();
            assert!(matches![info.side, SurfaceSide::Back]);            
            assert!(info.normal.z < 0.);
        } else {
            panic!("Wrong intersection!!")
        }

        // Test 7: Towards the centre outside
        let r = 1.;
        let centre = Point3D::new(3.0, 0.0, -2.0);
        let origin = Point3D::new(3.0, -10.0, -2.0);
        let sphere = Sphere3D::new(r, centre);
        let exp = get_intersect(r, centre, origin);
        let ray = Ray3D {
            origin,
            direction: Vector3D::new(0., 1., 0.),
        };
        if let Some(info) = sphere.intersect(&ray) {
            let t = (info.p - ray.origin).length();
            let exp_t = 9.;
            floats_are_close(t, exp_t).unwrap();
            points_are_close(exp.unwrap(), info.p).unwrap();
            vectors_are_close(info.normal, Vector3D::new(0., -1., 0.)).unwrap();
            assert!(matches![info.side, SurfaceSide::Front]);
        } else {
            panic!("Wrong intersection!!")
        }

        // Test 8: Towards the centre from inside
        let r = 1.;
        let centre = Point3D::new(7.1, 0.0, 2.0);
        let origin = Point3D::new(7.1, 0.0, 2.0);
        let sphere = Sphere3D::new(r, centre);
        let exp = get_intersect(r, centre, origin);
        let ray = Ray3D {
            origin,
            direction: Vector3D::new(0., -1., 0.),
        };
        if let Some(info) = sphere.intersect(&ray) {
            let t = (info.p - ray.origin).length();

            let exp_t = r;
            floats_are_close(t, exp_t).unwrap();
            points_are_close(exp.unwrap(), info.p).unwrap();
            vectors_are_close(info.normal, Vector3D::new(0., 1., 0.)).unwrap();
            assert!(matches![info.side, SurfaceSide::Back]);
        } else {
            panic!("Wrong intersection!!")
        }
    }
}
