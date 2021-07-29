use crate::intersect_trait::{Intersect, SurfaceSide};
use crate::point3d::Point3D;
use crate::ray3d::Ray3D;
use crate::vector3d::Vector3D;

pub struct Sphere3D {
    pub centre: Point3D,
    pub radius: f64,
    pub r_squared: f64,
    centre_squared: f64,
}

impl Sphere3D {
    pub fn new(radius: f64, centre: Point3D) -> Self {
        Self {
            radius,
            centre,
            r_squared: radius * radius,
            centre_squared: centre * centre,
        }
    }
}

impl Intersect for Sphere3D {
    
    const ID : &'static str = "sphere";

    fn intersect(&self, ray: &Ray3D) -> Option<f64> {
        // Force ray.direction to be normalized
        debug_assert!((1.-ray.direction.length()).abs() < 100.*f64::EPSILON);
        // let a = ray.direction * ray.direction;        
        let b = 2. * (ray.origin * ray.direction - ray.direction * self.centre);
        let c = ray.origin * ray.origin - 2. * (ray.origin * self.centre) + self.centre_squared
            - self.r_squared;

        let discr = b * b - 4. */* a **/ c;
        if discr < 0.0 {
            return None;
        }
        let discr_sqrt = discr.sqrt();
        
        // Postpone the division
        let double_t1 = -b + discr_sqrt;// / 2./ a;
        let double_t2 = -b - discr_sqrt;// / 2./ a;

        
        // Return the smallest positive value.
        let t : f64;
        if double_t1 > 0. && double_t2 > 0. {
            if double_t1 > double_t2 {
                t =double_t2/2.;                
            } else {
                t = double_t1/2.;                
            }
        } else if double_t1 > 0. && double_t2 < 0. {
            t = double_t1/2.;            
        } else if double_t1 < 0. && double_t2 > 0. {
            t = double_t2/2.;            
        } else {
            // both negative
            return None
        }
        
        Some(t)
    }

    fn normal_at_intersection(&self, ray: &Ray3D, t: f64)->(Vector3D, SurfaceSide){
        let inter_point = ray.project(t);
        let mut r = inter_point - self.centre;        
        r.normalize();
        let (side, normal) = SurfaceSide::get_side(r, ray.direction);
        (normal, side)
    }
}

#[cfg(test)]
mod testing {
    use super::*;

    #[test]
    fn test_new() {
        let centre = Point3D::new(2.1, 1.2, -2.);
        let s = Sphere3D::new(std::f64::consts::PI, centre);

        assert_eq!(s.centre, centre);

        assert!((s.radius - std::f64::consts::PI).abs() < f64::EPSILON);
        assert!((s.r_squared - std::f64::consts::PI * std::f64::consts::PI).abs() < f64::EPSILON);
        assert!((s.centre_squared - centre * centre).abs() < f64::EPSILON);
    }

    #[test]
    fn test_sphere_intersect() {
        fn get_intersect(r: f64, centre: Point3D, p: Point3D) -> Option<Point3D> {
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
        if let Some(t) = sphere.intersect(&ray) {
            let (normal, side) = sphere.normal_at_intersection(&ray, t);
            let exp_t = 10. - r;
            assert!((t - exp_t).abs() < 100.*f64::EPSILON);
            assert!((exp.unwrap()- ray.project(t)).length() < 100.*f64::EPSILON);
            assert!( (normal-Vector3D::new(0., -1., 0.)).length()< 100.*f64::EPSILON);
            assert!(matches![side, SurfaceSide::Front]);
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
        if let Some(t) = sphere.intersect(&ray) {
            let (normal, side) = sphere.normal_at_intersection(&ray, t);
            let exp_t = r;
            assert!((t - exp_t).abs() < f64::EPSILON);
            assert!((exp.unwrap()- ray.project(t)).length() < 100.*f64::EPSILON);
            assert!((normal- Vector3D::new(0., 1., 0.)).length()< 100.*f64::EPSILON);
            assert!(matches![side, SurfaceSide::Back]);
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
        if let Some(t) = sphere.intersect(&ray) {
            let (normal, side) = sphere.normal_at_intersection(&ray, t);
            let exp_t = 10. - r;
            assert!((t - exp_t).abs() < f64::EPSILON);
            assert!( (exp.unwrap()- ray.project(t)).length() < 100.*f64::EPSILON);
            assert!( (normal- Vector3D::new(0., -1., 0.)).length()< 100.*f64::EPSILON);
            assert!(matches![side, SurfaceSide::Front]);
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
        if let Some(t) = sphere.intersect(&ray) {
            let (normal, side) = sphere.normal_at_intersection(&ray, t);
            let exp_t = r;
            assert!((t - exp_t).abs() < f64::EPSILON);
            assert!( (exp.unwrap()- ray.project(t)).length() < 100.*f64::EPSILON);
            assert!((normal- Vector3D::new(0., 1., 0.)).length()< 100.*f64::EPSILON);
            assert!(matches![side, SurfaceSide::Back]);
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
        if let Some(t) = sphere.intersect(&ray) {
            let (_normal, side) = sphere.normal_at_intersection(&ray, t);
            // let exp_t = 9.;
            // assert!((t - exp_t).abs()< f64::EPSILON);
            assert!((exp.unwrap() - ray.project(t)).length() < 0.0001);
            // assert_eq!(normal, Vector3D::new(0., -1., 0.));
            assert!(matches![side, SurfaceSide::Front]);
        } else {
            panic!("Wrong intersection!!")
        }

        // Test 6: Towards the centre from inside
        let r = 3.;
        let centre = Point3D::new(1.0, 0.0, -2.0);
        let origin = Point3D::new(0.9, 0.0, -0.05);
        let sphere = Sphere3D::new(r, centre);
        let exp = get_intersect(r, centre, origin);
        let ray = Ray3D {
            origin,
            direction: Vector3D::new(0., -1., 0.),
        };
        if let Some(t) = sphere.intersect(&ray) {
            let (_normal, side) = sphere.normal_at_intersection(&ray, t);
            // let exp_t = r;
            // assert!((t - exp_t).abs()< f64::EPSILON);
            assert_eq!(exp.unwrap(), ray.project(t));
            // assert_eq!(normal, Vector3D::new(0., 1., 0.));
            assert!(matches![side, SurfaceSide::Back]);
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
        if let Some(t) = sphere.intersect(&ray) {
            let (normal, side) = sphere.normal_at_intersection(&ray, t);
            let exp_t = 9.;
            assert!((t - exp_t).abs() < f64::EPSILON);
            assert!( (exp.unwrap()- ray.project(t)).length()< 100.*f64::EPSILON);
            assert!((normal- Vector3D::new(0., -1., 0.)).length()< 100.*f64::EPSILON);
            assert!(matches![side, SurfaceSide::Front]);
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
        if let Some(t) = sphere.intersect(&ray) {
            let (normal, side) = sphere.normal_at_intersection(&ray, t);
            let exp_t = r;
            assert!((t - exp_t).abs() < f64::EPSILON);
            assert!((exp.unwrap()- ray.project(t)).length()< 100.*f64::EPSILON);
            assert!((normal- Vector3D::new(0., 1., 0.)).length()< 100.*f64::EPSILON);
            assert!(matches![side, SurfaceSide::Back]);
        } else {
            panic!("Wrong intersection!!")
        }
    }
}
