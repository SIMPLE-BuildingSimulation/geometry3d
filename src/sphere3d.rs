use crate::point3d::Point3D;
use crate::intersect_trait::Intersect;
use crate::ray3d::Ray3D;

pub struct Sphere3D{
    pub centre: Point3D,
    pub radius: f64,
    r_squared: f64,
    centre_squared: f64
}

impl Sphere3D {
    pub fn new(radius: f64, centre: Point3D)->Self{
        Self{
            radius,
            centre,
            r_squared: radius*radius,
            centre_squared: centre*centre,
        }
    }
}

impl Intersect for Sphere3D{
    fn intersect(&self, ray: &Ray3D)->Option<f64>{
        let a = ray.direction()*ray.direction();
        let b = 2.*(
            ray.origin()*ray.direction() - 
            ray.direction()*self.centre
        );
        let c = 
            ray.origin()*ray.origin()
            - 2.*(ray.origin()*self.centre)
            +self.centre_squared
            -self.r_squared;
        
        let discr = b*b - 4.*a*c;
        if discr < 0.0{
            return None
        }
        let t1 = (-b + discr.sqrt())/2./a;
        let t2 = (-b - discr.sqrt())/2./a;

        // Return the smallest positive value.
        if t1 > 0. && t2 > 0. {
            if t1 > t2{
                Some(t2)
            }else{
                Some(t1)
            }
        }else if t1>0. && t2 < 0. {
            Some(t1)
        }else if t1 < 0. && t2 > 0.{
            Some(t2)
        }else{
            // both negative
            None
        }
        
    }
}