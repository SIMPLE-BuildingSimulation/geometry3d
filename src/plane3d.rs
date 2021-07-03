use crate::point3d::Point3D;
use crate::vector3d::Vector3D;
use crate::intersect_trait::Intersect;
use crate::ray3d::Ray3D;

/// Describes a plane based on a [`Point3D`] (`P`) contained in the plane and 
/// a [`Vector3D`] (`N`) normal to the plane.
/// 
/// A vector of these characteristics can also be described by an equation
/// ```math
/// 0 = Ax+By+Cz+D
/// ```
/// Where $`A = \vec{N}_x`$, $`B = \vec{N}_y`$, $`C = \vec{N}_z`$ and $`D = \vec{N}\cdot\vec{P}`$, 
pub struct Plane3D{
    // A point contained in the plane
    //point: Point3D,
    
    /// Normal to the plane
    normal: Vector3D,

    /// The D coefficient in the equation explained earlier.    
    d: f64
}

impl Plane3D{    
    /// Creates a new plane
    pub fn new(point:Point3D, normal:Vector3D)->Self{
        let mut normal = normal;
        normal.normalize();
        Self{
            normal,
            // point,
            d: normal * point,
        }
    }

    pub fn test_point(&self, point:Point3D)->bool{
        (self.normal * point - self.d).abs() < f64::EPSILON
        
    }
}

impl Intersect for Plane3D{    
    fn intersect(&self, ray: &Ray3D)->Option<f64>{
        let den = self.normal*ray.direction();
        // They do not intercect
        if den.abs()<f64::EPSILON{
            return None;
        }
        let t = (self.normal * ray.origin() + self.d)/den;
        if t < 0. {
            None
        }else{
            // return
            Some(t)
        }
    }
}

