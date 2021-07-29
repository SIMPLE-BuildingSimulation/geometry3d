
use crate::intersect_trait::{Intersect, SurfaceSide};
// use crate::point3d::Point3D;
use crate::ray3d::Ray3D;
use crate::vector3d::Vector3D;

/// Represents a solid angle pointing from any [`Point3D`] in the scene towards 
/// a certain `direction` (i.e., a [`Vector3D`]). So, this is not really a surface
/// because it is at an infinite distance from any point.
pub struct DistantSource3D {
    
    /// The direction pointing to the source
    pub direction: Vector3D,
    
    /// The solid angle in Sr
    pub omega: f64,

    /// The angle in Radians
    pub angle: f64,

    /// Cos half angle
    cos_half_alpha:f64
    
}

impl DistantSource3D {    
    /// Creates a new [`DistantSource3D`] geometry.
    ///
    /// # Inputs:
    /// * direction: A [`Vector3D`] pointing to the source
    /// * angle: The flat angle in Radians (e.g., a hemisphere would be $`\pi`$)
    pub fn new(direction: Vector3D, angle: f64) -> Self {
        let tan_half_angle = (angle / 2.0).tan();
        let omega = tan_half_angle * tan_half_angle * std::f64::consts::PI;        
        Self {
            omega,
            angle,
            cos_half_alpha: (angle/2.).cos(),
            direction: direction.get_normalized(),            
        }
    }

    
}

impl Intersect for DistantSource3D {
    const ID : &'static str = "source";
    
    fn intersect(&self, ray: &Ray3D) -> Option<f64> {
        debug_assert!((1.-ray.direction.length()).abs() < 100.*f64::EPSILON);
        // intersects as long as it is in the same direction... 
        let cos_angle = ray.direction * self.direction;
        if cos_angle >= self.cos_half_alpha{            
            Some(f64::MAX - 1.)
        }else{
            None
        }        
    }

    fn normal_at_intersection(&self, ray: &Ray3D, _t: f64)->(Vector3D, SurfaceSide){
        (ray.direction * -1., SurfaceSide::Front)
    }

    fn is_infinite(&self)->bool{
        self.angle > 0.95 * std::f64::consts::PI
    }
}
