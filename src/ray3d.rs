use crate::point3d::Point3D;
use crate::vector3d::Vector3D;

pub struct Ray3D {
    origin: Point3D,
    direction: Vector3D,
}

impl Ray3D{
    
    pub fn new(origin:Point3D, direction:Vector3D)->Self{
        let mut direction = direction;
        direction.normalize();
        Self{
            origin,
            direction
        }
    }

    /// Returns the point that the [Ray3D] would be in after
    /// advancing `t`
    pub fn advance(&self, t: f64)->Point3D{
        self.origin + self.direction * t
    }

    pub fn direction(&self)->Vector3D{
        self.direction
    }

    pub fn origin(&self)->Point3D{
        self.origin
    }
}
