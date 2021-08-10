use crate::point3d::Point3D;
use crate::vector3d::Vector3D;
use crate::Float;

pub struct Ray3D {
    pub origin: Point3D,
    pub direction: Vector3D,
}

impl Ray3D {
    /// Returns the point that the [Ray3D] would be in after
    /// advancing `t`
    pub fn project(&self, t: Float) -> Point3D {
        self.origin + self.direction * t
    }

    /// Translates the [Ray3D] `t` units of its length
    /// into the future.
    pub fn advance(&mut self, t: Float) {
        self.origin = self.origin + self.direction * t
    }
}
