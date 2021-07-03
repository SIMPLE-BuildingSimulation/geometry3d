use crate::ray3d::Ray3D;

pub trait Intersect{
    /// Intercects a [`Ray3D]` traveling forward with an object. If the distance
    /// is negative (i.e., the object is behind the plane), it should return
    /// [`None`]
    fn intersect(&self, ray: &Ray3D)->Option<f64>;
}

