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

// Define whether we are working with
// doubles (i.e., f64) or floats (i.e., f32)

/// The kind of Floating point number used in the
/// library... the `"float"` feature means it becomes `f32`
/// and `f64` is used otherwise.
#[cfg(feature = "float")]
type Float = f32;
#[cfg(feature = "float")]
const PI: Float = std::f32::consts::PI;

#[cfg(not(feature = "float"))]
type Float = f64;
#[cfg(not(feature = "float"))]
const PI: Float = std::f64::consts::PI;

#[cfg(feature = "parallel")]
type RefCount<T> = std::sync::Arc<T>;
#[cfg(not(feature = "parallel"))]
type RefCount<T> = std::rc::Rc<T>;

mod utils;
// pub mod intersect_trait;
pub mod intersection;
pub mod round_error;


// Objects
mod cylinder3d;
pub use cylinder3d::Cylinder3D;

mod disk3d;
pub use disk3d::Disk3D;

mod distant_source3d;
pub use distant_source3d::DistantSource3D;

mod loop3d;
pub use loop3d::Loop3D;

mod plane3d;
pub use plane3d::Plane3D;

mod point3d;
pub use point3d::Point3D;

mod polygon3d;
pub use polygon3d::Polygon3D;

mod ray3d;
pub use ray3d::Ray3D;

mod segment3d;
pub use segment3d::Segment3D;

mod sphere3d;
pub use sphere3d::Sphere3D;

mod transform;
pub use transform::Transform;

mod triangle3d;
pub use triangle3d::{Triangle3D, PointInTriangle};

mod triangulation3d;
pub use triangulation3d::Triangulation3D;

mod vector3d;
pub use vector3d::Vector3D;


mod bbox3d;
pub use bbox3d::{BBox3D, BBoxAxis};
