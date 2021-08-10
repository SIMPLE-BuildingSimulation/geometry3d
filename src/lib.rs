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

pub mod cylinder3d;
pub mod disk3d;
pub mod distant_source3d;
pub mod intersect_trait;
pub mod loop3d;
pub mod plane3d;
pub mod point3d;
pub mod polygon3d;
pub mod ray3d;
pub mod round_error;
pub mod segment3d;
pub mod sphere3d;
pub mod transform;
pub mod triangle3d;
pub mod triangulation3d;
mod utils;
pub mod vector3d;
