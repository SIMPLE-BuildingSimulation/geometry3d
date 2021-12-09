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
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERefCountHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

use crate::Float;
use crate::Point3D;

/// An Axis-aligned Bounding Box, represented by 
/// two corners (i.e., by two `Point3D` objects)
pub struct BBox3D {
    /// The lower corner
    pub min: Point3D, 

    /// Higher corner
    pub max: Point3D
}

/// Identifies the minimum and maximum `x`, `y`, `z`
fn get_mins_maxs(mut x1: Float, mut y1: Float, mut z1: Float, mut x2: Float, mut y2: Float, mut z2: Float)->(Float, Float, Float, Float, Float, Float){
    
    if x1 > x2 {
        std::mem::swap(&mut x1, &mut x2);
    }

    if y1 > y2 {
        std::mem::swap(&mut y1, &mut y2);
    }

    if z1 > z2 {
        std::mem::swap(&mut z1, &mut z2);
    }

    (x1,y1,z1,x2,y2,z2)
}

impl BBox3D {

    /// Creates a new `BBox3D` from two points. The points
    /// will be all scrambled up in order to find the minimum
    /// `x`, `y` and `z`.
    pub fn new(a: Point3D, b: Point3D)->Self{
        
        let (min_x, min_y, min_z, max_x, max_y, max_z) = get_mins_maxs(a.x, a.y, a.z, b.x, b.y, b.z);

        let min = Point3D::new(min_x, min_y, min_z);
        let max = Point3D::new(max_x, max_y, max_z);
        Self{
            min,max
        }
    }

    /// Creates a `BBox3D` that contains a single point, thus
    /// its `min` and its `max` values are equal
    pub fn from_point(p: Point3D)->Self{
        Self{
            min: p, max: p
        }
    }


    /// Creates a new `BBox3D` that contains an existing `BBox3D`
    /// and a `Point3D`
    pub fn from_union_point(bbox: &Self, pt: Point3D)->Self{
        
        let (min_x, min_y, min_z, .. ) = get_mins_maxs(bbox.min.x, bbox.min.y, bbox.min.z, pt.x, pt.y, pt.z);
        let (.., max_x, max_y, max_z ) = get_mins_maxs(bbox.max.x, bbox.max.y, bbox.max.z, pt.x, pt.y, pt.z);
        
        let min = Point3D::new(min_x, min_y, min_z);
        let max = Point3D::new(max_x, max_y, max_z);
        Self{
            min,max
        }
    }

    /// Creates a new `BBox3D` that contains two existing `BBox3D`    
    pub fn from_union(bbox1: &Self, bbox2: &Self)->Self{
        
        let (min_x, min_y, min_z, .. ) = get_mins_maxs(bbox1.min.x, bbox1.min.y, bbox1.min.z, bbox2.min.x, bbox2.min.y, bbox2.min.z);
        let (.., max_x, max_y, max_z ) = get_mins_maxs(bbox1.max.x, bbox1.max.y, bbox1.max.z, bbox2.max.x, bbox2.max.y, bbox2.max.z);
        
        let min = Point3D::new(min_x, min_y, min_z);
        let max = Point3D::new(max_x, max_y, max_z);
        Self{
            min,max
        }
    }

    /// Creates a new `BBox3D` that contains the intersection of two existing `BBox3D`    
    pub fn from_intersection(bbox1: &Self, bbox2: &Self)->Self{
        
        
        let (.., min_x, min_y, min_z)  = get_mins_maxs(bbox1.min.x, bbox1.min.y, bbox1.min.z, bbox2.min.x, bbox2.min.y, bbox2.min.z);        
        let (max_x, max_y, max_z , .. )= get_mins_maxs(bbox1.max.x, bbox1.max.y, bbox1.max.z, bbox2.max.x, bbox2.max.y, bbox2.max.z);
        
        let min = Point3D::new(min_x, min_y, min_z);
        let max = Point3D::new(max_x, max_y, max_z);
        Self{
            min,max
        }
    }

    /// Checks if a `BBox3D` overlaps with another one
    pub fn overlaps(&self, other: &Self)->bool{
        let x = (self.max.x >= other.min.x) && (self.min.x <= other.max.x);
        let y = (self.max.y >= other.min.y) && (self.min.y <= other.max.y);
        let z = (self.max.z >= other.min.z) && (self.min.z <= other.max.z);
        x && y && z
    }

    /// Checks if a `Point3D` is inside this `BBox3D`
    pub fn point_inside(&self, pt: Point3D)->bool{
        pt.x >= self.min.x && pt.x <= self.max.x &&
        pt.y >= self.min.y && pt.y <= self.max.y &&
        pt.z >= self.min.z && pt.z <= self.max.z
    }

    /// Checks if a `Point3D` is inside this `BBox3D` but any
    /// `Point3D` located in the upper boundary is considered
    /// to be outside
    pub fn point_inside_exclusive(&self, pt: Point3D)->bool{
        pt.x >= self.min.x && pt.x < self.max.x &&
        pt.y >= self.min.y && pt.y < self.max.y &&
        pt.z >= self.min.z && pt.z < self.max.z
    }
}


/***********/
/* TESTING */
/***********/

#[cfg(test)]
mod testing {
    use super::*;
    use crate::Vector3D;

    #[test]
    fn test_get_mins_maxs(){

        let min_x = 1.;
        let max_x = 41.;
        let min_y = -10.;
        let max_y = 1.;
        let min_z = 0.2;
        let max_z = 1.23;

        let (x1,y1,z1,x2,y2,z2) = get_mins_maxs(min_x, min_y, min_z, max_x, max_y, max_z);
        assert!( ( x1 - min_x ).abs() < 1e-15 );
        assert!( ( x2 - max_x ).abs() < 1e-15 );
        assert!( ( y1 - min_y ).abs() < 1e-15 );
        assert!( ( y2 - max_y ).abs() < 1e-15 );
        assert!( ( z1 - min_z ).abs() < 1e-15 );
        assert!( ( z2 - max_z ).abs() < 1e-15 );


        let (x1,y1,z1,x2,y2,z2) = get_mins_maxs(max_x, max_y, max_z, min_x, min_y, min_z);
        assert!( ( x1 - min_x ).abs() < 1e-15 );
        assert!( ( x2 - max_x ).abs() < 1e-15 );
        assert!( ( y1 - min_y ).abs() < 1e-15 );
        assert!( ( y2 - max_y ).abs() < 1e-15 );
        assert!( ( z1 - min_z ).abs() < 1e-15 );
        assert!( ( z2 - max_z ).abs() < 1e-15 );


        let (x1,y1,z1,x2,y2,z2) = get_mins_maxs(min_x, max_y, min_z, max_x, min_y, max_z);
        assert!( ( x1 - min_x ).abs() < 1e-15 );
        assert!( ( x2 - max_x ).abs() < 1e-15 );
        assert!( ( y1 - min_y ).abs() < 1e-15 );
        assert!( ( y2 - max_y ).abs() < 1e-15 );
        assert!( ( z1 - min_z ).abs() < 1e-15 );
        assert!( ( z2 - max_z ).abs() < 1e-15 );
    }

    #[test]
    fn test_new(){
        let a = Point3D::new(-1., 2., 3.);
        let b = Point3D::new(1., -2., 30.);

        let bbox = BBox3D::new(a,b);

        assert!( (a.x - bbox.min.x).abs() < 1e-15);
        assert!( (b.x - bbox.max.x).abs() < 1e-15);

        assert!( (b.y - bbox.min.y).abs() < 1e-15);
        assert!( (a.y - bbox.max.y).abs() < 1e-15);

        assert!( (a.z - bbox.min.z).abs() < 1e-15);
        assert!( (b.z - bbox.max.z).abs() < 1e-15);
    }

    #[test]
    fn test_from_point(){
        let a = Point3D::new(123., 1., 231.);
        let bbox = BBox3D::from_point(a);

        assert!( (bbox.min.x - a.x).abs() < 1e-15 );
        assert!( (bbox.min.y - a.y).abs() < 1e-15 );
        assert!( (bbox.min.z - a.z).abs() < 1e-15 );

        assert!( (bbox.max.x - a.x).abs() < 1e-15 );
        assert!( (bbox.max.y - a.y).abs() < 1e-15 );
        assert!( (bbox.max.z - a.z).abs() < 1e-15 );
    }    

    #[test]
    fn test_from_union_point(){
        let min = Point3D::new(0., 0., 0.);
        let max = Point3D::new(3., 5., 7.);

        let bbox1 = BBox3D::from_point(min);
        let bbox2 = BBox3D::from_union_point(&bbox1, max);
        assert!( (bbox2.min.x - min.x).abs() < 1e-15 );
        assert!( (bbox2.min.y - min.y).abs() < 1e-15 );
        assert!( (bbox2.min.z - min.z).abs() < 1e-15 );

        assert!( (bbox2.max.x - max.x).abs() < 1e-15 );
        assert!( (bbox2.max.y - max.y).abs() < 1e-15 );
        assert!( (bbox2.max.z - max.z).abs() < 1e-15 );

        let min = Point3D::new(-1., -1., -1.);
        let bbox2 = BBox3D::from_union_point(&bbox2, min);
        assert!( (bbox2.min.x - min.x).abs() < 1e-15 );
        assert!( (bbox2.min.y - min.y).abs() < 1e-15 );
        assert!( (bbox2.min.z - min.z).abs() < 1e-15 );

        assert!( (bbox2.max.x - max.x).abs() < 1e-15 );
        assert!( (bbox2.max.y - max.y).abs() < 1e-15 );
        assert!( (bbox2.max.z - max.z).abs() < 1e-15 );
        
    }

    #[test]
    fn test_from_union_box(){
        let min = Point3D::new(-3., -5., -7.);
        let center = Point3D::new(0., 0., 0.);
        let max = Point3D::new(3., 5., 7.);

        let bbox1 = BBox3D::new(min, center);
        let bbox2 = BBox3D::new(center, max);
        let bbox = BBox3D::from_union(&bbox1, &bbox2);

        assert!( (bbox.min.x - min.x).abs() < 1e-15 );
        assert!( (bbox.min.y - min.y).abs() < 1e-15 );
        assert!( (bbox.min.z - min.z).abs() < 1e-15 );

        assert!( (bbox.max.x - max.x).abs() < 1e-15 );
        assert!( (bbox.max.y - max.y).abs() < 1e-15 );
        assert!( (bbox.max.z - max.z).abs() < 1e-15 );
        
    }

    #[test]
    fn test_from_intersection(){
        let min = Point3D::new(-3., -5., -7.);        
        let max = Point3D::new(3., 5., 7.);

        let bbox1 = BBox3D::new(min, Point3D::new(999., 999., 999.));
        let bbox2 = BBox3D::new(Point3D::new(-999., -999., -999.), max);
        let bbox = BBox3D::from_intersection(&bbox1, &bbox2);

        assert!( (bbox.min.x - min.x).abs() < 1e-15 );
        assert!( (bbox.min.y - min.y).abs() < 1e-15 );
        assert!( (bbox.min.z - min.z).abs() < 1e-15 );

        assert!( (bbox.max.x - max.x).abs() < 1e-15 );
        assert!( (bbox.max.y - max.y).abs() < 1e-15 );
        assert!( (bbox.max.z - max.z).abs() < 1e-15 );
        
    }

    
    #[test]
    fn test_overlaps(){
        let min = Point3D::new(-3., -5., -7.);
        let center = Point3D::new(0., 0., 0.);
        let max = Point3D::new(3., 5., 7.);

        let bbox1 = BBox3D::new(min, center);
        let bbox2 = BBox3D::new(center, max);
        assert!(bbox1.overlaps(&bbox2));

        let min = Point3D::new(-3., -5., -7.);
        let center = Point3D::new(0., 0., 0.);
        let max = Point3D::new(3., 5., 7.);

        let bbox1 = BBox3D::new(min, center);
        let bbox2 = BBox3D::new(center + Vector3D::new(0.1, 0.1, 0.1), max);
        assert!(!bbox1.overlaps(&bbox2));

        let min = Point3D::new(-3., -5., -7.);
        let center = Point3D::new(0., 0., 0.);
        let max = Point3D::new(3., 5., 7.);

        let bbox1 = BBox3D::new(min, center);
        let bbox2 = BBox3D::new(center - Vector3D::new(0.1, 0.1, 0.1), max);
        assert!(bbox1.overlaps(&bbox2));
        
    }

    #[test]
    fn test_point_inside(){
        let min = Point3D::new(-3., -5., -7.);        
        let max = Point3D::new(3., 5., 7.);

        let bbox = BBox3D::new(min, max);
        
        assert!(bbox.point_inside(min));
        assert!(bbox.point_inside_exclusive(min));
        assert!(bbox.point_inside(max));
        assert!(!bbox.point_inside_exclusive(max));

        assert!(bbox.point_inside(Point3D::new(0., 0., 0.)));
        assert!(bbox.point_inside_exclusive(Point3D::new(0., 0., 0.)));


        assert!(!bbox.point_inside(Point3D::new(9e9, 9e9, 9e9)));
        assert!(!bbox.point_inside_exclusive(Point3D::new(9e9,9e9,9e9)));
        assert!(!bbox.point_inside(Point3D::new(-9e9, -9e9, -9e9)));
        assert!(!bbox.point_inside_exclusive(Point3D::new(-9e9,-9e9,-9e9)));

        
    }
}


