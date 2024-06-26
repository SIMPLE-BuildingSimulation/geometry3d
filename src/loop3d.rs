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

use serde::ser::SerializeSeq;
use serde::{Deserialize, Deserializer, Serialize};
use serde_json::Value;

use crate::Float;
use crate::{Point3D, Segment3D, Vector3D};

/// A set of [`Point3D`] in sequence, forming a closed loop.
/// It has some particularities.
///
/// ```
/// use geometry3d::{Loop3D, Point3D};
/// let mut the_loop = Loop3D::new();
/// assert!(the_loop.is_empty());
/// let l = 0.5;
///
/// the_loop.push(Point3D::new(-l, -l, 0.)).unwrap();
/// the_loop.push(Point3D::new(-l, l, 0.)).unwrap();
/// the_loop.push(Point3D::new(l, l, 0.)).unwrap();
/// the_loop.push(Point3D::new(l, -l, 0.)).unwrap();
///
/// assert!(the_loop.area().is_err());
/// the_loop.close().unwrap();
///
/// let a = the_loop.area().unwrap();
/// assert!((4. * l * l - a).abs() < 0.0001);
/// ```
/// # Note:
/// It has some peculiarities. For instance, it attempts
/// to reduce the number of points on a [`Loop3D`]. This is done by
/// identifying when a colinear [`Point3D`] is to be added and, instead
/// of extending the [`Loop3D`], replacing the last element.
///
/// ```
/// use geometry3d::{Loop3D, Point3D};
/// let mut the_loop = Loop3D::new();
///
/// the_loop.push(Point3D::new(0., 0., 0.)).unwrap();
/// the_loop.push(Point3D::new(1., 1., 0.)).unwrap();
/// assert_eq!(2, the_loop.n_vertices());
///
/// // Adding a collinear point will not extend.
/// let collinear = Point3D::new(2., 2., 0.);
/// the_loop.push(collinear).unwrap();
/// assert_eq!(2, the_loop.n_vertices());
/// assert_eq!(the_loop[1], collinear);
/// ```
///
#[derive(Debug, Clone)]
pub struct Loop3D {
    /// The points of the [`Loop3D`]
    vertices: Vec<Point3D>,

    /// The normal, following a right-hand-side convention
    normal: Vector3D,

    /// A flag indicating whether the [`Loop3D`] is considered finished or not.
    closed: bool,

    /// The area of the [`Loop3D`], only calculated when closing it
    area: Float,

    /// The perimeter of the loop
    perimeter: Float,
}

impl std::iter::IntoIterator for Loop3D {
    type Item = Point3D;
    type IntoIter = std::vec::IntoIter<Self::Item>;
    fn into_iter(self) -> Self::IntoIter {
        self.vertices.into_iter()
    }
}

impl Serialize for Loop3D {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        let mut seq = serializer.serialize_seq(Some(3 * self.vertices.len()))?;
        for Point3D { x, y, z } in self.vertices.iter() {
            seq.serialize_element(x)?;
            seq.serialize_element(y)?;
            seq.serialize_element(z)?;
        }

        seq.end()
    }
}

impl<'de> Deserialize<'de> for Loop3D {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let data: Value = Deserialize::deserialize(deserializer)?;
        let mut ret = Self::new();

        if let Value::Array(a) = data {
            let mut it = a.iter();

            while let Some(x) = it.next() {
                let x = match x {
                    Value::Number(x) => x.as_f64().unwrap() as Float,
                    _ => panic!("Expecting Polygon3D to be an array of numbers"),
                };
                let y = it.next();
                let y = match y {
                    Some(Value::Number(y)) => y.as_f64().unwrap() as Float,
                    _ => panic!("Expecting Polygon3D to be an array of numbers"),
                };
                let z = it.next();
                let z = match z {
                    Some(Value::Number(z)) => z.as_f64().unwrap() as Float,
                    _ => panic!("Expecting Polygon3D to be an array of numbers"),
                };
                ret.push(Point3D { x, y, z }).unwrap();
            }
        }

        ret.close().unwrap();

        Ok(ret)
    }
}

impl Default for Loop3D {
    fn default() -> Self {
        Self::new()
    }
}

impl std::ops::Index<usize> for Loop3D {
    type Output = Point3D;

    fn index(&self, index: usize) -> &Self::Output {
        if index >= self.vertices.len() {
            panic!(
                "Trying to get a vertex out of bounds... {} available, index was {}",
                self.vertices.len(),
                index
            );
        }
        &self.vertices[index]
    }
}

impl Loop3D {
    /// Creates a new and empty [`Loop3D`]. It has a Zero Normal and an
    /// area of -1. These attributes are filled automatically when pushing
    /// vertices into the Loop.
    pub fn new() -> Loop3D {
        Loop3D {
            vertices: Vec::new(),
            normal: Vector3D::new(0., 0., 0.),
            closed: false,
            area: -1.0,
            perimeter: -1.0,
        }
    }

    /// Creates a new and empty [`Loop3D`] with a specific `capacity`. It has a Zero Normal and an
    /// area of -1. These attributes are filled automatically when pushing
    /// vertices into the Loop.
    pub fn with_capacity(capacity: usize) -> Loop3D {
        Loop3D {
            vertices: Vec::with_capacity(capacity),
            normal: Vector3D::new(0., 0., 0.),
            closed: false,
            area: -1.0,
            perimeter: -1.0,
        }
    }

    /// is closed?
    pub fn closed(&self) -> bool {
        self.closed
    }

    /// Creates a clone of `self`, removing the
    /// collinear points.
    ///
    /// The returned [`Loop3D`] will be closed if `self`
    /// is closed and it has more than 3 vertices (it might not
    /// happen, as loops can be modified after closed... not very
    /// safe, but possible)
    ///
    /// # Example
    ///
    /// ```
    /// use geometry3d::{Loop3D, Point3D};
    ///
    /// let mut l = Loop3D::with_capacity(4);
    /// // Add a triangle
    /// l.push(Point3D::new(0., 0., 0.)).unwrap();
    /// l.push(Point3D::new(1., 1., 0.)).unwrap();
    /// l.push(Point3D::new(0., 1., 0.)).unwrap();
    /// l.push(Point3D::new(0., 0.5, 0.)).unwrap();
    ///
    /// l = l.sanitize().unwrap();    
    /// ```
    pub fn sanitize(self) -> Result<Self, String> {
        let mut new = Self::with_capacity(self.len());
        for v in self.vertices.iter() {
            new.push(*v).unwrap();
        }
        if self.closed && new.vertices.len() >= 3 {
            new.close()?
        }
        Ok(new)
    }

    /// Checks if the [`Loop3D`] has Zero vertices
    pub fn is_empty(&self) -> bool {
        self.vertices.is_empty()
    }

    /// Borrows the vertices
    pub fn vertices(&self) -> &[Point3D] {
        &self.vertices
    }

    /// Removes a vertex
    pub fn remove(&mut self, i: usize) {
        self.vertices.remove(i);
    }

    /// Checks whether a [`Point3D`] can be added to a [`Loop3D`] while keeping
    /// it valid
    ///
    /// It checks that:
    /// * That the [`Loop3D`] has not been closed yet
    /// * that the new [`Point3D`] is coplanar witht he rest of the [`Point3D`]    
    /// * Adding the [`Point3D`] will not make the [`Loop3D`] intersect with itself
    fn valid_to_add(&self, point: Point3D) -> Result<(), String> {
        //check if it is closed
        if self.closed {
            return Err("Trying to add a point to a closed Loop3D".to_string());
        }

        // Check if we can already validate coplanarity
        if !self.normal.is_zero() {
            // Normal should be there.
            if !self.is_coplanar(point)? {
                return Err("Trying to add a non-coplanar point to Loop3D".to_string());
            }
        }

        // Check if the new vertex would make the loop intersect with itself
        let n = self.vertices.len();
        if n >= 3 {
            let last_v = self.vertices[n - 1];
            let new_edge = Segment3D::new(last_v, point);
            let mut intersect = Point3D::new(0., 0., 0.); // dummy variable
            for i in 0..n - 2 {
                let v = self.vertices[i];
                let v_p1 = self.vertices[i + 1];
                let this_s = Segment3D::new(v, v_p1);
                // Check the point of intersection.
                if new_edge.intersect(&this_s, &mut intersect) {
                    return Err(
                        "Trying to push a point that would make the Loop3D intersect with itself"
                            .to_string(),
                    );
                }
            }
        }
        Ok(())
    }

    /// Pushes a new [`Point3D`] into the [`Loop3D`].
    ///
    /// If the [`Point3D`] being
    /// pushed is collinear with the previous two, then instead of pushing a new
    /// [`Point3D`] it will update the last one (i.e., because the shape and area)
    /// of the [`Loop3D`] will still be the same.
    ///
    /// Returns an error if the point being added would make the [`Loop3D`]
    /// intersect itself, or if the new [`Point3D`] is not coplanar with the
    /// [`Loop3D`], or if the [`Loop3D`] is closed.
    pub fn push(&mut self, point: Point3D) -> Result<(), String> {
        // Check the point
        self.valid_to_add(point)?;

        let n = self.vertices.len();

        // If there are previous points, Check the points before the new addition
        if n >= 2 {
            let a = self.vertices[n - 2];
            let b = self.vertices[n - 1];

            if a.is_collinear(b, point).unwrap() {
                // if it is collinear, update last point instead of
                // adding a new one
                self.vertices[n - 1] = point;
            } else {
                self.vertices.push(point);
            }
        } else {
            self.vertices.push(point);
        }

        // Calcualte the normal if possible
        if self.vertices.len() == 3 {
            self.set_normal()?;
        }
        Ok(())
    }

    /// Counts the vertices in the [`Loop3D`]
    #[deprecated = "Use len() instead"]
    pub fn n_vertices(&self) -> usize {
        self.vertices.len()
    }

    /// Counts the vertices in the [`Loop3D`]    
    pub fn len(&self) -> usize {
        self.vertices.len()
    }

    /// Checks if a [`Segment3D`] intersects any of the other [`Segment3D`]
    /// in the [`Loop3D`] and if its midpoint is inside of it.
    ///
    /// Note that the idea is to call it by using segments that go from
    /// one vertex to another. I mean, this function takes *any* segment,
    /// meaning that a small [`Segment3D`] "floating" inside of a big [`Loop3D`]
    /// will be considered a diagonal... be careful with this
    pub fn is_diagonal(&self, s: Segment3D) -> Result<bool, String> {
        if s.length < 1e-5 {
            // Very small segment cannot be diagonal
            return Ok(false);
        }
        let mut inter = Point3D::new(0., 0., 0.);
        let n = self.len();
        // It cannot intercect any
        for i in 0..=n {
            let a = self.vertices[i % n];
            let b = self.vertices[(i + 1) % n];

            let poly_s = Segment3D::new(a, b);
            let intersects = s.intersect(&poly_s, &mut inter);
            // If they are contained and are the same length, then they are the same segment

            const TINY: Float = 1e-7;

            let different_length = (s.length() - poly_s.length()).abs() > TINY;
            let contains = s.contains(&poly_s)? && different_length;
            if intersects || contains {
                return Ok(false);
            }
        }
        // And the midpoint must be in the loop.
        if !self.test_point(s.midpoint()).unwrap() {
            return Ok(false);
        }

        Ok(true)
    }

    /// Opens a [`Loop3D`]
    pub fn open(&mut self) {
        self.closed = false
    }

    /// Closes a [`Loop3D`], calculating its area and checking the connection
    /// between the first and last vertex. If the first and the last
    pub fn close(&mut self) -> Result<(), String> {
        // Check if we can try to close now...
        if self.vertices.len() < 3 {
            return Err("Loops need at least 3 vertices".to_string());
        }

        // Check the last vertex for collinearity
        let n = self.vertices.len();
        let a = self.vertices[n - 2];
        let b = self.vertices[n - 1];
        let c = self.vertices[0];

        if a.is_collinear(b, c)? {
            // collinear. Remove the last vertex
            self.vertices.pop();
        }

        // Check if closing would intercept
        self.valid_to_add(self.vertices[0])?;

        // Check the first vertex for collinearity
        let n = self.vertices.len();
        let a = self.vertices[n - 1];
        let b = self.vertices[0];
        let c = self.vertices[1];

        if a.is_collinear(b, c)? {
            // collinear. Remove the last vertex
            self.vertices.remove(0);
        }

        // Close
        self.closed = true;
        self.set_area()?;
        self.set_perimeter()?;
        Ok(())
    }

    /// Sets the normal [`Vector3D`] for a [`Loop3D`]
    fn set_normal(&mut self) -> Result<(), String> {
        if self.vertices.len() < 3 {
            return Err(
                "Trying to set the normal of a Polygon3D with less than three Point3D".to_string(),
            );
        }

        let a = self.vertices[0];
        let b = self.vertices[1];
        let c = self.vertices[2];

        let ab = b - a;
        let bc = c - b;

        self.normal = ab.cross(bc);
        self.normal.normalize();

        Ok(())
    }

    /// Retrieves the normal of the vector.
    ///
    /// # Note
    /// If the [`Loop3D`] has less than 3 vertices, then
    /// the Normal will be `Vector3D(0., 0., 0.)`, which is the default.
    pub fn normal(&self) -> Vector3D {
        self.normal
    }

    /// Checks whether a [`Point3D`] is coplanar with the rest of the
    /// points.
    pub fn is_coplanar(&self, p: Point3D) -> Result<bool, String> {
        // This should not happen, but you never know..
        if self.vertices.is_empty() {
            let msg = "Trying to check whether point is coplanar in a Loop3D without any vertices"
                .to_string();
            return Err(msg);
        }

        if self.normal.is_zero() {
            let msg =
                "Trying to check whether point is coplanar in a Loop3D without normal".to_string();
            return Err(msg);
        }

        let first_point = self.vertices[0];
        let d = first_point - p;

        let aux = (self.normal * d).abs();
        Ok(aux < 1e-7)
    }

    /// Tests whether a [`Point3D`] dwells inside of the [`Loop3D`].
    pub fn test_point(&self, point: Point3D) -> Result<bool, String> {
        // Check if the loop is done
        if !self.closed {
            return Err("Trying to test_point in an open Loop3D".to_string());
        }

        // Check if coplanar
        if !self.is_coplanar(point)? {
            return Ok(false);
        }

        // Ray cast
        let d = (point - (self.vertices[0] + self.vertices[1]) * 0.5) * 1000.; // Should be enough...?
        let ray = Segment3D::new(point, point + d);

        let mut n_cross = 0;
        let n = self.vertices.len();
        for i in 0..n {
            let vertex_a = self.vertices[i];
            let vertex_b = self.vertices[(i + 1) % n];
            let segment_ab = Segment3D::new(vertex_a, vertex_b);

            // Check if the point is in the segment.
            if segment_ab.contains_point(point)? {
                return Ok(true);
            }

            // Check if the ray and the segment touch. We only consider
            // touching at the start (e.g., t_a between [0 and 1) ) in
            // order not to count vertices twice.
            if let Some((t_a, t_b)) = segment_ab.get_intersection_pt(&ray) {
                // If the ray intersects
                if (0. ..=1.).contains(&t_b) && (0. ..=1.).contains(&t_a) {
                    if t_a < Float::EPSILON {
                        // if the intersection is at the start of the segment
                        let side_normal = d.cross(segment_ab.as_vector3d());
                        if side_normal.is_same_direction(self.normal) {
                            n_cross += 1
                        }
                    } else if t_a < 1. {
                        // intersection is within the segment (not including the end)
                        n_cross += 1;
                    } else {
                        // if the intersection is at the end of the segment
                        let side_normal = d.cross(segment_ab.as_reversed_vector3d());
                        if side_normal.is_same_direction(self.normal) {
                            n_cross += 1
                        }
                    }
                }
            }
        }
        // If did not touch OR touched an odd number of
        // times, then it was outside
        Ok(n_cross != 0 && n_cross % 2 != 0)
    } // end of test_point

    /// Calculates and caches the perimeter of the [`Loop3D`]
    fn set_perimeter(&mut self) -> Result<Float, String> {
        if !self.closed {
            let msg =
                "Trying to calculate the perimeter of a Loop3D that is not closed".to_string();
            return Err(msg);
        }

        if self.normal.is_zero() {
            let msg = "Trying to calculate the perimeter of a Loop3D with Zero normal".to_string();
            return Err(msg);
        }

        let n = self.vertices.len();
        if n < 3 {
            let msg =
                "Trying to calculate the perimeter of a Loop3D with less than three valid vertices"
                    .to_string();
            return Err(msg);
        }

        let mut per = 0.0;
        for i in 0..n {
            per += (self.vertices[i % n] - self.vertices[(i + 1) % n]).length();
        }

        self.perimeter = per;
        Ok(self.perimeter)
    }

    /// Calculates and caches the area of the [`Loop3D`]
    fn set_area(&mut self) -> Result<Float, String> {
        if !self.closed {
            let msg = "Trying to calculate the area of a Loop3D that is not closed".to_string();
            return Err(msg);
        }

        if self.normal.is_zero() {
            let msg = "Trying to calculate the area of a Loop3D with Zero normal".to_string();
            return Err(msg);
        }

        // Set the right hand side vector
        let mut rhs = Vector3D::new(0.0, 0.0, 0.0);

        let n = self.vertices.len();
        if n < 3 {
            let msg =
                "Trying to calculate the area of a Loop3D with less than three valid vertices"
                    .to_string();
            return Err(msg);
        }

        // We need to go around from 0 to N vertices,
        // ... so, n+1 valid vertices.
        let mut v: Vector3D = self.vertices[0].into();
        let mut v_p1: Vector3D = self.vertices[1].into();
        for i in 2..n + 2 {
            rhs += v.cross(v_p1);
            v = v_p1;
            v_p1 = self.vertices[i % n].into();
        }

        let area = self.normal * rhs / 2.0;
        if area < 0. {
            self.normal *= -1.;
        }
        self.area = area.abs();
        Ok(self.area)
    }

    /// Returns the area of the [`Loop3D`]
    pub fn area(&self) -> Result<Float, String> {
        if !self.is_closed() {
            Err("Trying to get the area of an open Loop3D".to_string())
        } else {
            Ok(self.area)
        }
    }

    /// Returns the perimeter of the [`Loop3D`]
    pub fn perimeter(&self) -> Result<Float, String> {
        if !self.is_closed() {
            Err("Trying to get the perimeter of an open Loop3D".to_string())
        } else {
            Ok(self.perimeter)
        }
    }

    /// Returns the centroid; i.e., the average of all vertices.
    pub fn centroid(&self) -> Result<Point3D, String> {
        if !self.is_closed() {
            Err("Trying to get the centroid of an open Loop3D".to_string())
        } else {
            let n = self.vertices.len() as Float;
            let (mut x, mut y, mut z) = (0., 0., 0.);
            for v in &self.vertices {
                x += v.x;
                y += v.y;
                z += v.z;
            }

            Ok(Point3D::new(x / n, y / n, z / n))
        }
    }

    /// Indicates whether the [`Loop3D`] has been closed already
    pub fn is_closed(&self) -> bool {
        self.closed
    }

    /// Checks whether the [`Loop3D`] contains a [`Segment3D`] `s`
    pub fn contains_segment(&self, s: &Segment3D) -> bool {
        let n = self.vertices.len();
        for (i, v) in self.vertices.iter().enumerate() {
            let next_v = self.vertices[(i + 1) % n];
            let segment = Segment3D::new(*v, next_v);
            if segment.compare(s) {
                return true;
            }
        }
        false
    }
}

/***********/
/* TESTING */
/***********/

#[cfg(test)]
mod testing {

    use crate::{Polygon3D, Triangulation3D};

    use super::*;

    #[test]
    fn serde_ok() {
        let a = "[
            0.0,0,0,  
            1.0,1,1,  
            2,3,-1
        ]";

        let p: Loop3D = serde_json::from_str(a).unwrap();
        assert!(p.closed);
        assert_eq!(p.vertices.len(), 3);

        assert_eq!(p.vertices[0].x, 0.);
        assert_eq!(p.vertices[0].y, 0.);
        assert_eq!(p.vertices[0].z, 0.);

        assert_eq!(p.vertices[1].x, 1.);
        assert_eq!(p.vertices[1].y, 1.);
        assert_eq!(p.vertices[1].z, 1.);

        assert_eq!(p.vertices[2].x, 2.);
        assert_eq!(p.vertices[2].y, 3.);
        assert_eq!(p.vertices[2].z, -1.);

        println!("{}", serde_json::to_string(&p).unwrap());
    }

    #[test]
    fn test_new() {
        let l = Loop3D::new();
        assert_eq!(l.vertices.len(), 0);

        let l = Loop3D::default();
        assert_eq!(l.vertices.len(), 0);

        let v = Point3D::new(0., 1., 0.);
        let v_cp = v;
        let v_clone = v.clone();
        assert_eq!(v, v_cp);
        assert_eq!(v, v_clone);
    }

    #[test]
    fn test_push() {
        let mut l = Loop3D::new();
        assert_eq!(l.vertices.len(), 0);

        l.push(Point3D::new(1., 2., 3.)).unwrap();

        assert_eq!(l.vertices.len(), 1);
        assert_eq!(l[0], Point3D::new(1., 2., 3.));

        // n_vertices
        assert_eq!(l.len(), 1);

        l.push(Point3D::new(4., 5., 6.)).unwrap();
        assert_eq!(l.len(), 2);
        assert_eq!(l[1], Point3D::new(4., 5., 6.));

        // Collinear point in the middle
        let mut l = Loop3D::new();
        assert_eq!(0, l.vertices.len());
        l.push(Point3D::new(-2., -2., 0.)).unwrap(); // 0
        assert_eq!(1, l.vertices.len());
        l.push(Point3D::new(0., -2., 0.)).unwrap(); // 1 -- collinear point
        assert_eq!(2, l.vertices.len());
        l.push(Point3D::new(2., -2., 0.)).unwrap(); // 2
        assert_eq!(2, l.vertices.len());
        l.push(Point3D::new(2., 2., 0.)).unwrap(); // 3
        assert_eq!(l.len(), 3);
        l.push(Point3D::new(-2., 2., 0.)).unwrap(); // 4
        assert_eq!(l.len(), 4);
        l.close().unwrap();
        assert_eq!(l.len(), 4);
        assert!((l.area - 16.).abs() < 1e-4);

        assert_eq!(l[0], Point3D::new(-2., -2., 0.));
        assert_eq!(l[1], Point3D::new(2., -2., 0.));
        assert_eq!(l[2], Point3D::new(2., 2., 0.));
        assert_eq!(l[3], Point3D::new(-2., 2., 0.));

        // Collinear point in the end
        let mut l = Loop3D::new();
        assert_eq!(0, l.vertices.len());
        l.push(Point3D::new(-2., -2., 0.)).unwrap(); // 0
        assert_eq!(1, l.vertices.len());
        l.push(Point3D::new(2., -2., 0.)).unwrap(); // 1
        assert_eq!(2, l.vertices.len());
        l.push(Point3D::new(2., 2., 0.)).unwrap(); // 2
        assert_eq!(l.len(), 3);
        l.push(Point3D::new(-2., 2., 0.)).unwrap(); // 3
        assert_eq!(l.len(), 4);
        l.push(Point3D::new(-2., 0., 0.)).unwrap(); // 4 -- collinear point... will be removed when closing
        assert_eq!(5, l.vertices.len());
        l.close().unwrap();
        assert_eq!(l.len(), 4);
        assert!((l.area - 16.).abs() < 1e-4);
        assert_eq!(l[0], Point3D::new(-2., -2., 0.));
        assert_eq!(l[1], Point3D::new(2., -2., 0.));
        assert_eq!(l[2], Point3D::new(2., 2., 0.));
        assert_eq!(l[3], Point3D::new(-2., 2., 0.));

        // Collinear point in the beginning
        let mut l = Loop3D::new();
        assert_eq!(0, l.vertices.len());
        l.push(Point3D::new(0., -2., 0.)).unwrap(); // 0  -- collinear point... will be removed when closing
        assert_eq!(1, l.vertices.len());
        l.push(Point3D::new(2., -2., 0.)).unwrap(); // 1
        assert_eq!(2, l.vertices.len());
        l.push(Point3D::new(2., 2., 0.)).unwrap(); // 2
        assert_eq!(l.len(), 3);
        l.push(Point3D::new(-2., 2., 0.)).unwrap(); // 3
        assert_eq!(l.len(), 4);
        l.push(Point3D::new(-2., -2., 0.)).unwrap(); // 4
        assert_eq!(5, l.vertices.len());
        l.close().unwrap();
        assert_eq!(l.len(), 4);
        assert!((l.area - 16.).abs() < 1e-4);
        assert_eq!(l[0], Point3D::new(2., -2., 0.));
        assert_eq!(l[1], Point3D::new(2., 2., 0.));
        assert_eq!(l[2], Point3D::new(-2., 2., 0.));
        assert_eq!(l[3], Point3D::new(-2., -2., 0.));

        // INTERSECT WITH ITSELF.

        let mut l = Loop3D::new();
        assert!(l.push(Point3D::new(-2., -2., 0.)).is_ok()); // 0 collinear
        assert!(l.push(Point3D::new(2., 2., 0.)).is_ok()); // 1
        assert!(l.push(Point3D::new(-2., 2., 0.)).is_ok()); // 2

        // This should fail.
        assert!(l.push(Point3D::new(2., -2., 0.)).is_err()); // 3
    } // end of test_push

    #[test]
    fn test_is_coplanar() {
        let mut l = Loop3D::new();
        l.push(Point3D::new(-2., -2., 0.)).unwrap(); // 0
        l.push(Point3D::new(2., -2., 0.)).unwrap(); // 1
        l.push(Point3D::new(2., 2., 0.)).unwrap(); // 2
        l.push(Point3D::new(-2., 2., 0.)).unwrap(); // 3
        l.close().unwrap();

        let r = l.is_coplanar(Point3D::new(0., 0., 0.));
        match r {
            Ok(b) => {
                assert!(b)
            }
            Err(e) => panic!("{}", e),
        }

        let r = l.is_coplanar(Point3D::new(0., 0., 10.1));
        match r {
            Ok(b) => {
                assert!(!b)
            }
            Err(e) => panic!("{}", e),
        }
    }

    #[test]
    fn test_point_weird() {
        let mut the_loop = Loop3D::new();
        the_loop
            .push(Point3D::new(10., -1.2246467991473533E-15, 0.))
            .unwrap();
        the_loop
            .push(Point3D::new(-10., 1.2246467991473533E-15, 0.))
            .unwrap();
        the_loop
            .push(Point3D::new(-10., 1.2246467991473533E-15, 3.))
            .unwrap();
        the_loop
            .push(Point3D::new(10., -1.2246467991473533E-15, 3.))
            .unwrap();
        the_loop.close().unwrap();

        let a = Point3D::new(-10., 1.2246467991473533E-15, 0.);
        let b = Point3D::new(10., -1.2246467991473533E-15, 3.);
        let mid = (a + b) / 2.;
        let r = the_loop.test_point(mid).unwrap();
        assert!(r);
    }

    #[test]
    fn test_point_convex_loop_interior() {
        //let normal = Vector3D::new(0., 0., 1.);
        let mut the_loop = Loop3D::new();
        let l = 0.5;
        the_loop.push(Point3D::new(-l, -l, 0.)).unwrap();
        the_loop.push(Point3D::new(l, -l, 0.)).unwrap();
        the_loop.push(Point3D::new(l, l, 0.)).unwrap();
        the_loop.push(Point3D::new(-l, l, 0.)).unwrap();
        the_loop.close().unwrap();

        let r = the_loop.test_point(Point3D::new(0., 0., 0.)).unwrap();
        assert!(r);
    }

    #[test]
    fn test_point_convex_loop_exterior() {
        //Vector3D normal = Vector3D(0, 0, 1);
        let mut the_loop = Loop3D::new();
        let l = 1. / (2 as Float).sqrt();
        the_loop.push(Point3D::new(-l, -l, 0.)).unwrap();
        the_loop.push(Point3D::new(l, -l, 0.)).unwrap();
        the_loop.push(Point3D::new(l, l, 0.)).unwrap();
        the_loop.push(Point3D::new(-l, l, 0.)).unwrap();
        the_loop.close().unwrap();

        let r = the_loop.test_point(Point3D::new(-10., 0., 0.)).unwrap();
        assert!(!r);
    }

    #[test]
    fn test_point_concave_loop_exterior1() {
        //Vector3D normal = Vector3D(0, 0, 1);
        let mut the_loop = Loop3D::new();

        let l = 1.0 / (2 as Float).sqrt();
        let bigl = 2. / (2 as Float).sqrt();

        the_loop.push(Point3D::new(-bigl, -bigl, 0.)).unwrap();
        the_loop.push(Point3D::new(0.0, -bigl, 0.)).unwrap(); // collinear point, moving in X
        the_loop.push(Point3D::new(bigl, -bigl, 0.)).unwrap();
        the_loop.push(Point3D::new(bigl, 0., 0.)).unwrap(); // collinear point, moving in Y
        the_loop.push(Point3D::new(bigl, bigl, 0.)).unwrap();
        the_loop.push(Point3D::new(-bigl, bigl, 0.)).unwrap();
        the_loop.push(Point3D::new(-bigl, -bigl, 0.)).unwrap();

        the_loop.push(Point3D::new(-l, -l, 0.)).unwrap();
        the_loop.push(Point3D::new(-l, l, 0.)).unwrap();
        the_loop.push(Point3D::new(l, l, 0.)).unwrap();
        the_loop.push(Point3D::new(l, -l, 0.)).unwrap();
        the_loop.push(Point3D::new(-l, -l, 0.)).unwrap();

        the_loop.close().unwrap();

        let r = the_loop.test_point(Point3D::new(0., 0., 0.)).unwrap();
        assert!(!r);
    }

    #[test]
    fn test_point_concave_loop_exterior2() {
        //Vector3D normal = Vector3D(0, 0, 1);
        let mut the_loop = Loop3D::new();

        let l = 1. / (2 as Float).sqrt();
        let bigl = 2. / (2 as Float).sqrt();

        the_loop.push(Point3D::new(-bigl, -bigl, 0.)).unwrap();
        the_loop.push(Point3D::new(0.0, -bigl, 0.)).unwrap(); // collinear point, moving in X
        the_loop.push(Point3D::new(bigl, -bigl, 0.)).unwrap();
        the_loop.push(Point3D::new(bigl, 0., 0.)).unwrap(); // collinear point, moving in Y
        the_loop.push(Point3D::new(bigl, bigl, 0.)).unwrap();
        the_loop.push(Point3D::new(-bigl, bigl, 0.)).unwrap();
        the_loop.push(Point3D::new(-bigl, -bigl, 0.)).unwrap();

        the_loop.push(Point3D::new(-l, -l, 0.)).unwrap();
        the_loop.push(Point3D::new(-l, l, 0.)).unwrap();
        the_loop.push(Point3D::new(l, l, 0.)).unwrap();
        the_loop.push(Point3D::new(l, -l, 0.)).unwrap();
        the_loop.push(Point3D::new(-l, -l, 0.)).unwrap();

        the_loop.close().unwrap();

        let r = the_loop.test_point(Point3D::new(-10., 0., 0.)).unwrap();
        assert!(!r);
    }

    #[test]
    fn test_point_concave_loop_interior() {
        //Vector3D normal = Vector3D(0, 0, 1);
        let mut the_loop = Loop3D::new();
        let l = 0.5;
        let bigl = 1.;

        the_loop.push(Point3D::new(-bigl, -bigl, 0.)).unwrap();
        the_loop.push(Point3D::new(0., -bigl, 0.)).unwrap(); // collinear point, moving in X
        the_loop.push(Point3D::new(bigl, -bigl, 0.)).unwrap();
        the_loop.push(Point3D::new(bigl, 0., 0.)).unwrap(); // collinear point, moving in Y
        the_loop.push(Point3D::new(bigl, bigl, 0.)).unwrap();
        the_loop.push(Point3D::new(-bigl, bigl, 0.)).unwrap();
        the_loop.push(Point3D::new(-bigl, -bigl, 0.)).unwrap();

        the_loop.push(Point3D::new(-l, -l, 0.)).unwrap();
        the_loop.push(Point3D::new(-l, l, 0.)).unwrap();
        the_loop.push(Point3D::new(l, l, 0.)).unwrap();
        the_loop.push(Point3D::new(l, -l, 0.)).unwrap();
        the_loop.push(Point3D::new(-l, -l, 0.)).unwrap();

        the_loop.close().unwrap();

        let r = the_loop
            .test_point(Point3D::new(-(bigl + l) / 2., -(bigl + l) / 2., 0.))
            .unwrap();
        assert!(r);
    }

    #[test]
    fn test_point_through_vertex() {
        //Vector3D normal = Vector3D(0, 0, 1);
        let mut the_loop = Loop3D::new();
        let l = 1.;
        let bigl = 3.;

        the_loop.push(Point3D::new(0., 0., 0.)).unwrap();
        the_loop.push(Point3D::new(bigl, 0., 0.)).unwrap();
        the_loop.push(Point3D::new(bigl, bigl, 0.)).unwrap();
        the_loop.push(Point3D::new(0., bigl, 0.)).unwrap();
        the_loop.push(Point3D::new(0., 0., 0.)).unwrap();

        the_loop.push(Point3D::new(l, l, 0.)).unwrap();
        the_loop.push(Point3D::new(l, 2. * l, 0.)).unwrap();
        the_loop.push(Point3D::new(2. * l, 2. * l, 0.)).unwrap();
        the_loop.push(Point3D::new(2. * l, l, 0.)).unwrap();
        the_loop.push(Point3D::new(l, l, 0.)).unwrap();

        the_loop.close().unwrap();

        let r = the_loop.test_point(Point3D::new(l / 2., l, 0.)).unwrap();
        assert!(r);
    }

    #[test]
    fn test_point_concave_loop_interior_with_clean() {
        //Vector3D normal = Vector3D(0, 0, 1);

        let mut the_loop = Loop3D::new();
        let l = 1. / (2 as Float).sqrt();
        let bigl = 2. / (2 as Float).sqrt();

        the_loop.push(Point3D::new(-bigl, -bigl, 0.)).unwrap();
        the_loop.push(Point3D::new(0., -bigl, 0.)).unwrap(); // collinear point, moving in X
        the_loop.push(Point3D::new(bigl, -bigl, 0.)).unwrap();
        the_loop.push(Point3D::new(bigl, 0., 0.)).unwrap(); // collinear point, moving in Y
        the_loop.push(Point3D::new(bigl, bigl, 0.)).unwrap();
        the_loop.push(Point3D::new(-bigl, bigl, 0.)).unwrap();
        the_loop.push(Point3D::new(-bigl, -bigl, 0.)).unwrap();

        the_loop.push(Point3D::new(-l, -l, 0.)).unwrap();
        the_loop.push(Point3D::new(-l, l, 0.)).unwrap();
        the_loop.push(Point3D::new(l, l, 0.)).unwrap();
        the_loop.push(Point3D::new(l, -l, 0.)).unwrap();
        the_loop.push(Point3D::new(-l, -l, 0.)).unwrap();

        the_loop.close().unwrap();

        let r = the_loop
            .test_point(Point3D::new(
                -1.5 / (2 as Float).sqrt(),
                -1.5 / (2 as Float).sqrt(),
                0.,
            ))
            .unwrap();
        assert!(r);
    }

    #[test]
    fn test_point_non_coplanar() {
        let mut the_loop = Loop3D::new();
        let l = 1. / (2 as Float).sqrt();

        the_loop.push(Point3D::new(-l, -l, 0.)).unwrap();
        the_loop.push(Point3D::new(-l, l, 0.)).unwrap();
        the_loop.push(Point3D::new(l, l, 0.)).unwrap();
        the_loop.push(Point3D::new(l, -l, 0.)).unwrap();
        the_loop.close().unwrap();

        let r = the_loop
            .test_point(Point3D::new(
                -1.5 / (2 as Float).sqrt(),
                -1.5 / (2 as Float).sqrt(),
                1.,
            ))
            .unwrap();
        assert!(!r);
    }

    #[test]
    fn test_area_1() {
        //Vector3D normal = Vector3D(0, 0, 1);
        let mut the_loop = Loop3D::new();
        let l = 0.5;

        the_loop.push(Point3D::new(-l, -l, 0.)).unwrap();
        the_loop.push(Point3D::new(-l, l, 0.)).unwrap();
        the_loop.push(Point3D::new(l, l, 0.)).unwrap();
        the_loop.push(Point3D::new(l, -l, 0.)).unwrap();

        assert!(the_loop.area().is_err());
        the_loop.close().unwrap();

        let a = the_loop.area().unwrap();
        assert!((4. * l * l - a).abs() < 0.0001);
    }

    #[test]
    fn test_area_2() {
        let l = 1.;

        // Counterclock wise
        let mut the_loop = Loop3D::new();
        the_loop.push(Point3D::new(l, 0., 0.)).unwrap(); //1
        the_loop.push(Point3D::new(l, l, 0.)).unwrap(); //2
        the_loop.push(Point3D::new(2.0 * l, l, 0.)).unwrap(); //3
        the_loop.push(Point3D::new(2.0 * l, 2.0 * l, 0.)).unwrap(); //4
        the_loop.push(Point3D::new(0.0, 2.0 * l, 0.)).unwrap(); //5
        the_loop.push(Point3D::new(0., 0., 0.)).unwrap(); //0

        assert!(the_loop.area().is_err());

        the_loop.close().unwrap();

        let a = the_loop.area().unwrap();
        assert!((3.0 - a).abs() < 1e-4);
        assert!(
            the_loop.normal().compare(Vector3D::new(0., 0., 1.)),
            "normal = {}",
            the_loop.normal()
        );

        // Clockwise
        let l = 1.;

        // Counterclock wise
        let mut the_loop = Loop3D::new();
        the_loop.push(Point3D::new(l, 0., 0.)).unwrap(); //1
        the_loop.push(Point3D::new(0., 0., 0.)).unwrap(); //0
        the_loop.push(Point3D::new(0.0, 2.0 * l, 0.)).unwrap(); //5
        the_loop.push(Point3D::new(2.0 * l, 2.0 * l, 0.)).unwrap(); //4
        the_loop.push(Point3D::new(2.0 * l, l, 0.)).unwrap(); //3
        the_loop.push(Point3D::new(l, l, 0.)).unwrap(); //2

        the_loop.close().unwrap();

        let a = the_loop.area().unwrap();
        assert!((3.0 - a).abs() < 1e-4);
        assert!(
            the_loop.normal().compare(Vector3D::new(0., 0., -1.)),
            "normal = {}",
            the_loop.normal()
        );
    }

    #[test]
    fn test_close() {
        let l = 1.;

        // Two elements... cannot close
        let mut the_loop = Loop3D::new();
        the_loop.push(Point3D::new(0., 0., 0.)).unwrap();
        the_loop.push(Point3D::new(l, 0., 0.)).unwrap();
        assert!(the_loop.close().is_err());

        // Three elements... can close
        let mut the_loop = Loop3D::new();
        the_loop.push(Point3D::new(0., 0., 0.)).unwrap();
        the_loop.push(Point3D::new(l, 0., 0.)).unwrap();
        the_loop.push(Point3D::new(0., l, 0.)).unwrap();
        assert!(the_loop.close().is_ok());

        // Three elements, but in the same line... cannot close
        let mut the_loop = Loop3D::new();
        the_loop.push(Point3D::new(0., 0., 0.)).unwrap();
        the_loop.push(Point3D::new(l, 0., 0.)).unwrap();
        the_loop.push(Point3D::new(2.0 * l, 0., 0.)).unwrap();
        assert!(!the_loop.close().is_ok());

        // four elements, two in the same line... can close
        let mut the_loop = Loop3D::new();
        the_loop.push(Point3D::new(0., 0., 0.)).unwrap();
        the_loop.push(Point3D::new(l, 0., 0.)).unwrap();
        the_loop.push(Point3D::new(2.0 * l, 0., 0.)).unwrap();
        the_loop.push(Point3D::new(0., l, 0.)).unwrap();
        assert!(the_loop.close().is_ok());
    }

    #[test]
    fn test_is_diagonal() {
        let mut l = Loop3D::new();
        l.push(Point3D::new(-2., -2., 0.)).unwrap(); // 0
        l.push(Point3D::new(2., -2., 0.)).unwrap(); // 1
        l.push(Point3D::new(2., 2., 0.)).unwrap(); // 2
        l.push(Point3D::new(-2., 2., 0.)).unwrap(); // 3
        l.close().unwrap();

        // Intersects a segment
        assert!(!l
            .is_diagonal(Segment3D::new(
                Point3D::new(-4., 0., 0.),
                Point3D::new(4., 0., 0.),
            ))
            .unwrap());

        // Doesn't intersect, but is outside
        assert!(!l
            .is_diagonal(Segment3D::new(
                Point3D::new(4., 0., 0.),
                Point3D::new(5., 0., 0.),
            ))
            .unwrap());

        // Doesn't intersect, is inside == is_diagonal
        assert!(l
            .is_diagonal(Segment3D::new(
                Point3D::new(-2., -2., 0.),
                Point3D::new(2., 2., 0.),
            ))
            .unwrap());
    }

    #[test]
    fn test_contains_segment() {
        let mut l = Loop3D::new();
        let p0 = Point3D::new(-2., -2., 0.);
        let p1 = Point3D::new(2., -2., 0.);
        let p2 = Point3D::new(2., 2., 0.);
        let p3 = Point3D::new(-2., 2., 0.);
        l.push(p0).unwrap(); // 0
        l.push(p1).unwrap(); // 1
        l.push(p2).unwrap(); // 2
        l.push(p3).unwrap(); // 3

        // Existing segments, in both directions
        assert!(l.contains_segment(&Segment3D::new(p0, p1)));
        assert!(l.contains_segment(&Segment3D::new(p1, p2)));
        assert!(l.contains_segment(&Segment3D::new(p2, p3)));
        assert!(l.contains_segment(&Segment3D::new(p3, p0)));
        assert!(l.contains_segment(&Segment3D::new(p3, p2)));
        assert!(l.contains_segment(&Segment3D::new(p2, p1)));
        assert!(l.contains_segment(&Segment3D::new(p1, p0)));
        assert!(l.contains_segment(&Segment3D::new(p0, p3)));

        // Diagonals
        assert!(!l.contains_segment(&Segment3D::new(p1, p3)));
        assert!(!l.contains_segment(&Segment3D::new(p3, p1)));
        assert!(!l.contains_segment(&Segment3D::new(p0, p2)));
        assert!(!l.contains_segment(&Segment3D::new(p2, p0)));

        // Segment inside
        assert!(!l.contains_segment(&Segment3D::new(
            Point3D::new(-0.5, -0.5, 0.),
            Point3D::new(0.5, 0.5, 0.),
        )));

        // Segment that crosses from in to out
        assert!(!l.contains_segment(&Segment3D::new(
            Point3D::new(-0.5, -0.5, 0.),
            Point3D::new(10.5, 10.5, 0.),
        )));

        // Segment contained in another segment
        assert!(!l.contains_segment(&Segment3D::new(
            Point3D::new(-1., -2., 0.),
            Point3D::new(1., -2., 0.),
        )));
    }

    #[test]
    fn test_valid_to_add() {
        let mut outer = Loop3D::new();

        let p0 = Point3D::new(0., 0., 0.);
        assert!(outer.valid_to_add(p0).is_ok());
        outer.push(p0).unwrap();

        let p1 = Point3D::new(0., 3., 0.);
        assert!(outer.valid_to_add(p1).is_ok());
        outer.push(p1).unwrap();

        let p2 = Point3D::new(3., 3., 0.);
        assert!(outer.valid_to_add(p2).is_ok());
        outer.push(p2).unwrap();

        let p3 = Point3D::new(5., 5., 0.);
        assert!(outer.valid_to_add(p3).is_ok());
        outer.push(p3).unwrap();

        let p4 = Point3D::new(3., 6., 0.);
        assert!(outer.valid_to_add(p4).is_ok());
        outer.push(p4).unwrap();

        let p5 = Point3D::new(0., 5., 0.);
        assert!(outer.valid_to_add(p5).is_ok());
    }

    #[test]
    fn test_perimeter_centroid() {
        // A square with the center at the origin.
        /*****/
        let mut outer_loop = Loop3D::new();
        let l = 2. as Float;
        outer_loop.push(Point3D::new(-l, -l, 0.)).unwrap();
        outer_loop.push(Point3D::new(l, -l, 0.)).unwrap();
        outer_loop.push(Point3D::new(l, l, 0.)).unwrap();
        outer_loop.push(Point3D::new(-l, l, 0.)).unwrap();
        outer_loop.close().unwrap();

        assert_eq!(outer_loop.perimeter, 8. * l);
        assert_eq!(outer_loop.perimeter().unwrap(), 8. * l);

        let c = outer_loop.centroid().unwrap();
        assert!(c.x.abs() < 1e-8);
        assert!(c.y.abs() < 1e-8);
        assert!(c.z.abs() < 1e-8);
    }

    #[test]
    fn test_sanitize() {
        let mut l = Loop3D::with_capacity(4);
        // Add a triangle
        l.vertices.push(Point3D::new(0., 0., 0.));
        l.vertices.push(Point3D::new(1., 1., 0.));
        l.vertices.push(Point3D::new(0., 1., 0.));
        // And then two aligned points
        l.vertices.push(Point3D::new(0., 0.5, 0.)); // <- Collinear point
        l.vertices.push(Point3D::new(0., 0.2, 0.));

        // start with 5 points
        assert_eq!(l.len(), 5);

        // Sanitizing removes the collinear point,
        // withouth closing.
        l = l.sanitize().unwrap();
        assert_eq!(l.len(), 4);
        assert!(!l.closed());

        // We close it, (last point we
        // added is collinear, so it will be removed).
        l.close().unwrap();
        assert_eq!(l.len(), 3);

        // Sanitizing now should return a closed polygon.
        l = l.sanitize().unwrap();
        assert!(l.closed());
    }

    #[test]
    fn test_weird_loop() {
        let mut l = Loop3D::new();
        l.push(Point3D {
            x: 0.,
            y: 1.3500000000000001,
            z: 0.0,
        })
        .unwrap();
        l.push(Point3D {
            x: 0.,
            y: 2.0899999999999999,
            z: 0.82999999999999996,
        })
        .unwrap();
        l.push(Point3D {
            x: 0.,
            y: 2.9900000000000002,
            z: 0.82999999999999996,
        })
        .unwrap();
        l.push(Point3D {
            x: 0.,
            y: 2.9900000000000002,
            z: 2.29,
        })
        .unwrap();
        l.push(Point3D {
            x: 0.,
            y: 2.0899999999999999,
            z: 2.29,
        })
        .unwrap();
        l.push(Point3D {
            x: 0.,
            y: 1.3500000000000001,
            z: 2.7000000000000002,
        })
        .unwrap();
        l.push(Point3D {
            x: 0.,
            y: 3.7400000000000002,
            z: 2.7000000000000002,
        })
        .unwrap();
        l.push(Point3D {
            x: 0.,
            y: 3.7400000000000002,
            z: 0.0,
        })
        .unwrap();

        l.close().unwrap();

        let poly = Polygon3D::new(l).unwrap();
        Triangulation3D::from_polygon(&poly).unwrap();
    }

    #[test]
    fn test_weird_loop_2() {
        let mut outer = Loop3D::new();
        outer
            .push(Point3D {
                x: 8.7699999999999996,
                y: 3.7400000000000002,
                z: 0.0,
            })
            .unwrap(); // 0
        outer
            .push(Point3D {
                x: 8.7699999999999996,
                y: 3.7400000000000002,
                z: 2.7000000000000002,
            })
            .unwrap(); // 1
        outer
            .push(Point3D {
                x: 8.7699999999999996,
                y: 0.0,
                z: 2.7000000000000002,
            })
            .unwrap(); // 1
        outer
            .push(Point3D {
                x: 8.7699999999999996,
                y: 0.0,
                z: 0.0,
            })
            .unwrap(); // 1
        outer.close().unwrap();

        let mut inner = Loop3D::new();

        inner
            .push(Point3D {
                x: 8.7699999999999996,
                y: 1.2649999999999999,
                z: 0.69999999999999996,
            })
            .unwrap(); // 2
        inner
            .push(Point3D {
                x: 8.7699999999999996,
                y: 1.2649999999999999,
                z: 2.29,
            })
            .unwrap(); // 3
        inner
            .push(Point3D {
                x: 8.7699999999999996,
                y: 2.4750000000000001,
                z: 2.29,
            })
            .unwrap(); // 4
        inner
            .push(Point3D {
                x: 8.7699999999999996,
                y: 2.4740000000000002,
                z: 0.69999999999999996,
            })
            .unwrap(); // 5

        inner.close().unwrap();

        let mut poly = Polygon3D::new(outer).unwrap();
        poly.cut_hole(inner).unwrap();
        Triangulation3D::from_polygon(&poly).unwrap();
    }

    #[test]
    fn test_weird_loop_3() {
        let mut outer = Loop3D::new();
        outer
            .push(Point3D {
                x: 2.98,
                y: 1.3500000000000001,
                z: 2.7000000000000002,
            })
            .unwrap(); // 0
        outer
            .push(Point3D {
                x: 4.0199999999999996,
                y: 1.3500000000000001,
                z: 2.7000000000000002,
            })
            .unwrap(); // 1
        outer
            .push(Point3D {
                x: 4.0199999999999996,
                y: 6.0300000000000002,
                z: 2.7000000000000002,
            })
            .unwrap(); // 1
        outer
            .push(Point3D {
                x: 2.98,
                y: 6.0300000000000002,
                z: 2.7000000000000002,
            })
            .unwrap(); // 1

        outer
            .push(Point3D {
                x: 2.98,
                y: 5.7599999999999998,
                z: 2.7000000000000002,
            })
            .unwrap(); // 2
        outer
            .push(Point3D {
                x: 1.9199999999999999,
                y: 5.7599999999999998,
                z: 2.7000000000000002,
            })
            .unwrap(); // 3
        outer
            .push(Point3D {
                x: 1.9199999999999999,
                y: 4.7999999999999998,
                z: 2.7000000000000002,
            })
            .unwrap(); // 4
        outer
            .push(Point3D {
                x: 2.98,
                y: 4.7999999999999998,
                z: 2.7000000000000002,
            })
            .unwrap(); // 5

        outer.close().unwrap();

        let poly = Polygon3D::new(outer).unwrap();
        Triangulation3D::from_polygon(&poly).unwrap();
    }
}
