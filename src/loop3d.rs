use crate::point3d::*;
use crate::segment3d::*;
use crate::vector3d::*;


#[derive(Clone)]
pub struct Loop3D {
    vertices: Vec<Point3D>,
    normal: Vector3D,
    closed: bool,
    area: f64,
}

impl Default for Loop3D {
    fn default() -> Self {
        Self::new()
    }
}

impl std::ops::Index<usize> for Loop3D {
    type Output = Point3D;

    fn index(&self, index: usize) -> &Self::Output {
        if index >= self.vertices.len(){
            panic!("Trying to get a vertex out of bounds... {} available, index was {}", self.vertices.len(), index);
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
        }
    }

    /// Borrows the vertices
    pub fn vertices(&self)->&[Point3D]{
        &self.vertices
    }

    /// Removes a vertex
    pub fn remove(&mut self, i: usize){
        self.vertices.remove(i);
    }

    /// Checks whether a [`Point3D`] can be added to a [`Loop3D`] while keeping 
    /// it valid
    /// 
    /// It checks that:
    /// * That the [`Loop3D`] has not been closed yet
    /// * that the new [`Point3D`] is coplanar witht he rest of the [`Point3D`]    
    /// * Adding the [`Point3D`] will not make the [`Loop3D`] intersect with itself
    fn valid_to_add(&self, point:Point3D)->Result<(),String>{
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
    /// of the [`Polygon3D`] will still be the same. If you want to keep both
    /// points, use the method `force_push`.
    /// 
    /// Returns an error if the point being added would make the [`Polygon3D`] 
    /// intersect itself, or if the new [`Point3D`] is not coplanar with the 
    /// [`Polygon3D`], or if the [`Polygon3D`] is closed.
    pub fn push(&mut self, point: Point3D) -> Result<(), String> {
        
        // Check the point
        self.valid_to_add(point)?;

        let n = self.vertices.len();
        
        // If there are previous points, Check the points before the new addition
        if n >= 2 {
            let a = self.vertices[n - 2];
            let b = self.vertices[n-1];

            if a.is_collinear(b, point).unwrap() {
                // if it is collinear, update last point instead of 
                // adding a new one
                self.vertices[n-1]=point;
            }else{
                self.vertices.push(point);
            }    
        }else{
            self.vertices.push(point);
        }

        // Calcualte the normal if possible
        if self.vertices.len() == 3 {
            self.set_normal()?;
        }
        Ok(())
    }

    /// Counts the vertices in the [`Polygon3D`]
    pub fn n_vertices(&self) -> usize {
        self.vertices.len()
    }

    /// Checks if a [`Segment3D`] intersects any of the other [`Segment3D`]
    /// in the [`Loop3D`] and if its midpoint is inside of it.
    ///
    /// Note that the idea is to call it by using segments that go from
    /// one vertex to another. I mean, this function takes *any* segment,
    /// meaning that a small [`Segment3D`] "floating" inside of a big [`Loop3D`]
    /// will be considered a diagonal... be careful with this
    pub fn is_diagonal(&self, s: Segment3D) -> bool {
        let mut inter = Point3D::new(0., 0., 0.);
        let n = self.n_vertices();
        // It cannot intercept any
        for i in 0..n + 1 {
            let a = self.vertices[i % n];
            let b = self.vertices[(i + 1) % n];

            let poly_s = Segment3D::new(a, b);
            if s.intersect(&poly_s, &mut inter) {
                return false;
            }
        }
        // And the midpoint must be in the loop.
        if !self.test_point(s.midpoint()).unwrap() {
            return false;
        }

        true // return
    }
    

    /// Closes a [`Loop3D`], calculating its area and checking the connection
    /// between the first and last vertex. If the first and the last 
    pub fn close(&mut self) -> Result<(), String> {
        // Check if we can try to close now...
        if self.vertices.len() < 3 {
            return Err("Trying to close a Loop3D with less than 3 vertices".to_string());
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
        Ok(())
    }


    /// Sets the normal [`Vector3D`] for a [`Polygon3D`]
    fn set_normal(&mut self) -> Result<(),String>{

        if self.vertices.len() < 3 {
            return Err("Trying to set the normal of a Polygon3D with less than three Point3D".to_string())
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
    /// If the [`Polygon3D`] has less than 3 vertices, then
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
        Ok(aux < 10. * f64::EPSILON)
    }

    /// Tests whether a [`Point3D`] dwells inside of the [`Polygon3D`].
    pub fn test_point(&self, point: Point3D) -> Result<bool, String> {
        // Check if the loop is done
        if !self.closed {
            return Err("Trying to test_point in an open Loop3D".to_string());
        }

        // Check if coplanar
        if !self.is_coplanar(point)? {
            return Ok(false);
        }

        // Ok, it is coplanar...

        // Ray cast
        let d = (point - self.vertices[0]) * 1000000.; // Should be enough...?
        let ray = Segment3D::new(point, point + d);

        let mut dummy = Point3D::new(0., 0., 0.);
        let mut n_cross = 0;        
        let n = self.vertices.len();
        for i in 0..n {
            let a = self.vertices[i];
            let b = self.vertices[(i+1)%n];            
            let s = Segment3D::new(a, b);

            // Check if the point is in the segment.
            if s.contains_point(point)? {
                return Ok(true);
            }

            // Check if the ray and the segment touch. We only consuder 
            // touching at the start (e.g., t_a between [0 and 1) ) in 
            // order not to count vertices twice.
            if let Some((t_a, t_b)) = s.get_intersection_pt(&ray) {                                 
                if (0. ..1.).contains(&t_a) && (0. ..=1.).contains(&t_b) {
                    n_cross += 1;
                }                 
            }
            
        }
        // If did not touch OR touched an odd number of
        // times, then it was outside
        Ok(n_cross != 0 && n_cross % 2 != 0)
    } // end of test_point

    /// Calculates and caches the area of the [`Loop3D`]
    fn set_area(&mut self) -> Result<f64, String> {
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
        let mut i = 0; // inded in real vertices
        let mut v = self.vertices[0].as_vector3d();
        let mut v_p1 = self.vertices[1].as_vector3d();
        for i in 2..n + 2 {
            rhs += v.cross(v_p1);
            v = v_p1;
            v_p1 = self.vertices[i%n].as_vector3d();
        }

        let area = self.normal * rhs / 2.0;
        if area < 0. {
            self.normal *= -1.;
        }
        self.area = area.abs();
        Ok(self.area)
    }

    pub fn area(&self) -> f64 {
        self.area
    }

    pub fn is_closed(&self) -> bool {
        self.closed
    }
}

/***********/
/* TESTING */
/***********/

#[cfg(test)]
mod testing {
    use super::*;

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
        assert_eq!(l.n_vertices(), 1);

        l.push(Point3D::new(4., 5., 6.)).unwrap();
        assert_eq!(l.n_vertices(), 2);
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
        assert_eq!(l.n_vertices(), 3);
        l.push(Point3D::new(-2., 2., 0.)).unwrap(); // 4
        assert_eq!(l.n_vertices(), 4);                
        l.close().unwrap();
        assert_eq!(l.n_vertices(), 4);   
        assert_eq!(l.area, 16.);

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
        assert_eq!(l.n_vertices(), 3);
        l.push(Point3D::new(-2., 2., 0.)).unwrap(); // 3
        assert_eq!(l.n_vertices(), 4);                
        l.push(Point3D::new(-2., 0., 0.)).unwrap(); // 4 -- collinear point... will be removed when closing
        assert_eq!(5, l.vertices.len());
        l.close().unwrap();
        assert_eq!(l.n_vertices(), 4);
        assert_eq!(l.area, 16.);
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
        assert_eq!(l.n_vertices(), 3);
        l.push(Point3D::new(-2., 2., 0.)).unwrap(); // 3
        assert_eq!(l.n_vertices(), 4);                
        l.push(Point3D::new(-2., -2., 0.)).unwrap(); // 4 
        assert_eq!(5, l.vertices.len());
        l.close().unwrap();
        assert_eq!(l.n_vertices(), 4);
        assert_eq!(l.area, 16.);
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
        let l = 1. / (2 as f64).sqrt();
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

        let l = 1.0 / (2 as f64).sqrt();
        let bigl = 2. / (2 as f64).sqrt();

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

        let l = 1. / (2 as f64).sqrt();
        let bigl = 2. / (2 as f64).sqrt();

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
    fn test_point_concave_loop_interior_with_clean() {
        //Vector3D normal = Vector3D(0, 0, 1);
        let mut the_loop = Loop3D::new();
        let l = 1. / (2 as f64).sqrt();
        let bigl = 2. / (2 as f64).sqrt();

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
                -1.5 / (2 as f64).sqrt(),
                -1.5 / (2 as f64).sqrt(),
                0.,
            ))
            .unwrap();
        assert!(r);
    }

    #[test]
    fn test_point_non_coplanar() {
        let mut the_loop = Loop3D::new();
        let l = 1. / (2 as f64).sqrt();

        the_loop.push(Point3D::new(-l, -l, 0.)).unwrap();
        the_loop.push(Point3D::new(-l, l, 0.)).unwrap();
        the_loop.push(Point3D::new(l, l, 0.)).unwrap();
        the_loop.push(Point3D::new(l, -l, 0.)).unwrap();
        the_loop.close().unwrap();

        let r = the_loop
            .test_point(Point3D::new(
                -1.5 / (2 as f64).sqrt(),
                -1.5 / (2 as f64).sqrt(),
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

        the_loop.close().unwrap();

        let a = the_loop.area();
        assert_eq!(4. * l * l, a);
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

        the_loop.close().unwrap();

        let a = the_loop.area();
        assert_eq!(3.0, a);
        assert!(the_loop.normal().compare(Vector3D::new(0., 0., 1.)));

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

        let a = the_loop.area();
        assert_eq!(3.0, a);
        assert!(the_loop.normal().compare(Vector3D::new(0., 0., -1.)));
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
        assert!(!l.is_diagonal(Segment3D::new(
            Point3D::new(-4., 0., 0.),
            Point3D::new(4., 0., 0.),
        )));

        // Doesn't intersect, but is outside
        assert!(!l.is_diagonal(Segment3D::new(
            Point3D::new(4., 0., 0.),
            Point3D::new(5., 0., 0.),
        )));

        // Doesn't intersect, is inside == is_diagonal
        assert!(l.is_diagonal(Segment3D::new(
            Point3D::new(-2., -2., 0.),
            Point3D::new(2., 2., 0.),
        )));
    }

    
}
