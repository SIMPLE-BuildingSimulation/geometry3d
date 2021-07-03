use crate::intersect_trait::Intersect;
use crate::point3d::*;
use crate::segment3d::*;
use crate::ray3d::Ray3D;
use crate::vector3d::Vector3D;

pub struct Triangle3D {
    // Vertices
    v0: Point3D,
    v1: Point3D,
    v2: Point3D,

    // Segments
    s0: Segment3D,
    s1: Segment3D,
    s2: Segment3D,

    // Neighbours
    n0: i32,
    n1: i32,
    n2: i32,

    // Constraints
    c0: bool,
    c1: bool,
    c2: bool,

    // Other info about the triangle.
    area: f64,
    circumradius: f64,
    aspect_ratio: f64,
    circumcenter: Point3D,
    centroid: Point3D,

    valid: bool,

    // This will be
    // assigned when pushing it into a
    // triangulation.
    index: usize,
}

#[derive(Clone,Copy,PartialEq,Eq)]
pub enum PointInTriangle{
    VertexA,
    VertexB,
    VertexC,
    EdgeAB,
    EdgeBC, 
    EdgeAC,
    Inside,
    Outside
}

impl PointInTriangle {
    pub fn is_vertex(&self)->bool{
        matches!(self,
            Self::VertexA |
            Self::VertexB |
            Self::VertexC
        )
    }

    pub fn is_edge(&self)->bool{
        matches!(self,
            Self::EdgeAB |
            Self::EdgeAC |
            Self::EdgeBC
         )        
    }
}

impl Triangle3D {
    pub fn new(
        vertex_a: Point3D,
        vertex_b: Point3D,
        vertex_c: Point3D,
        i: usize,
    ) -> Result<Triangle3D, String> {
        // check that points are not the same
        if vertex_a.compare(vertex_b) || vertex_a.compare(vertex_c) || vertex_b.compare(vertex_c) {
            let msg = "Trying to build a Triangle3D with two equal points".to_string();
            return Err(msg);
        }

        // Check that they are not collinear
        if vertex_a.is_collinear(vertex_b, vertex_c).unwrap() {
            let msg = "Trying to build a Triangle3D from collinear points".to_string();
            return Err(msg);
        }

        // Build the triangle
        let mut t = Triangle3D {
            v0: vertex_a,
            v1: vertex_b,
            v2: vertex_c,

            s0: Segment3D::new(vertex_a, vertex_b),
            s1: Segment3D::new(vertex_b, vertex_c),
            s2: Segment3D::new(vertex_c, vertex_a),

            n0: -1,
            n1: -1,
            n2: -1,

            c0: false,
            c1: false,
            c2: false,

            area: -1.,
            aspect_ratio: -1.,
            circumradius: -1.,
            circumcenter: Point3D::new(0., 0., 0.),
            centroid: Point3D::new(0., 0., 0.),

            index: i,
            valid: true,
        };

        t.set_area();
        t.set_circumradius();
        t.set_aspect_ratio();
        t.set_circumcenter();
        t.set_centroid();

        Ok(t)
    }

    pub fn index(&self) -> usize {
        self.index
    }

    pub fn invalidate(&mut self) {
        self.valid = false
    }

    pub fn is_valid(&self) -> bool {
        self.valid
    }

    pub fn set_neighbour(&mut self, edge: usize, i: usize) -> Result<(), String> {
        match edge {
            0 => self.n0 = i as i32,
            1 => self.n1 = i as i32,
            2 => self.n2 = i as i32,
            _ => {
                return Err(format!(
                    "Trying to set neighbour with index '{}'... it is out of bounds",
                    i
                ))
            }
        }
        Ok(())
    }

    pub fn neighbour(&self, edge: usize) -> Result<i32, String> {
        match edge {
            0 => Ok(self.n0),
            1 => Ok(self.n1),
            2 => Ok(self.n2),
            _ => Err(format!(
                "Trying to get neighbour with index '{}'... it is out of bounds",
                edge
            )),
        }
    }

    /* Area */

    fn set_area(&mut self) {
        if self.area < 0. {
            let a = self.s0.length();
            let b = self.s1.length();
            let c = self.s2.length();

            self.area = ((c + b + a)
                * ((c + b + a) / 2. - a)
                * ((c + b + a) / 2. - b)
                * ((c + b + a) / 2. - c)
                / 2.)
                .sqrt();
        }
    }

    pub fn area(&self) -> f64 {
        self.area
    }

    /* Circumradius */

    fn set_circumradius(&mut self) {
        let a = self.s0.length();
        let b = self.s1.length();
        let c = self.s2.length();
        let s = (a + b + c) * (b + c - a) * (c + a - b) * (a + b - c);
        self.circumradius = a * b * c / s.sqrt();
    }

    pub fn circumradius(&self) -> f64 {
        self.circumradius
    }

    /* Asect ratio */

    fn set_aspect_ratio(&mut self) {
        let mut min_segment = 1E19;
        for i in 0..3 {
            let s = self.segment(i).unwrap();
            if s.length() < min_segment {
                min_segment = s.length();
            }
        }
        let ar = self.circumradius() / min_segment;
        self.aspect_ratio = ar;
    }

    pub fn aspect_ratio(&self) -> f64 {
        self.aspect_ratio
    }

    /* Circumcenter */

    fn set_circumcenter(&mut self) {
        let ab = self.v1 - self.v0;
        let ac = self.v2 - self.v0;

        let ab_cross_ac = ab.cross(ac);

        let ac_sq_length = ac.length() * ac.length();
        let ab_sq_length = ab.length() * ab.length();
        let ab_x_ac_sq_length = ab_cross_ac.length() * ab_cross_ac.length();

        let a_center = ((ab_cross_ac.cross(ab)) * ac_sq_length
            + ac.cross(ab_cross_ac) * ab_sq_length)
            / (2. * ab_x_ac_sq_length);

        self.circumcenter = self.v0 + a_center;
    }

    pub fn circumcenter(&self) -> Point3D {
        self.circumcenter
    }

    /* Centroid */

    fn set_centroid(&mut self) {
        let dx = self.v0.x + self.v1.x + self.v2.x;
        let dy = self.v0.y + self.v1.y + self.v2.y;
        let dz = self.v0.z + self.v1.z + self.v2.z;

        self.centroid = Point3D::new(dx / 3., dy / 3., dz / 3.);
    }

    pub fn centroid(&self) -> Point3D {
        self.centroid
    }

    /* Other */

    pub fn vertex(&self, i: usize) -> Result<Point3D, String> {
        match i {
            0 => Ok(self.v0),
            1 => Ok(self.v1),
            2 => Ok(self.v2),
            _ => Err(format!(
                "Trying to get vertex with index '{}'... it is out of bounds",
                i
            )),
        }
    }

    pub fn a(&self)->Point3D{
        self.v0
    }
    pub fn b(&self)->Point3D{
        self.v1
    }
    pub fn c(&self)->Point3D{
        self.v0
    }

    pub fn segment(&self, i: usize) -> Result<Segment3D, String> {
        match i {
            0 => Ok(self.s0),
            1 => Ok(self.s1),
            2 => Ok(self.s2),
            _ => Err(format!(
                "Trying to get Segment with index '{}'... it is out of bounds",
                i
            )),
        }
    }

    pub fn ab(&self)->Segment3D{
        self.s0
    }

    pub fn bc(&self)->Segment3D{
        self.s1
    }
    pub fn ca(&self)->Segment3D{
        self.s2
    }

    /// Tests whether a point—which is assumed to be within the plane
    /// of the triangle—is inside the triangle.
    /// 
    /// # Explanation
    /// This is not something I made up. I found it [here](http://blackpawn.com/texts/pointinpoly/)
    /// and also in other places. (the explanation is here not as novelty)
    ///  
    /// If a triangle in 3 dimentions as vertices $`\vec{A}`$, $`\vec{B}`$, and $`\vec{C}`$, then any 
    /// point in the plane of the triangle can be described by the following equation:
    /// 
    /// ```math 
    /// \vec{P} = \vec{A} + \alpha \left(\vec{B}-\vec{A}\right)+\beta \left(\vec{C}-\vec{A}\right)
    /// ```
    /// Or, after stating that $`\hat{e}_1=\left(\vec{B}-\vec{A}\right)`$ and $`\hat{e}_2 = \left(\vec{C}-\vec{A}\right)`$ 
    /// and rearranging, we can express the same equation as:
    ///     
    /// ```math
    /// \vec{P} - \vec{A}=  \alpha \hat{e}_1+\beta \hat{e}_2
    /// ```
    /// 
    /// Since this is a 3D equation,  it is actually three equations and there are only two incognita 
    /// ($`\alpha`$ and $`\beta`$). So, we can multiply it by $`\hat{e}_1`$ and $`\hat{e}_2`$ and produce 
    /// a system of equations that we can solve. After doing this, the equation above becomes:
    /// 
    /// ```math
    /// \hat{e}_1\left(\vec{P} - \vec{A}\right)=  \alpha \hat{e}_1\hat{e}_1+\beta \hat{e}_1\hat{e}_2
    /// ```
    /// and
    /// ```math
    /// \hat{e}_2\left(\vec{P} - \vec{A}\right)=  \alpha \hat{e}_1\hat{e}_2+\beta \hat{e}_2\hat{e}_2
    /// ```
    /// 
    /// Which can be represented in matrix form as follows:
    /// 
    /// ```math
    /// \begin{Bmatrix}\hat{e}_1\left(\vec{P} - \vec{A}\right)\\\hat{e}_2\left(\vec{P} - \vec{A}\right)\end{Bmatrix}=\begin{pmatrix}\hat{e}_1\hat{e}_1&\hat{e}_1\hat{e}_2\\\hat{e}_1\hat{e}_2&\hat{e}_2\hat{e}_2\end{pmatrix}\begin{Bmatrix}\alpha\\\beta\end{Bmatrix}
    /// ```
    /// 
    /// What is left from here is to interpret the values of $`\alpha`$ and $`\beta`$ from the equations, 
    /// which allows us to position the point within the triangle.
    pub fn test_point(&self, p: Point3D) -> PointInTriangle {
        
        // get vertices
        let vertex_a = self.v0;
        let vertex_b = self.v1;
        let vertex_c = self.v2;

        // Compute vectors
        let e1 = vertex_b - vertex_a;
        let e2 = vertex_c - vertex_a;
        let p_minus_a = p - vertex_a;

        // Compute dot products
        let e2e2 = e2 * e2;
        let e1e2 = e2 * e1;
        let e1e1 = e1 * e1;

        let left1 = e1 * p_minus_a;
        let left2 = e2 * p_minus_a;

        // Compute barycentric coordinates
        let det =  e1e1*e2e2 - e1e2 * e1e2;        
        let alpha = ( e2e2 * left1 - e1e2 * left2) / det;
        let beta =  (-e1e2 * left1 + e1e1 * left2) / det;
        let w = 1. - alpha - beta;

        // Check if point is in triangle
        if alpha >= -f64::EPSILON && beta >= -f64::EPSILON && w >= -f64::EPSILON {
            // Somewhere in the triangle
            if alpha <= f64::EPSILON && beta <= f64::EPSILON {
                // w is 1... Vertex A
                PointInTriangle::VertexA
            } else if alpha <= f64::EPSILON && w <= f64::EPSILON {
                // beta is 1; Vertex C
                PointInTriangle::VertexC
            } else if beta <= f64::EPSILON && w <= f64::EPSILON {
                // Alpha is 1; Vertex B
                PointInTriangle::VertexB
            } else if alpha <= f64::EPSILON {
                PointInTriangle::EdgeAC
            } else if w <= f64::EPSILON {
                PointInTriangle::EdgeBC
            } else if beta <= f64::EPSILON {
                PointInTriangle::EdgeAB
            } else {
                PointInTriangle::Inside
            }            
        }else{            
            PointInTriangle::Outside
        }
    }    

    pub fn get_edge_index_by_points(&self, a: Point3D, b: Point3D) -> Option<usize> {
        if a.compare(self.s0.start()) && b.compare(self.s0.end())
            || a.compare(self.s0.end()) && b.compare(self.s0.start())
        {
            return Some(0);
        }

        if a.compare(self.s1.start()) && b.compare(self.s1.end())
            || a.compare(self.s1.end()) && b.compare(self.s1.start())
        {
            return Some(1);
        }

        if a.compare(self.s2.start()) && b.compare(self.s2.end())
            || a.compare(self.s2.end()) && b.compare(self.s2.start())
        {
            return Some(2);
        }

        None
    }

    pub fn has_vertex(&self, p: Point3D) -> bool {
        // is this faster than iterate?
        if self.v0.compare(p) {
            return true;
        }
        if self.v1.compare(p) {
            return true;
        }
        if self.v2.compare(p) {
            return true;
        }

        false
    }

    pub fn compare(&self, t: &Triangle3D) -> bool {
        // Vertices do not need to be in the same order,
        // necessarily... So, check if the three vertices
        // are in the other triangle.

        // is this faster than iterate?
        if !t.has_vertex(self.v0) {
            return false;
        }
        if !t.has_vertex(self.v1) {
            return false;
        }
        if !t.has_vertex(self.v2) {
            return false;
        }
        true
    }

    pub fn constrain(&mut self, i: usize) -> Result<(), String> {
        match i {
            0 => {
                self.c0 = true;
                Ok(())
            }
            1 => {
                self.c1 = true;
                Ok(())
            }
            2 => {
                self.c2 = true;
                Ok(())
            }
            _ => Err(format!(
                "Trying to constrain a vertex with index '{}'... it is out of bounds",
                i
            )),
        }
    }

    pub fn is_constrained(&self, i: usize) -> Result<bool, String> {
        match i {
            0 => Ok(self.c0),
            1 => Ok(self.c1),
            2 => Ok(self.c2),
            _ => Err(format!(
                "Checking for a constrained vertex with index '{}'... it is out of bounds",
                i
            )),
        }
    }
}

fn det_3x3(col0: &Vector3D,col1:&Vector3D,col2:&Vector3D)->f64{
     col0.x()*(col1.y()*col2.z()-col2.y()*col1.z())
    -col1.x()*(col0.y()*col2.z()-col2.y()*col0.z())
    +col2.x()*(col0.y()*col1.z()-col1.y()*col0.z())
}

impl Intersect for Triangle3D{
    fn intersect(&self, ray: &Ray3D)->Option<f64>{
        // Solve `Rorigin+t*Rdirection = A + alpha*(B-A) + beta*(C-A)`;
        // Meaning: 
        // alpha(A-B) + beta(A-C) + t*Rdirection = (A - Rorigin)
        // Solve this with Cramer's rule
        
        let a_ro = self.a() - ray.origin();
        let a_b = self.a() - self.b();
        let a_c = self.a()-self.b();
        let rd = ray.direction();

        let det_a = det_3x3(&a_b, &a_c, &rd);

        let alpha = det_3x3(&a_ro, &a_c, &rd)/det_a;
        let beta = det_3x3(&a_b, &a_ro, &rd)/det_a;            
        let t = det_3x3(&a_b, &a_c, &a_ro)/det_a;  


        // t must be positive, and alpha, beta and gamma must add to 1 and 
        // be positive
        if t < 0. || alpha + beta > 1. || alpha < 0. || beta < 0. {
            None
        }else{
            Some(t)
        }        
    }
}

/***********/
/* TESTING */
/***********/

#[cfg(test)]
mod testing {
    use super::*;

    #[test]
    fn test_det_3x3(){
        assert_eq!(1.,det_3x3(
            &Vector3D::new(1.,0.,0.),
            &Vector3D::new(0.,1.,0.),
            &Vector3D::new(0.,0.,1.)));
        assert_eq!(8.,det_3x3(
            &Vector3D::new(2.,0.,0.),
            &Vector3D::new(0.,2.,0.),
            &Vector3D::new(0.,0.,2.)));
        assert_eq!(49.,det_3x3(
            &Vector3D::new(2.,2.,1.),
            &Vector3D::new(-3.,0.,4.),
            &Vector3D::new(1.,-1.,5.)));        
    }

    #[test]
    fn test_new() {
        // Buid triangle
        // This is a right triangle, so it fits
        // within a circle, and has a circumradius equals
        // half of its hypotenuse
        let a = Point3D::new(0., 0., 0.);
        let b = Point3D::new(6., 0., 0.);
        let c = Point3D::new(0., 8., 0.);

        let t = Triangle3D::new(a, b, c, 2).unwrap();

        assert_eq!(t.index(), 2);

        // Test vertices
        assert!(t.v0.compare(a));
        assert!(t.v1.compare(b));
        assert!(t.v2.compare(c));

        let v0 = t.vertex(0).unwrap();
        assert!(v0.compare(a));
        let v1 = t.vertex(1).unwrap();
        assert!(v1.compare(b));
        let v2 = t.vertex(2).unwrap();
        assert!(v2.compare(c));

        // test segments
        let ab = t.segment(0).unwrap();
        let ab_ref = Segment3D::new(a, b);
        assert!(ab.compare(&ab_ref));

        let bc = t.segment(1).unwrap();
        let bc_ref = Segment3D::new(b, c);
        assert!(bc.compare(&bc_ref));

        let ca = t.segment(2).unwrap();
        let ca_ref = Segment3D::new(c, a);
        assert!(ca.compare(&ca_ref));

        // Check that getting out of bounds is error
        assert!(t.vertex(3).is_err());

        // Check that the area is OK
        assert!((t.area() - 6. * 8. / 2.).abs() < 1E-10);

        // Check circumradius
        assert_eq!(t.circumradius(), 5.);

        // Check aspect ratio; circumradius/smallest side
        assert_eq!(t.aspect_ratio(), 5. / 6.);

        // Check circumcenter
        assert!(t.circumcenter().compare(Point3D::new(3., 4., 0.)));

        // check centroid
        assert!(t.centroid().compare(Point3D::new(6. / 3., 8. / 3., 0.)));
    }

    #[test]
    fn test_test_point() {

        let a = Point3D::new(0., 0., 0.);
        let b = Point3D::new(1., 0., 0.);
        let c = Point3D::new(0., 1., 0.);

        let triangle = Triangle3D::new(a, b, c, 1).unwrap();

        // Vertex A.
        assert!(PointInTriangle::VertexA == triangle.test_point(a));        

        // Vertex B.
        assert!(PointInTriangle::VertexB == triangle.test_point(b));       

        // Vertex C.
        assert!(PointInTriangle::VertexC == triangle.test_point(c));
        
        // Segment AB.
        let p = Point3D::new(0.5, 0., 0.);
        assert!(PointInTriangle::EdgeAB == triangle.test_point(p));

        // Segment AC.
        let p = Point3D::new(0., 0.5, 0.);
        assert!(PointInTriangle::EdgeAC == triangle.test_point(p));

        // Segment BC.
        let p = Point3D::new(0.5, 0.5, 0.);
        assert!(PointInTriangle::EdgeBC == triangle.test_point(p));
        
        // Point outside
        let p = Point3D::new(0., -1., 0.);
        assert!(PointInTriangle::Outside == triangle.test_point(p));        

        // Point inside
        let p = Point3D::new(0.1, 0.1, 0.);
        assert!(PointInTriangle::Inside == triangle.test_point(p));        
    }

    #[test]
    fn test_circumcenter() {
        // TEST 1... centerd in origin
        let mut l = 3.;
        let a = Point3D::new(-l, 0., 0.);
        let b = Point3D::new(l, 0., 0.);
        let c = Point3D::new(0., l, 0.);

        let t = Triangle3D::new(a, b, c, 2).unwrap();

        let center1 = t.circumcenter();
        assert!(center1.compare(Point3D::new(0., 0., 0.)));

        // Test 2: Centered somewhere else
        l = 3.;
        let cx = 23.;
        let cy = 21.;
        let cz = 3.12314;
        let a2 = Point3D::new(-l + cx, 0. + cy, cz);
        let b2 = Point3D::new(l + cx, 0. + cy, cz);
        let c2 = Point3D::new(0. + cx, l + cy, cz);

        let t2 = Triangle3D::new(a2, b2, c2, 2).unwrap();

        let center2 = t2.circumcenter();

        assert!(center2.compare(Point3D::new(cx, cy, cz)));

        // Test 3: From internet solved problem
        let a3 = Point3D::new(3., 2., 0.);
        let b3 = Point3D::new(1., 4., 0.);
        let c3 = Point3D::new(5., 4., 0.);

        let t3 = Triangle3D::new(a3, b3, c3, 1).unwrap();
        let center3 = t3.circumcenter();
        assert!(center3.compare(Point3D::new(3., 4., 0.)));
    }

    #[test]
    fn test_get_edge_index_by_points() {
        let a = Point3D::new(0., 0., 0.);
        let b = Point3D::new(6., 0., 0.);
        let c = Point3D::new(0., 8., 0.);

        let t = Triangle3D::new(a, b, c, 0).unwrap();

        assert_eq!(t.get_edge_index_by_points(a, b), Some(0));
        assert_eq!(t.get_edge_index_by_points(b, a), Some(0));
        assert_eq!(t.get_edge_index_by_points(b, c), Some(1));
        assert_eq!(t.get_edge_index_by_points(c, b), Some(1));
        assert_eq!(t.get_edge_index_by_points(c, a), Some(2));
        assert_eq!(t.get_edge_index_by_points(a, c), Some(2));

        // test segments
        let ab = t.segment(0).unwrap();
        let ab_ref = Segment3D::new(a, b);
        assert!(ab.compare(&ab_ref));

        let bc = t.segment(1).unwrap();
        let bc_ref = Segment3D::new(b, c);
        assert!(bc.compare(&bc_ref));

        let ca = t.segment(2).unwrap();
        let ca_ref = Segment3D::new(c, a);
        assert!(ca.compare(&ca_ref));
    }

    #[test]
    fn test_has_vertex() {
        let a = Point3D::new(0., 0., 0.);
        let b = Point3D::new(6., 0., 0.);
        let c = Point3D::new(0., 8., 0.);

        let not_there = Point3D::new(12., 1., 99.);

        let t = Triangle3D::new(a, b, c, 0).unwrap();

        assert!(t.has_vertex(a));
        assert!(t.has_vertex(b));
        assert!(t.has_vertex(c));

        assert!(!t.has_vertex(not_there));

        let t2 = Triangle3D::new(a, c, b, 0).unwrap();
        assert!(t.compare(&t2));

        let t3 = Triangle3D::new(a, c, not_there, 0).unwrap();
        assert!(!t.compare(&t3));
    }

    #[test]
    fn test_constraints() {
        let l = 3.;
        let a = Point3D::new(-l, 0., 0.);
        let b = Point3D::new(l, 0., 0.);
        let c = Point3D::new(0., l, 0.);

        let mut t = Triangle3D::new(a, b, c, 2).unwrap();

        assert!(!t.is_constrained(0).unwrap());
        assert!(!t.is_constrained(1).unwrap());
        assert!(!t.is_constrained(2).unwrap());
        assert!(t.is_constrained(3).is_err());

        t.constrain(0).unwrap();

        assert!(t.is_constrained(0).unwrap());
        assert!(!t.is_constrained(1).unwrap());
        assert!(!t.is_constrained(2).unwrap());

        assert!(t.constrain(3).is_err());
    }

    #[test]
    fn test_set_neighbour() {
        // This should work.
        let l = 3.;
        let a = Point3D::new(-l, 0., 0.);
        let b = Point3D::new(l, 0., 0.);
        let c = Point3D::new(0., l, 0.);

        let mut t1 = Triangle3D::new(a, b, c, 0).unwrap();

        assert!(t1.set_neighbour(0, 1).is_ok());
        assert!(t1.set_neighbour(1, 11).is_ok());
        assert!(t1.set_neighbour(2, 111).is_ok());

        assert_eq!(t1.n0, 1);
        assert_eq!(t1.n1, 11);
        assert_eq!(t1.n2, 111);
    }
}
