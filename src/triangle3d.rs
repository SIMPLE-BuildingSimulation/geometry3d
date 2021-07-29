use crate::intersect_trait::{Intersect, SurfaceSide};
use crate::point3d::*;
use crate::ray3d::Ray3D;
use crate::segment3d::*;
use crate::vector3d::Vector3D;

pub struct Triangle3D {
    // Vertices
    a: Point3D,
    b: Point3D,
    c: Point3D,
    normal: Vector3D,
    area: f64,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum PointInTriangle {
    VertexA,
    VertexB,
    VertexC,
    EdgeAB,
    EdgeBC,
    EdgeAC,
    Inside,
    Outside,
}

impl PointInTriangle {
    pub fn is_vertex(&self) -> bool {
        matches!(self, Self::VertexA | Self::VertexB | Self::VertexC)
    }

    pub fn is_edge(&self) -> bool {
        matches!(self, Self::EdgeAB | Self::EdgeAC | Self::EdgeBC)
    }
}

impl std::fmt::Display for Triangle3D {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Triangle3D[\n\t{}\n\t{}\n\t{}\n]\n",
            self.a, self.b, self.c
        )
    }
}

impl Triangle3D {
    pub fn new(vertex_a: Point3D, vertex_b: Point3D, vertex_c: Point3D) -> Result<Self, String> {
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
            a: vertex_a,
            b: vertex_b,
            c: vertex_c,

            area: -1.,
            normal: Vector3D::new(0., 0., 0.),
        };

        t.set_area();
        t.set_normal();

        Ok(t)
    }

    /* Normal */

    /// Calculates the normal of the Triangle
    fn set_normal(&mut self) {
        let b_a = self.b - self.a;
        let c_a = self.c - self.b;
        self.normal = b_a.cross(c_a);
        self.normal.normalize();
    }

    /// Gets the normal of the [`Triangle3D`]. This value is
    /// cached, so this should be fast.
    pub fn normal(&self) -> Vector3D {
        self.normal
    }

    /// Sets the area of the [`Triangle3D`]. This is used when
    /// constructing the [`Triangle3D`]
    fn set_area(&mut self) {
        let ab = self.ab().length();
        let bc = self.bc().length();
        let ca = self.ca().length();

        self.area = ((ca + bc + ab)
            * ((ca + bc + ab) / 2. - ab)
            * ((ca + bc + ab) / 2. - bc)
            * ((ca + bc + ab) / 2. - ca)
            / 2.)
            .sqrt();
    }

    /// Gets the Area of the [`Triangle3D`]... it is cached when creater
    pub fn area(&self) -> f64 {
        self.area
    }

    /// Gets the Circumradus of the [`Triangle3D`]. This value
    /// is not cached, so it might be slow to rely on this
    /// too often.
    pub fn circumradius(&self) -> f64 {
        let a = self.ab().length();
        let b = self.bc().length();
        let c = self.ca().length();
        let s = (a + b + c) * (b + c - a) * (c + a - b) * (a + b - c);
        a * b * c / s.sqrt()
    }

    /// Gets the Aspect Ratio of the [`Triangle3D`]. This is useful
    /// for triangulating polygons.
    pub fn aspect_ratio(&self) -> f64 {
        let mut min_segment = 1E19;
        for i in 0..3 {
            let s = self.segment(i).unwrap();
            if s.length() < min_segment {
                min_segment = s.length();
            }
        }
        // return
        self.circumradius() / min_segment
    }

    /// retrieves the Circumcenter
    pub fn circumcenter(&self) -> Point3D {
        let ab = self.b - self.a;
        let ac = self.c - self.a;

        let ab_cross_ac = ab.cross(ac);

        let ac_sq_length = ac.length() * ac.length();
        let ab_sq_length = ab.length() * ab.length();
        let ab_x_ac_sq_length = ab_cross_ac.length() * ab_cross_ac.length();

        let a_center = ((ab_cross_ac.cross(ab)) * ac_sq_length
            + ac.cross(ab_cross_ac) * ab_sq_length)
            / (2. * ab_x_ac_sq_length);

        self.a + a_center
    }

    /// Gets the centroid of the [`Triangle3D`]. This value
    /// is not cached, so it might be slow to rely on this
    /// too often.
    pub fn centroid(&self) -> Point3D {
        let dx = self.a.x + self.b.x + self.c.x;
        let dy = self.a.y + self.b.y + self.c.y;
        let dz = self.a.z + self.b.z + self.c.z;

        Point3D::new(dx / 3., dy / 3., dz / 3.)
    }

    /* Other */

    pub fn vertex(&self, i: usize) -> Result<Point3D, String> {
        match i {
            0 => Ok(self.a),
            1 => Ok(self.b),
            2 => Ok(self.c),
            _ => Err(format!(
                "Trying to get vertex with index '{}'... it is out of bounds",
                i
            )),
        }
    }

    /// Retrieves the first vertex of an A,B,C [`Triangle3D`]
    pub fn a(&self) -> Point3D {
        self.a
    }

    /// Retrieves the second vertex of an A,B,C [`Triangle3D`]
    pub fn b(&self) -> Point3D {
        self.b
    }

    /// Retrieves the third vertex of an A,B,C [`Triangle3D`]
    pub fn c(&self) -> Point3D {
        self.c
    }

    /// Retrieves a segment of an A,B,C [`Triangle3D`]
    pub fn segment(&self, i: usize) -> Result<Segment3D, String> {
        match i {
            0 => Ok(self.ab()),
            1 => Ok(self.bc()),
            2 => Ok(self.ca()),
            _ => Err(format!(
                "Trying to get Segment with index '{}'... it is out of bounds",
                i
            )),
        }
    }

    /// Retrieves the first segment of an A,B,C [`Triangle3D`]
    pub fn ab(&self) -> Segment3D {
        Segment3D::new(self.a, self.b)
    }

    /// Retrieves the second segment of an A,B,C [`Triangle3D`]
    pub fn bc(&self) -> Segment3D {
        Segment3D::new(self.b, self.c)
    }

    /// Retrieves the third segment of an A,B,C [`Triangle3D`]
    pub fn ca(&self) -> Segment3D {
        Segment3D::new(self.c, self.a)
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
        let vertex_a = self.a;
        let vertex_b = self.b;
        let vertex_c = self.c;

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
        let det = e1e1 * e2e2 - e1e2 * e1e2;
        let alpha = (e2e2 * left1 - e1e2 * left2) / det;
        let beta = (-e1e2 * left1 + e1e1 * left2) / det;
        let w = 1. - alpha - beta;

        // Check if point is in triangle
        const TINY: f64 = 100. * f64::EPSILON;
        if alpha >= -TINY && beta >= -TINY && w >= -TINY {
            // Somewhere in the triangle
            if alpha <= TINY && beta <= TINY {
                // w is 1... Vertex A
                PointInTriangle::VertexA
            } else if alpha <= TINY && w <= TINY {
                // beta is 1; Vertex C
                PointInTriangle::VertexC
            } else if beta <= TINY && w <= TINY {
                // Alpha is 1; Vertex B
                PointInTriangle::VertexB
            } else if alpha <= TINY {
                PointInTriangle::EdgeAC
            } else if w <= TINY {
                PointInTriangle::EdgeBC
            } else if beta <= TINY {
                PointInTriangle::EdgeAB
            } else {
                PointInTriangle::Inside
            }
        } else {
            PointInTriangle::Outside
        }
    }

    /// Gets the edge number corresponding to a [`Segment3D`] in the
    /// Triangle.
    pub fn get_edge_index_from_segment(&self, s: &Segment3D) -> Option<usize> {
        if s.compare(&self.ab()) {
            Some(0)
        } else if s.compare(&self.bc()) {
            Some(1)
        } else if s.compare(&self.ca()) {
            Some(2)
        } else {
            None
        }
    }

    pub fn get_edge_index_from_points(&self, a: Point3D, b: Point3D) -> Option<usize> {
        let segment = Segment3D::new(a, b);
        self.get_edge_index_from_segment(&segment)
    }

    /// Checks if a [`Triangle3D`] contains a certain Vertex in
    /// the same position as a [`Point3D`] `p`
    pub fn has_vertex(&self, p: Point3D) -> bool {
        // is this faster than iterate?
        if self.a.compare(p) {
            true
        } else if self.b.compare(p) {
            true
        } else if self.c.compare(p) {
            true
        } else {
            false
        }
    }

    /// Checks whether three [`Triangle3D`] objects are made
    /// of he same vertices.
    ///
    /// Vertices do not need to be in the same order,
    /// necessarily... So, check if the three vertices
    /// are in the other triangle.
    pub fn compare(&self, t: &Triangle3D) -> bool {
        // is this faster than iterate?
        if !t.has_vertex(self.a) {
            return false;
        }
        if !t.has_vertex(self.b) {
            return false;
        }
        if !t.has_vertex(self.c) {
            return false;
        }
        true
    }
}

fn det_3x3(col0: &Vector3D, col1: &Vector3D, col2: &Vector3D) -> f64 {
    col0.x * (col1.y * col2.z - col2.y * col1.z)
        - col1.x * (col0.y * col2.z - col2.y * col0.z)
        + col2.x * (col0.y * col1.z - col1.y * col0.z)
}

impl Intersect for Triangle3D {
    
    const ID : &'static str = "triangle";

    fn intersect(&self, ray: &Ray3D) -> Option<f64> {
        // Solve `Rorigin+t*Rdirection = A + alpha*(B-A) + beta*(C-A)`;
        // Meaning:
        // alpha(A-B) + beta(A-C) + t*Rdirection = (A - Rorigin)
        // Solve this with Cramer's rule

        let a_ro = self.a() - ray.origin;
        let a_b = self.a() - self.b();
        let a_c = self.a() - self.b();
        let rd = ray.direction;

        let det_a = det_3x3(&a_b, &a_c, &rd);

        let alpha = det_3x3(&a_ro, &a_c, &rd) / det_a;
        let beta = det_3x3(&a_b, &a_ro, &rd) / det_a;
        let t = det_3x3(&a_b, &a_c, &a_ro) / det_a;

        // t must be positive, and alpha, beta and gamma must add to 1 and
        // be positive
        if t < 0. || alpha + beta > 1. || alpha < 0. || beta < 0. {
            None
        } else {
            Some(t)            
        }
    }

    fn normal_at_intersection(&self, ray: &Ray3D, _t: f64)->(Vector3D, SurfaceSide){
        let (side, normal) = SurfaceSide::get_side(self.normal, ray.direction);
        (normal, side)
    }
}

/***********/
/* TESTING */
/***********/

#[cfg(test)]
mod testing {
    use super::*;

    #[test]
    fn test_det_3x3() {
        assert_eq!(
            1.,
            det_3x3(
                &Vector3D::new(1., 0., 0.),
                &Vector3D::new(0., 1., 0.),
                &Vector3D::new(0., 0., 1.)
            )
        );
        assert_eq!(
            8.,
            det_3x3(
                &Vector3D::new(2., 0., 0.),
                &Vector3D::new(0., 2., 0.),
                &Vector3D::new(0., 0., 2.)
            )
        );
        assert_eq!(
            49.,
            det_3x3(
                &Vector3D::new(2., 2., 1.),
                &Vector3D::new(-3., 0., 4.),
                &Vector3D::new(1., -1., 5.)
            )
        );
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

        let t = Triangle3D::new(a, b, c).unwrap();

        // Test vertices
        assert!(t.a.compare(a));
        assert!(t.b.compare(b));
        assert!(t.c.compare(c));

        let a = t.vertex(0).unwrap();
        assert!(a.compare(a));
        let b = t.vertex(1).unwrap();
        assert!(b.compare(b));
        let c = t.vertex(2).unwrap();
        assert!(c.compare(c));

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

        let triangle = Triangle3D::new(a, b, c).unwrap();

        // Vertex A.
        assert_eq!(PointInTriangle::VertexA, triangle.test_point(a));

        // Vertex B.
        assert_eq!(PointInTriangle::VertexB, triangle.test_point(b));

        // Vertex C.
        assert_eq!(PointInTriangle::VertexC, triangle.test_point(c));

        // Segment AB.
        let p = Point3D::new(0.5, 0., 0.);
        assert_eq!(PointInTriangle::EdgeAB, triangle.test_point(p));

        // Segment AC.
        let p = Point3D::new(0., 0.5, 0.);
        assert_eq!(PointInTriangle::EdgeAC, triangle.test_point(p));

        // Segment BC.
        let p = Point3D::new(0.5, 0.5, 0.);
        assert_eq!(PointInTriangle::EdgeBC, triangle.test_point(p));

        // Point outside
        let p = Point3D::new(0., -1., 0.);
        assert_eq!(PointInTriangle::Outside, triangle.test_point(p));

        // Point inside
        let p = Point3D::new(0.1, 0.1, 0.);
        assert_eq!(PointInTriangle::Inside, triangle.test_point(p));

        // CASE 2
        let a = Point3D::new(2., 2., 0.);
        let b = Point3D::new(0., 0., 0.);
        let c = Point3D::new(4., 0., 0.);
        let triangle = Triangle3D::new(a, b, c).unwrap();
        let p = Point3D::new(2., 0., 0.);

        assert_eq!(PointInTriangle::EdgeBC, triangle.test_point(p));
    }

    #[test]
    fn test_circumcenter() {
        // TEST 1... centerd in origin
        let mut l = 3.;
        let a = Point3D::new(-l, 0., 0.);
        let b = Point3D::new(l, 0., 0.);
        let c = Point3D::new(0., l, 0.);

        let t = Triangle3D::new(a, b, c).unwrap();

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

        let t2 = Triangle3D::new(a2, b2, c2).unwrap();

        let center2 = t2.circumcenter();

        assert!(center2.compare(Point3D::new(cx, cy, cz)));

        // Test 3: From internet solved problem
        let a3 = Point3D::new(3., 2., 0.);
        let b3 = Point3D::new(1., 4., 0.);
        let c3 = Point3D::new(5., 4., 0.);

        let t3 = Triangle3D::new(a3, b3, c3).unwrap();
        let center3 = t3.circumcenter();
        assert!(center3.compare(Point3D::new(3., 4., 0.)));
    }

    #[test]
    fn test_get_edge_index_from_points() {
        let a = Point3D::new(0., 0., 0.);
        let b = Point3D::new(6., 0., 0.);
        let c = Point3D::new(0., 8., 0.);

        let t = Triangle3D::new(a, b, c).unwrap();

        assert_eq!(
            t.get_edge_index_from_segment(&Segment3D::new(a, b)),
            Some(0)
        );
        assert_eq!(
            t.get_edge_index_from_segment(&Segment3D::new(b, a)),
            Some(0)
        );
        assert_eq!(
            t.get_edge_index_from_segment(&Segment3D::new(b, c)),
            Some(1)
        );
        assert_eq!(
            t.get_edge_index_from_segment(&Segment3D::new(c, b)),
            Some(1)
        );
        assert_eq!(
            t.get_edge_index_from_segment(&Segment3D::new(c, a)),
            Some(2)
        );
        assert_eq!(
            t.get_edge_index_from_segment(&Segment3D::new(a, c)),
            Some(2)
        );

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

        let t = Triangle3D::new(a, b, c).unwrap();

        assert!(t.has_vertex(a));
        assert!(t.has_vertex(b));
        assert!(t.has_vertex(c));

        assert!(!t.has_vertex(not_there));

        let t2 = Triangle3D::new(a, c, b).unwrap();
        assert!(t.compare(&t2));

        let t3 = Triangle3D::new(a, c, not_there).unwrap();
        assert!(!t.compare(&t3));
    }
}
