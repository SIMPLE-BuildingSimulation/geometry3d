use crate::point3d::*;
use crate::segment3d::*;

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

    pub fn test_point(&self, p: Point3D, code: &mut i8) -> bool {
        /*
        Source: http://blackpawn.com/texts/pointinpoly/
        */
        // get vertices
        let vertex_a = self.v0;
        let vertex_b = self.v1;
        let vertex_c = self.v2;

        // Compute vectors
        let v0 = vertex_c - vertex_a;
        let v1 = vertex_b - vertex_a;
        let v2 = p - vertex_a;

        // Compute dot products
        let dot00 = v0 * v0;
        let dot01 = v0 * v1;
        let dot02 = v0 * v2;
        let dot11 = v1 * v1;
        let dot12 = v1 * v2;

        // Compute barycentric coordinates
        let inv_denom = 1. / (dot00 * dot11 - dot01 * dot01);
        let u = (dot11 * dot02 - dot01 * dot12) * inv_denom;
        let v = (dot00 * dot12 - dot01 * dot02) * inv_denom;
        let w = 1. - u - v;

        // Check if point is in triangle
        if u >= -f64::EPSILON && v >= -f64::EPSILON && w >= -f64::EPSILON {
            // Somewhere in the triangle
            if u <= f64::EPSILON && v <= f64::EPSILON {
                *code = 0; // vertex a
            } else if u <= f64::EPSILON && w <= f64::EPSILON {
                *code = 1; // vertex b
            } else if v <= f64::EPSILON && w <= f64::EPSILON {
                *code = 2; // vertex c
            } else if u <= f64::EPSILON {
                *code = 3; // edge AB
            } else if w <= f64::EPSILON {
                *code = 4; // edge BC
            } else if v <= f64::EPSILON {
                *code = 5; // edge AC
            } else {
                *code = 6; // inside the triangle
            }
            return true;
        }
        *code = -1;
        // return
        false
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

/***********/
/* TESTING */
/***********/

#[cfg(test)]
mod testing {
    use super::*;

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
        let mut code: i8 = 1;
        let mut in_triangle: bool;

        let a = Point3D::new(-1., 0., 0.);
        let b = Point3D::new(1., 0., 0.);
        let c = Point3D::new(0., 1., 0.);

        let triangle = Triangle3D::new(a, b, c, 1).unwrap();

        // Vertex A.
        in_triangle = triangle.test_point(a, &mut code);
        assert!(in_triangle);
        assert_eq!(code, 0);

        // Vertex B.
        in_triangle = triangle.test_point(b, &mut code);
        assert!(in_triangle);
        assert_eq!(code, 1);

        // Vertex C.
        in_triangle = triangle.test_point(c, &mut code);
        assert!(in_triangle);
        assert_eq!(code, 2);

        // Segment AB.
        let origin = Point3D::new(0., 0., 0.);
        in_triangle = triangle.test_point(origin, &mut code);
        assert!(in_triangle);
        assert_eq!(code, 3);

        // Point outside
        let point = Point3D::new(0., -1., 0.);
        in_triangle = triangle.test_point(point, &mut code);
        assert!(!in_triangle);
        assert_eq!(code, -1);
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
