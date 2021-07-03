use crate::point3d::*;

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Segment3D {
    start: Point3D,
    end: Point3D,
    length: f64,
}

impl Segment3D {
    pub fn new(a: Point3D, b: Point3D) -> Segment3D {
        let l = a.distance(b);
        Segment3D {
            start: a,
            end: b,
            length: l,
        }
    }

    pub fn start(&self) -> Point3D {
        self.start
    }

    pub fn end(&self) -> Point3D {
        self.end
    }

    pub fn compare(&self, other: &Segment3D) -> bool {
        // Start and end might be in different order.
        (self.start.compare(other.start) && self.end.compare(other.end))
            || (self.end.compare(other.start) && self.start.compare(other.end))
    }

    pub fn length(&self) -> f64 {
        self.length
    }

    pub fn contains(&self, input: &Segment3D) -> Result<bool, String> {
        let tiny = 1E-9;
        if self.length() < tiny {
            let msg = "Trying to check whether a segment is contained in a zero-length segment"
                .to_string();
            return Err(msg);
        }

        let a1 = self.start();
        let b1 = self.end();
        let a2 = input.start();
        let b2 = input.end();

        // If the four points are not collinear, then no
        if !a1.is_collinear(b1, a2).unwrap() || !a1.is_collinear(b1, b2).unwrap() {
            return Ok(false);
        }

        // they are in the same line, so now just
        // calculate the interpolation
        let a1b1 = b1 - a1;

        let alpha: f64;
        let beta: f64;
        if a1b1.x().abs() > tiny {
            alpha = (a2.x - a1.x) / a1b1.x();
            beta = (a2.x - a1.x) / a1b1.x();
        } else if a1b1.y().abs() > tiny {
            alpha = (a2.y - a1.y) / a1b1.y();
            beta = (a2.y - a1.y) / a1b1.y();
        } else if a1b1.z().abs() > tiny {
            alpha = (a2.z - a1.z) / a1b1.z();
            beta = (a2.z - a1.z) / a1b1.z();
        } else {
            // We should never get here.
            let msg = "Trying to check whether a segment is contained in a zero-length segment"
                .to_string();
            return Err(msg);
        }

        Ok((0. ..=1.).contains(&alpha) && (0. ..=1.).contains(&beta))
    }

    pub fn intersect(&self, input: &Segment3D, output: &mut Point3D) -> bool {
        let a = self.end - self.start;
        let b = input.end() - input.start();

        if a.is_same_direction(b) {
            return false;
        }

        // check if coplanar
        let normal = a.cross(b);
        let delta = self.start() - input.start();

        if delta.cross(normal).is_zero() {
            return false;
        }

        // They are coplanar... check for intersection
        let t_a: f64;
        let t_b: f64;
        let det: f64;

        // Check for intersection.
        if normal.z().abs() > f64::EPSILON {
            det = a.y() * b.x() - a.x() * b.y();

            t_a = (b.y() * delta.x() - b.x() * delta.y()) / det;
            t_b = (a.y() * delta.x() - a.x() * delta.y()) / det;
        } else if normal.x().abs() > f64::EPSILON {
            det = a.y() * b.z() - a.z() * b.y();
            t_a = (b.y() * delta.z() - b.z() * delta.y()) / det;
            t_b = (a.y() * delta.z() - a.z() * delta.y()) / det;
        } else if normal.y().abs() > f64::EPSILON {
            det = a.x() * b.z() - a.z() * b.x();
            t_a = (b.x() * delta.z() - b.z() * delta.x()) / det;
            t_b = (a.x() * delta.z() - a.z() * delta.x()) / det;
        } else {
            return false;
        }

        // if (0. ..=1.).contains(&t_a) && (0. ..=1.).contains(&t_b) {
        if (f64::EPSILON ..1.).contains(&t_a) && (f64::EPSILON ..1.).contains(&t_b) {
            let pt_x = self.start.x + t_a * a.x();
            let pt_y = self.start.y + t_a * a.y();
            let pt_z = self.start.z + t_a * a.z();

            *output = Point3D::new(pt_x, pt_y, pt_z);

            return true;
        }
        // return
        false
    } // end of intersect

    pub fn midpoint(&self) -> Point3D {
        let dx = self.start.x + self.end.x;
        let dy = self.start.y + self.end.y;
        let dz = self.start.z + self.end.z;
        Point3D::new(dx / 2.0, dy / 2.0, dz / 2.0)
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
        let start = Point3D::new(1., 2., 3.);
        let end = Point3D::new(4., 5., 6.);

        let s = Segment3D::new(start, end);

        assert_eq!(s.start.x, 1.);
        assert_eq!(s.start.y, 2.);
        assert_eq!(s.start.z, 3.);

        assert_eq!(s.end.x, 4.);
        assert_eq!(s.end.y, 5.);
        assert_eq!(s.end.z, 6.);

        assert_eq!(s.start().x, 1.);
        assert_eq!(s.start().y, 2.);
        assert_eq!(s.start().z, 3.);

        assert_eq!(s.end().x, 4.);
        assert_eq!(s.end().y, 5.);
        assert_eq!(s.end().z, 6.);
    }

    #[test]
    fn test_compare() {
        let start = Point3D::new(1., 2., 3.);
        let end = Point3D::new(4., 5., 6.);

        let s = Segment3D::new(start, end);

        assert!(s.compare(&s));

        let end2 = Point3D::new(4.2, 5., 6.);
        let s2 = Segment3D::new(start, end2);
        assert!(!s.compare(&s2));
        assert!(!s2.compare(&s));
    }

    #[test]
    fn test_intersect() {
        let origin = Point3D::new(0., 0., 0.);

        let ax = Point3D::new(-1.0, 0.0, 0.0);
        let bx = Point3D::new(1.0, 0.0, 0.0);
        let offset_ax = Point3D::new(-1.0, 0.0, 1.0);
        let offset_bx = Point3D::new(1.0, 0.0, 1.0);

        let ay = Point3D::new(0., -1., 0.);
        let by = Point3D::new(0., 1., 0.);

        let az = Point3D::new(0., 0., -1.);
        let bz = Point3D::new(0., 0., 1.);
        let offset_az = Point3D::new(1., 0., -1.0);
        let offset_bz = Point3D::new(1., 0., 1.0);

        let x_axis = Segment3D::new(ax, bx);
        let y_axis = Segment3D::new(ay, by);
        let z_axis = Segment3D::new(az, bz);
        let offset_x = Segment3D::new(offset_ax, offset_bx);
        let offset_z = Segment3D::new(offset_az, offset_bz);
        let semi_z = Segment3D::new(origin, bz);

        // Start tests
        let mut intersection = Point3D::new(-1.0, -1.0, -1.0);
        let mut do_intersect: bool;

        // X Y
        do_intersect = x_axis.intersect(&y_axis, &mut intersection);
        assert!(do_intersect);
        assert!(intersection.compare(origin));

        // X Z
        do_intersect = x_axis.intersect(&z_axis, &mut intersection);
        assert!(do_intersect);
        assert!(intersection.compare(origin));

        // Y Z
        do_intersect = z_axis.intersect(&y_axis, &mut intersection);
        assert!(do_intersect);
        assert!(intersection.compare(origin));

        // X OFFSET-X
        do_intersect = offset_x.intersect(&x_axis, &mut intersection);
        assert!(!do_intersect);

        // Z OFFSET-X
        do_intersect = offset_x.intersect(&z_axis, &mut intersection);
        assert!(do_intersect);
        assert!(intersection.compare(Point3D::new(0., 0., 1.)));

        // OFFSET-Z OFFSET-X
        do_intersect = offset_x.intersect(&offset_z, &mut intersection);
        assert!(do_intersect);
        //assert!(intersection.compare(Point3D::new(1., 0., 1.)));

        // Semi-z / X
        do_intersect = x_axis.intersect(&semi_z, &mut intersection);
        assert!(do_intersect);
        assert!(intersection.compare(origin));

        // colinear
        do_intersect = x_axis.intersect(&x_axis, &mut intersection);
        assert!(!do_intersect);
    }

    #[test]
    fn test_midpoint() {
        let x = 1.2312 as f64;
        let y = 1123.2312 as f64;
        let z = 31.2312 as f64;

        let o = Point3D::new(0., 0., 0.);
        let end = Point3D::new(x, y, z);
        let s = Segment3D::new(o, end);

        let midpoint = s.midpoint();
        assert_eq!(midpoint.x, x / 2.);
        assert_eq!(midpoint.y, y / 2.);
        assert_eq!(midpoint.z, z / 2.);
    }

    #[test]
    fn test_contains() {
        // RANDOM AXIS
        let a = Point3D::new(5.0, -1.0, 32.0);
        let b = Point3D::new(1.2, 6.4, -2.);

        let main_s = Segment3D::new(a, b);

        let check = |alpha: f64, beta: f64| -> bool {
            let a2 = a + (b - a) * alpha;
            let b2 = a + (b - a) * beta;
            let s = Segment3D::new(a2, b2);

            return main_s.contains(&s).unwrap();
        };

        assert!(check(0.5, 0.55));
        assert!(check(0.9, 0.55));
        assert!(check(0.0, 0.35));
        assert!(check(1.0, 0.15));

        assert!(!check(-0.2, 0.55));
        assert!(!check(1.6, 1.55));

        // Z Axis
        let a = Point3D::new(0., 0., 32.0);
        let b = Point3D::new(0., 0., -2.);
        let main_s = Segment3D::new(a, b);

        let check = |alpha: f64, beta: f64| -> bool {
            let a2 = a + (b - a) * alpha;
            let b2 = a + (b - a) * beta;
            let s = Segment3D::new(a2, b2);

            return main_s.contains(&s).unwrap();
        };

        assert!(check(0.5, 0.55));
        assert!(check(0.9, 0.55));
        assert!(check(0.0, 0.35));
        assert!(check(1.0, 0.15));

        assert!(!check(-0.2, 0.55));
        assert!(!check(1.6, 1.55));

        // X Axis
        let a = Point3D::new(5.0, 0., 0.);
        let b = Point3D::new(1.2, 0., 0.);

        let main_s = Segment3D::new(a, b);

        let check = |alpha: f64, beta: f64| -> bool {
            let a2 = a + (b - a) * alpha;
            let b2 = a + (b - a) * beta;
            let s = Segment3D::new(a2, b2);

            return main_s.contains(&s).unwrap();
        };

        assert!(check(0.5, 0.55));
        assert!(check(0.9, 0.55));
        assert!(check(0.0, 0.35));
        assert!(check(1.0, 0.15));

        assert!(!check(-0.2, 0.55));
        assert!(!check(1.6, 1.55));

        // Y AXIS

        let a = Point3D::new(0., 1., 0.);
        let b = Point3D::new(0., 10., 0.);

        let main_s = Segment3D::new(a, b);

        let check = |alpha: f64, beta: f64| -> bool {
            let a2 = a + (b - a) * alpha;
            let b2 = a + (b - a) * beta;
            let s = Segment3D::new(a2, b2);

            return main_s.contains(&s).unwrap();
        };

        assert!(check(0.5, 0.55));
        assert!(check(0.9, 0.55));
        assert!(check(0.0, 0.35));
        assert!(check(1.0, 0.15));

        assert!(!check(-0.2, 0.55));
        assert!(!check(1.6, 1.55));
    }
}
