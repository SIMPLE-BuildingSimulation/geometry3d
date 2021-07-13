use crate::point3d::*;

/// An imaginary line starting at one [`Point3D`] and ending
/// on another [`Point3D`].
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Segment3D {
    pub start: Point3D,
    pub end: Point3D,
    pub length: f64,
}

impl Segment3D {
    /// Creates a new [`Segment3D`]
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

    pub fn length(&self) -> f64 {
        self.length
    }

    /// Compares the Start and End [`Point3D`] of two [`Segment3D`]. This will
    /// return `true` if the start and end are equal(ish), even if one of the
    /// segments is reversed.
    pub fn compare(&self, other: &Segment3D) -> bool {
        // Start and end might be in different order.
        (self.start.compare(other.start) && self.end.compare(other.end))
            || (self.end.compare(other.start) && self.start.compare(other.end))
    }

    /// Checks if a [`Segment3D`] contains another [`Point3D`].
    pub fn contains_point(&self, point: Point3D) -> Result<bool, String> {
        if !point.is_collinear(self.start, self.end)? {
            Ok(false)
        } else {
            let ab = self.end - self.start;
            let ap = point - self.start;
            let ret: f64;
            if ab.x().abs() > f64::EPSILON {
                ret = ap.x() / ab.x();
            } else if ab.y().abs() > f64::EPSILON {
                ret = ap.y() / ab.y();
            } else if ab.z().abs() > f64::EPSILON {
                ret = ap.z() / ab.z();
            } else {
                return Err("Checking if Segment3D of length 0 contains a Point3D".to_string());
            }
            Ok((0. ..=1.).contains(&ret))
        }
    }

    /// Checks if a [`Segment3D`] contains another [`Segment3D`].
    pub fn contains(&self, input: &Segment3D) -> Result<bool, String> {
        const TINY: f64 = 100. * f64::EPSILON;
        if self.length() < TINY {
            let msg = "Trying to check whether a segment is contained in a zero-length segment"
                .to_string();
            return Err(msg);
        }

        let a1 = self.start();
        let b1 = self.end();
        let a2 = input.start();
        let b2 = input.end();

        // If the four points are not collinear, then no
        if !a1.is_collinear(b1, a2)? || !a1.is_collinear(b1, b2)? {
            return Ok(false);
        }

        // they are in the same line, so now just
        // calculate the interpolation
        let a1b1 = b1 - a1;

        let alpha: f64;
        let beta: f64;
        if a1b1.x().abs() > TINY {
            alpha = (a2.x - a1.x) / a1b1.x();
            beta = (a2.x - a1.x) / a1b1.x();
        } else if a1b1.y().abs() > TINY {
            alpha = (a2.y - a1.y) / a1b1.y();
            beta = (a2.y - a1.y) / a1b1.y();
        } else if a1b1.z().abs() > TINY {
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

    /// Checks where is it that two [`Segment3D`] intersect, returning the
    /// fraction of the caller [`Segment3D`] and the input [`Segment3D`]
    /// in which the two segments would intersect. Returns None if the caller and
    /// the input segments are in the same direction or if
    /// they dwell in different planes (i.e., they do not intercept)
    ///
    /// # Examples
    ///
    /// ```
    /// use geometry3d::point3d::Point3D;
    /// use geometry3d::segment3d::Segment3D;
    /// 
    /// let vertical = Segment3D::new(Point3D::new(0., 0., -1.), Point3D::new(0., 0., 1.));
    /// let horizontal = Segment3D::new(Point3D::new(-1., 0., 0.), Point3D::new(1., 0., 0.));
    /// assert_eq!(horizontal.get_intersection_pt(&vertical), Some((0.5, 0.5)));
    ///
    /// let vertical = Segment3D::new(Point3D::new(0., 0., -1.), Point3D::new(0., 0., 1.));
    /// let horizontal = Segment3D::new(Point3D::new(0., 0., 0.), Point3D::new(1., 0., 0.));
    /// assert_eq!(horizontal.get_intersection_pt(&vertical), Some((0.0, 0.5)));
    ///
    /// let vertical = Segment3D::new(Point3D::new(0., 0., -1.), Point3D::new(0., 0., 1.));
    /// let horizontal = Segment3D::new(Point3D::new(0.5, 0., 0.), Point3D::new(1., 0., 0.));
    /// assert_eq!(horizontal.get_intersection_pt(&vertical), Some((-1., 0.5)));
    ///
    /// ```
    pub fn get_intersection_pt(&self, input: &Segment3D) -> Option<(f64, f64)> {
        let a = self.end - self.start;
        let b = input.end() - input.start();

        if a.is_same_direction(b) {
            return None;
        }

        // check if coplanar
        let normal = a.cross(b);
        let delta = self.start() - input.start();

        if delta.cross(normal).is_zero() {
            return None;
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
            return None;
        }
        Some((t_a, t_b))
    }

    /// Checks if two [`Segment3D`] intersect each other. This returns `false` if
    /// one of the [`Segment3D`] barely touches the other one.
    pub fn intersect(&self, input: &Segment3D, output: &mut Point3D) -> bool {
        match self.get_intersection_pt(input) {
            Some((t_a, t_b)) => {
                let a = self.end - self.start;

                if (f64::EPSILON..1.).contains(&t_a) && (f64::EPSILON..1.).contains(&t_b) {
                    *output = self.start + a * t_a;
                    true
                } else {
                    false
                }
            }
            None => false,
        }
    } // end of intersect

    /// Checks if two [`Segment3D`] intersect each other. This returns `true` if
    /// one of the [`Segment3D`] barely touches the other one.
    pub fn touches(&self, input: &Segment3D, output: &mut Point3D) -> bool {
        match self.get_intersection_pt(input) {
            Some((t_a, t_b)) => {
                let a = self.end - self.start;

                if (0. ..=1.).contains(&t_a) && (0. ..=1.).contains(&t_b) {
                    *output = self.start + a * t_a;
                    true
                } else {
                    false
                }
            }
            None => false,
        }
    } // end of touches

    /// Gets the midpoint of the a [`Segment3D`]
    pub fn midpoint(&self) -> Point3D {
        (self.start + self.end) * 0.5
    }
}

/***********/
/* TESTING */
/***********/

#[cfg(test)]
mod testing {
    use super::*;

    #[test]
    fn test_intersection() {
        let vertical = Segment3D::new(Point3D::new(0., 0., -1.), Point3D::new(0., 0., 1.));
        let horizontal = Segment3D::new(Point3D::new(-1., 0., 0.), Point3D::new(1., 0., 0.));
        assert_eq!(horizontal.get_intersection_pt(&vertical), Some((0.5, 0.5)));

        let vertical = Segment3D::new(Point3D::new(0., 0., -1.), Point3D::new(0., 0., 1.));
        let horizontal = Segment3D::new(Point3D::new(0., 0., 0.), Point3D::new(1., 0., 0.));
        assert_eq!(horizontal.get_intersection_pt(&vertical), Some((0.0, 0.5)));

        let vertical = Segment3D::new(Point3D::new(0., 0., -1.), Point3D::new(0., 0., 1.));
        let horizontal = Segment3D::new(Point3D::new(0.5, 0., 0.), Point3D::new(1., 0., 0.));
        assert_eq!(horizontal.get_intersection_pt(&vertical), Some((-1., 0.5)));
    }

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
    fn test_intersect_touches() {
        let origin = Point3D::new(0., 0., 0.);

        let p_m1_0_0 = Point3D::new(-1.0, 0.0, 0.0);
        let p_1_0_0 = Point3D::new(1.0, 0.0, 0.0);
        let p_m1_0_1 = Point3D::new(-1.0, 0.0, 1.0);
        let p_1_0_1 = Point3D::new(1.0, 0.0, 1.0);

        let p_0_m1_0 = Point3D::new(0., -1., 0.);
        let p_0_1_0 = Point3D::new(0., 1., 0.);

        let p_0_0_m1 = Point3D::new(0., 0., -1.);
        let p_0_0_1 = Point3D::new(0., 0., 1.);
        let p_1_0_m1 = Point3D::new(1., 0., -1.0);

        let x_axis = Segment3D::new(p_m1_0_0, p_1_0_0);
        let y_axis = Segment3D::new(p_0_m1_0, p_0_1_0);
        let z_axis = Segment3D::new(p_0_0_m1, p_0_0_1);
        let offset_x = Segment3D::new(p_m1_0_1, p_1_0_1);
        let offset_z = Segment3D::new(p_1_0_m1, p_1_0_1);
        let semi_z = Segment3D::new(origin, p_0_0_1);

        // Start tests
        let mut intersection = Point3D::new(-1.0, -1.0, -1.0);
        let mut do_intersect: bool;
        let mut do_touch: bool;

        // X Y
        do_intersect = x_axis.intersect(&y_axis, &mut intersection);
        assert!(do_intersect);
        assert!(intersection.compare(origin));

        do_touch = x_axis.touches(&y_axis, &mut intersection);
        assert!(do_touch);
        assert!(intersection.compare(origin));

        // X Z
        do_intersect = x_axis.intersect(&z_axis, &mut intersection);
        assert!(do_intersect);
        assert!(intersection.compare(origin));

        do_touch = x_axis.touches(&z_axis, &mut intersection);
        assert!(do_touch);
        assert!(intersection.compare(origin));

        // Y Z
        do_intersect = z_axis.intersect(&y_axis, &mut intersection);
        assert!(do_intersect);
        assert!(intersection.compare(origin));

        do_touch = z_axis.touches(&y_axis, &mut intersection);
        assert!(do_touch);
        assert!(intersection.compare(origin));

        // X OFFSET-X
        do_intersect = offset_x.intersect(&x_axis, &mut intersection);
        assert!(!do_intersect);

        do_touch = offset_x.touches(&x_axis, &mut intersection);
        assert!(!do_touch);

        // Z OFFSET-X
        do_intersect = offset_x.intersect(&z_axis, &mut intersection);
        assert!(!do_intersect);
        do_touch = offset_x.touches(&z_axis, &mut intersection);
        assert!(do_touch);
        assert!(intersection.compare(Point3D::new(0., 0., 1.)));

        // OFFSET-Z OFFSET-X
        do_intersect = offset_x.intersect(&offset_z, &mut intersection);
        assert!(!do_intersect);
        do_touch = offset_x.touches(&offset_z, &mut intersection);
        assert!(do_touch);
        assert!(intersection.compare(Point3D::new(1., 0., 1.)));

        // Semi-z / X
        do_intersect = x_axis.intersect(&semi_z, &mut intersection);
        assert!(!do_intersect);
        do_touch = x_axis.touches(&semi_z, &mut intersection);
        assert!(do_touch);
        assert!(intersection.compare(origin));

        // colinear
        do_intersect = x_axis.intersect(&x_axis, &mut intersection);
        assert!(!do_intersect);

        do_touch = x_axis.touches(&x_axis, &mut intersection);
        assert!(!do_touch);
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
