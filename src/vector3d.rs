use std::fmt;

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Vector3D {
    x: f64,
    y: f64,
    z: f64,
    length: f64, // cached for speed.
}

impl fmt::Display for Vector3D {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Vector3D({:.5},{:.5},{:.5})", self.x, self.y, self.z)
    }
}

impl Vector3D {
    pub fn new(x: f64, y: f64, z: f64) -> Vector3D {
        let l2 = x * x + y * y + z * z;
        Vector3D {
            x,
            y,
            z,
            length: l2.sqrt(),
        }
    }

    pub fn x(&self) -> f64 {
        self.x
    }

    pub fn y(&self) -> f64 {
        self.y
    }

    pub fn z(&self) -> f64 {
        self.z
    }

    pub fn compare(&self, p: Vector3D) -> bool {
        (self.x - p.x).abs() < f64::EPSILON
            && (self.y - p.y).abs() < f64::EPSILON
            && (self.z - p.z).abs() < f64::EPSILON
    }

    pub fn cross(&self, v: Vector3D) -> Vector3D {
        let dx = self.y * v.z - self.z * v.y;
        let dy = self.z * v.x - self.x * v.z;
        let dz = self.x * v.y - self.y * v.x;

        Vector3D::new(dx, dy, dz)
    }

    pub fn length(&self) -> f64 {
        self.length
    }

    pub fn normalize(&mut self) {
        let l = self.length();
        self.x /= l;
        self.y /= l;
        self.z /= l;

        self.length = 1.0;
    }

    pub fn get_normalized(&self) -> Vector3D {
        let l = self.length();
        Vector3D {
            x: self.x / l,
            y: self.y / l,
            z: self.z / l,
            length: 1.0,
        }
    }

    pub fn is_zero(&self) -> bool {
        let l = self.length();
        l < 1E-10
    }

    pub fn is_parallel(&self, v: Vector3D) -> bool {
        // If any of them is zero, false
        if v.is_zero() || self.is_zero() {
            return false;
        }

        // If they are the same vector, true.
        if self.compare(v) {
            return true;
        }

        // This closure will be useful later.
        let check_other = |k_in: f64, v1: f64, v2: f64| {
            if v2.abs() > f64::EPSILON {
                // Can be divided. Check if division
                // results in K (or close to it).
                (v1 / v2 - k_in).abs() < f64::EPSILON
            } else {
                // Cannot be divided... check if v1 is
                // also tiny
                v1.abs() < f64::EPSILON
            }
        };

        // If they point in the same direction, K should be the same in all cases.
        // This value (1.0) is never really used I think, but I need to
        // initialize the variable... otherwise I get  warming.
        let k; // = 1.0 as f64;
        if v.x.abs() > f64::EPSILON {
            // Let's try anchoring at X
            k = self.x / v.x;

            // Check if Y and Z have the same K.
            check_other(k, self.y, v.y) && check_other(k, self.z, v.z)
        } else if v.y.abs() > f64::EPSILON {
            // Let's try anchoring at Y
            k = self.y / v.y;

            // return
            check_other(k, self.x, v.x) && check_other(k, self.z, v.z)
        } else {
            // Since v is not zero, then
            // v.z.abs() must be larget than tiny
            // Let's anchor at Z

            //if v.z.abs() > tiny {
            k = self.z / v.z;
            //}
            check_other(k, self.x, v.x) && check_other(k, self.y, v.y)
        }
    } // end of is_parallel

    pub fn is_same_direction(&self, v: Vector3D) -> bool {
        if !self.is_parallel(v) {
            // If they are not parallel, don't bother
            false
        } else {
            // Dot product must be positive for the
            // vectors to point in the same general
            // direction
            *self * v > 0.0
        }
    }
}

impl std::ops::Mul<Vector3D> for Vector3D {
    type Output = f64;

    fn mul(self, other: Vector3D) -> f64 {
        {
            self.x * other.x + self.y * other.y + self.z * other.z
        }
    }
}

impl std::ops::Mul<f64> for Vector3D {
    type Output = Vector3D;

    fn mul(self, s: f64) -> Vector3D {
        Vector3D::new(self.x * s, self.y * s, self.z * s)
    }
}

impl std::ops::MulAssign<f64> for Vector3D {
    fn mul_assign(&mut self, s: f64) {
        self.x *= s;
        self.y *= s;
        self.z *= s;
    }
}

impl std::ops::Div<f64> for Vector3D {
    type Output = Vector3D;

    fn div(self, s: f64) -> Vector3D {
        Vector3D::new(self.x / s, self.y / s, self.z / s)
    }
}

impl std::ops::DivAssign<f64> for Vector3D {
    fn div_assign(&mut self, s: f64) {
        self.x /= s;
        self.y /= s;
        self.z /= s;
    }
}

impl std::ops::Add<Vector3D> for Vector3D {
    type Output = Vector3D;

    fn add(self, other: Vector3D) -> Vector3D {
        Vector3D::new(self.x + other.x, self.y + other.y, self.z + other.z)
    }
}

impl std::ops::AddAssign<Vector3D> for Vector3D {
    fn add_assign(&mut self, other: Vector3D) {
        self.x += other.x;
        self.y += other.y;
        self.z += other.z;
    }
}

impl std::ops::SubAssign<Vector3D> for Vector3D {
    fn sub_assign(&mut self, other: Vector3D) {
        self.x -= other.x;
        self.y -= other.y;
        self.z -= other.z;
    }
}

impl std::ops::Neg for Vector3D {
    type Output = Vector3D;

    fn neg(self) -> Vector3D {
        Vector3D::new(-self.x, -self.y, -self.z)
    }
}

impl std::ops::Sub<Vector3D> for Vector3D {
    type Output = Vector3D;

    fn sub(self, other: Vector3D) -> Vector3D {
        Vector3D::new(self.x - other.x, self.y - other.y, self.z - other.z)
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
        let x = 1.0;
        let y = 2.0;
        let z = 3.0;

        let pt = Vector3D::new(x, y, z);
        assert_eq!(x, pt.x);
        assert_eq!(y, pt.y);
        assert_eq!(z, pt.z);

        assert_eq!(x, pt.x());
        assert_eq!(y, pt.y());
        assert_eq!(z, pt.z());
    }

    #[test]
    fn test_dot() {
        let x = Vector3D::new(1.0, 0.0, 0.0);
        let y = Vector3D::new(0.0, 1.0, 0.0);

        assert_eq!(x * y, 0.0);
        assert_eq!(y * x, 0.0);
        assert_eq!(x * x, 1.0);
        assert_eq!(y * y, 1.0);
    }

    #[test]
    fn test_scale() {
        let x = 1.2;
        let y = 5.22;
        let z = 9.123;
        let s = 4.123;

        let v = Vector3D::new(x, y, z);
        let new_v = v * s;

        assert_eq!(new_v.x, x * s);
        assert_eq!(new_v.y, y * s);
        assert_eq!(new_v.z, z * s);

        let v = Vector3D::new(x, y, z);
        let new_v = v / s;

        assert_eq!(new_v.x, x / s);
        assert_eq!(new_v.y, y / s);
        assert_eq!(new_v.z, z / s);

        let x = 1.2;
        let y = 5.22;
        let z = 9.123;
        let s = 4.123;

        let mut v = Vector3D::new(x, y, z);
        v *= s;

        assert_eq!(v.x, x * s);
        assert_eq!(v.y, y * s);
        assert_eq!(v.z, z * s);

        let mut v = Vector3D::new(x, y, z);
        v /= s;

        assert_eq!(v.x, x / s);
        assert_eq!(v.y, y / s);
        assert_eq!(v.z, z / s);
    }

    #[test]
    fn test_add() {
        let x = 1.2;
        let y = 5.22;
        let z = 9.123;
        let s = 1.212312;

        let mut v = Vector3D::new(x, y, z);
        let d = Vector3D::new(s * x, s * y, s * z);

        let v2 = v + d;

        assert!(v2.x - x * (s + 1.0) < 1E-10);
        assert!(v2.y - y * (s + 1.0) < 1E-10);
        assert!(v2.z - z * (s + 1.0) < 1E-10);

        v += d;
        assert!(v.x - x * (s + 1.0) < 1E-10);
        assert!(v.y - y * (s + 1.0) < 1E-10);
        assert!(v.z - z * (s + 1.0) < 1E-10);
    }

    #[test]
    fn test_sub() {
        let x = 1.2;
        let y = 5.22;
        let z = 9.123;
        let s = 1.212312;

        let mut v = Vector3D::new(x, y, z);
        let d = Vector3D::new(-s * x, -s * y, -s * z);

        let v2 = v - d;

        assert!(v2.x - x * (s + 1.0) < 1E-10);
        assert!(v2.y - y * (s + 1.0) < 1E-10);
        assert!(v2.z - z * (s + 1.0) < 1E-10);

        v -= d;
        assert!(v.x - x * (s + 1.0) < 1E-10);
        assert!(v.y - y * (s + 1.0) < 1E-10);
        assert!(v.z - z * (s + 1.0) < 1E-10);
    }

    #[test]
    fn test_compare() {
        let x = 123.1;
        let y = 543.1;
        let z = 9123.2;
        let d = 0.1;

        let p = Vector3D::new(x, y, z);
        let p2 = Vector3D::new(x, y, z);
        assert!(p.compare(p2));

        let p2 = Vector3D::new(x + d, y, z);
        assert!(!p.compare(p2));

        let p2 = Vector3D::new(x, y, z - d);
        assert!(!p.compare(p2));

        let p2 = Vector3D::new(x, -y, z);
        assert!(!p.compare(p2));
    }

    #[test]
    fn test_cross() {
        let zero = Vector3D::new(0.0, 0.0, 0.0);
        let x = Vector3D::new(1.0, 0.0, 0.0);
        let y = Vector3D::new(0.0, 1.0, 0.0);
        let z = Vector3D::new(0.0, 0.0, 1.0);

        assert!(x.cross(x).compare(zero));
        assert!(x.cross(y).compare(z));
        assert!(y.cross(z).compare(x));

        let a = Vector3D::new(2., 3., 4.);
        let b = Vector3D::new(5., 6., 7.);
        let c = a.cross(b);

        assert_eq!(c.x, -3.);
        assert_eq!(c.y, 6.);
        assert_eq!(c.z, -3.);
    }

    #[test]
    fn test_length() {
        let a = 5.0;
        let x = Vector3D::new(a, a, a);

        let l = x.length();
        let delta = l - (3.0 as f64).sqrt() * a;
        assert!(delta.abs() < 1E-10);
    }

    #[test]
    fn test_normalize() {
        let mut v = Vector3D::new(12.1, 54.1, 765.1);
        let l = v.length();

        let v_norm_ref = v / l;

        let v_norm = v.get_normalized();
        assert!(v_norm.compare(v_norm_ref));

        v.normalize();
        assert!(v.compare(v_norm_ref));
    }

    #[test]
    fn test_is_zero() {
        let zero = Vector3D::new(0.0, 0.0, 0.0);

        assert!(zero.is_zero());

        let nearly_zero = Vector3D::new(1E-6, 0.0, 0.0);
        assert!(!nearly_zero.is_zero());
    }

    #[test]
    fn test_is_parallel() {
        let v1 = Vector3D::new(123.1, 5.6, 99.12);
        let mut v2 = v1 * 3.14;

        // V2 is v1 scaled... same dir and
        // parallel
        assert!(v1.is_parallel(v2));
        assert!(v1.is_same_direction(v2));

        // V2 goes in the opposite dir...
        // Paralel, but no in same dir
        v2 /= -1.0;
        assert!(v1.is_parallel(v2));
        assert!(!v1.is_same_direction(v2));

        // Are not parallel... none of them
        //are true
        let d = Vector3D::new(0.1, 0.0, 0.0);
        v2 += d;
        assert!(!v1.is_parallel(v2));
        assert!(!v1.is_same_direction(v2));
    }

    #[test]
    fn test_negation() {
        let x = 1.2;
        let y = 5.22;
        let z = 9.123;
        let v1 = Vector3D::new(x, y, z);
        let v2 = Vector3D::new(-x, -y, -z);

        assert_eq!((-v1).x, -x);
        assert_eq!((-v1).y, -y);
        assert_eq!((-v1).z, -z);

        assert!(v1.compare(-v2));
    }
}
