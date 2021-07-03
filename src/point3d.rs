use crate::vector3d::*;

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Point3D {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Point3D {
    pub fn new(x: f64, y: f64, z: f64) -> Point3D {
        Point3D { x, y, z }
    }

    pub fn as_vector3d(&self)->Vector3D{
        Vector3D::new(self.x,self.y,self.z)
    }

    pub fn squared_distance(&self, point: Point3D) -> f64 {
        let dx = (self.x - point.x) * (self.x - point.x);
        let dy = (self.y - point.y) * (self.y - point.y);
        let dz = (self.z - point.z) * (self.z - point.z);
        dx + dy + dz
    }

    pub fn distance(&self, point: Point3D) -> f64 {
        let d2 = self.squared_distance(point);
        d2.sqrt()
    }    

    pub fn compare(&self, p: Point3D) -> bool {
        let eps = 1E-10;
        (self.x - p.x).abs() < eps && (self.y - p.y).abs() < eps && (self.z - p.z).abs() < eps
    }

    pub fn is_collinear(self, b: Point3D, c: Point3D) -> Result<bool, String> {
        // check that they are not ALL the same
        if self.compare(b) && self.compare(c) {
            let msg = "Trying to test collinearity with three equal Point3D".to_string();
            return Err(msg);
        }

        // Check if two of them are the same
        if self.compare(b) || self.compare(c) || b.compare(c) {
            return Ok(true);
        }

        let ab = b - self;
        let bc = c - b;

        Ok(ab.cross(bc).is_zero())
    }
}

impl std::ops::Add<Vector3D> for Point3D {
    type Output = Point3D;

    fn add(self, other: Vector3D) -> Point3D {
        Point3D {
            x: self.x + other.x(),
            y: self.y + other.y(),
            z: self.z + other.z(),
        }
    }
}

impl std::ops::Sub<Point3D> for Point3D {
    type Output = Vector3D;

    fn sub(self, other: Point3D) -> Vector3D {
        Vector3D::new(self.x - other.x, self.y - other.y, self.z - other.z)
    }
}

impl std::ops::Sub<&Vector3D> for &Point3D {
    type Output = Point3D;

    fn sub(self, other: &Vector3D) -> Point3D {
        Point3D {
            x: self.x - other.x(),
            y: self.y - other.y(),
            z: self.z - other.z(),
        }
    }
}

impl std::ops::Mul<Vector3D> for Point3D {
    type Output = f64;

    fn mul(self, other: Vector3D) -> f64 {
        {
            self.x * other.x() + self.y * other.y() + self.z * other.z()
        }
    }
}

impl std::ops::Mul<Point3D> for Point3D {
    type Output = f64;

    fn mul(self, other: Point3D) -> f64 {
        {
            self.x * other.x + self.y * other.y + self.z * other.z
        }
    }
}

/*********/
/* TESTS */
/*********/

#[cfg(test)]
mod testing {
    use super::*;

    #[test]
    fn test_new() {
        let x = 1.0;
        let y = 2.0;
        let z = 3.0;

        let pt = Point3D::new(x, y, z);
        assert_eq!(x, pt.x);
        assert_eq!(y, pt.y);
        assert_eq!(z, pt.z);

        assert_eq!(x, pt.x);
        assert_eq!(y, pt.y);
        assert_eq!(z, pt.z);
    }

    #[test]
    fn test_squared_distance() {
        // Difference in Z
        let a = Point3D::new(0.0, 0.0, 0.0);
        let b = Point3D::new(0.0, 0.0, 2.0);
        assert_eq!(4.0, a.squared_distance(b));

        let a = Point3D::new(0.0, 0.0, 2.0);
        let b = Point3D::new(0.0, 0.0, 0.0);
        assert_eq!(4.0, a.squared_distance(b));

        // Difference in Y
        let a = Point3D::new(0.0, 2.0, 0.0);
        let b = Point3D::new(0.0, 0.0, 0.0);
        assert_eq!(4.0, a.squared_distance(b));

        let a = Point3D::new(0.0, 0.0, 0.0);
        let b = Point3D::new(0.0, 2.0, 0.0);
        assert_eq!(4.0, a.squared_distance(b));

        // Difference in X
        let a = Point3D::new(0.0, 0.0, 0.0);
        let b = Point3D::new(2.0, 0.0, 0.0);
        assert_eq!(4.0, a.squared_distance(b));

        let a = Point3D::new(2.0, 0.0, 0.0);
        let b = Point3D::new(0.0, 0.0, 0.0);
        assert_eq!(4.0, a.squared_distance(b));

        // Difference in X and Z
        let a = Point3D::new(0.0, 0.0, 0.0);
        let b = Point3D::new(3.0, 0.0, 4.0);
        assert_eq!(25.0, a.squared_distance(b));

        let a = Point3D::new(3.0, 0.0, 4.0);
        let b = Point3D::new(0.0, 0.0, 0.0);
        assert_eq!(25.0, a.squared_distance(b));
    }

    #[test]
    fn test_mul(){
        let x = 23.;
        let y = 59.;
        let z = -0.23;

        let pt = Point3D::new(x,y,z);
        let other_pt = Point3D::new(z,x,y);
        let other_vec = Vector3D::new(z,x,y);

        assert_eq!(pt*other_pt, x*z+ y*x+ z*y);
        assert_eq!(pt*other_vec, x*z+ y*x+ z*y);
    }

    

    #[test]
    fn test_distance() {
        // Difference in Z
        let a = Point3D::new(0.0, 0.0, 0.0);
        let b = Point3D::new(0.0, 0.0, 2.0);
        assert_eq!(2.0, a.distance(b));

        let a = Point3D::new(0.0, 0.0, 2.0);
        let b = Point3D::new(0.0, 0.0, 0.0);
        assert_eq!(2.0, a.distance(b));

        // Difference in Y
        let a = Point3D::new(0.0, 2.0, 0.0);
        let b = Point3D::new(0.0, 0.0, 0.0);
        assert_eq!(2.0, a.distance(b));

        let a = Point3D::new(0.0, 0.0, 0.0);
        let b = Point3D::new(0.0, 2.0, 0.0);
        assert_eq!(2.0, a.distance(b));

        // Difference in X
        let a = Point3D::new(0.0, 0.0, 0.0);
        let b = Point3D::new(2.0, 0.0, 0.0);
        assert_eq!(2.0, a.distance(b));

        let a = Point3D::new(2.0, 0.0, 0.0);
        let b = Point3D::new(0.0, 0.0, 0.0);
        assert_eq!(2.0, a.distance(b));

        // Difference in X and Z
        let a = Point3D::new(0.0, 0.0, 0.0);
        let b = Point3D::new(3.0, 0.0, 4.0);
        assert_eq!(5.0, a.distance(b));

        let a = Point3D::new(3.0, 0.0, 4.0);
        let b = Point3D::new(0.0, 0.0, 0.0);
        assert_eq!(5.0, a.distance(b));
    }

    #[test]
    fn test_add() {
        let x = 1.2;
        let y = 5.22;
        let z = 9.123;
        let ini = Point3D::new(0.0, 0.0, 0.0);
        let v = Vector3D::new(x, y, z);
        let fin = ini + v;
        assert_eq!(x, fin.x);
        assert_eq!(y, fin.y);
        assert_eq!(z, fin.z);

        let fin = fin + v;
        assert_eq!(2.0 * x, fin.x);
        assert_eq!(2.0 * y, fin.y);
        assert_eq!(2.0 * z, fin.z);

        // Go backwards now
        let v = Vector3D::new(-x, -y, -z);
        let fin = fin + v;
        assert_eq!(x, fin.x);
        assert_eq!(y, fin.y);
        assert_eq!(z, fin.z);

        let fin = fin + v;
        assert_eq!(0.0, fin.x);
        assert_eq!(0.0, fin.y);
        assert_eq!(0.0, fin.z);
    }

    #[test]
    fn test_sub_vec() {
        let x = 1.2;
        let y = 5.22;
        let z = 9.123;
        let ini = Point3D::new(2.0 * x, 2.0 * y, 2.0 * z);
        let v = Vector3D::new(x, y, z);
        let fin = &ini - &v;

        assert_eq!(x, fin.x);
        assert_eq!(y, fin.y);
        assert_eq!(z, fin.z);

        let fin = &fin - &v;
        assert_eq!(0.0 * x, fin.x);
        assert_eq!(0.0 * y, fin.y);
        assert_eq!(0.0 * z, fin.z);

        let v = Vector3D::new(-x, -y, -z);
        let fin = &fin - &v;
        assert_eq!(x, fin.x);
        assert_eq!(y, fin.y);
        assert_eq!(z, fin.z);

        let fin = &fin - &v;
        assert_eq!(2.0 * x, fin.x);
        assert_eq!(2.0 * y, fin.y);
        assert_eq!(2.0 * z, fin.z);
    }

    

    #[test]
    fn test_sub_point() {
        let x = 1.2;
        let y = 5.22;
        let z = 9.123;
        let ini = Point3D::new(x, y, z);
        let fin = Point3D::new(2.0 * x, 2.0 * y, 2.0 * z);
        let delta = fin - ini;
        assert_eq!(delta.x(), x);
        assert_eq!(delta.y(), y);
        assert_eq!(delta.z(), z);

        let ini = Point3D::new(x, y, z);
        let fin = Point3D::new(0.0 * x, 0.0 * y, 0.0 * z);
        let delta = fin - ini;
        assert_eq!(delta.x(), -x);
        assert_eq!(delta.y(), -y);
        assert_eq!(delta.z(), -z);

        let ini = Point3D::new(0.0, 0.0, 0.0);
        let fin = Point3D::new(2.0 * x, 2.0 * y, 2.0 * z);
        let delta = fin - ini;
        assert_eq!(delta.x(), 2.0 * x);
        assert_eq!(delta.y(), 2.0 * y);
        assert_eq!(delta.z(), 2.0 * z);
    }

    #[test]
    fn test_as_vector3d() {
        let x = 123.1;
        let y = 543.1;
        let z = 9123.2;

        let p = Point3D::new(x, y, z);
        let v = p.as_vector3d();

        assert_eq!(x, v.x());
        assert_eq!(y, v.y());
        assert_eq!(z, v.z());
    }

    #[test]
    fn test_compare() {
        let x = 123.1;
        let y = 543.1;
        let z = 9123.2;
        let d = 0.1;

        let p = Point3D::new(x, y, z);
        let p2 = Point3D::new(x, y, z);
        assert!(p.compare(p2));

        let p2 = Point3D::new(x + d, y, z);
        assert!(!p.compare(p2));

        let p2 = Point3D::new(x, y, z - d);
        assert!(!p.compare(p2));

        let p2 = Point3D::new(x, -y, z);
        assert!(!p.compare(p2));
    }

    #[test]
    fn test_is_collinear() {
        let a = Point3D::new(0., 0., 0.);
        let b = Point3D::new(1., 0., 0.);
        let c = Point3D::new(3., 0., 0.);

        let d = Point3D::new(1., 2., 4.);

        assert!(a.is_collinear(b, c).unwrap());
        assert!(a.is_collinear(a, c).unwrap());
        assert!(b.is_collinear(a, c).unwrap());
        assert!(b.is_collinear(b, c).unwrap());
        assert!(c.is_collinear(b, a).unwrap());
        assert!(c.is_collinear(c, a).unwrap());

        assert!(!a.is_collinear(b, d).unwrap());

        assert!(c.is_collinear(c, c).is_err());
        assert!(a.is_collinear(a, a).is_err());
        assert!(b.is_collinear(b, b).is_err());
    }
} // end of Testing module
