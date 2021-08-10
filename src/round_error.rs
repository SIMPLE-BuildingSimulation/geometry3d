use crate::Float;

fn next_float_up(v: Float) -> Float {
    let mut v = v;

    // Handle infinity and negative zero for _NextFloatUp()_
    if v.is_infinite() && v > 0. {
        return v;
    }
    if v == -0.0 as Float {
        v = 0. as Float;
    }

    // Advance _v_ to next higher float
    let mut ui = v.to_bits();
    if v >= 0. {
        ui += 1;
    } else {
        ui -= 1;
    }
    return Float::from_bits(ui);
}

fn next_float_down(v: Float) -> Float {
    let mut v = v;

    // Handle infinity and positive zero for _NextFloatDown()_
    if v.is_infinite() && v < 0. {
        return v;
    }
    if v == 0.0 as Float {
        v = -0.0 as Float;
    }
    let mut ui = v.to_bits();
    if v > 0. {
        ui -= 1;
    } else {
        ui += 1;
    }
    return Float::from_bits(ui);
}

#[macro_export]
macro_rules! gamma {
    ( $n : expr ) => {{
        const HALF_EPSILON: Float = Float::EPSILON / 2.0;

        let nm = HALF_EPSILON * ($n as Float);
        (nm) / (1. - nm)
    }};
}

/// Contains a [`Float`] value as well as an error
/// interval.
#[derive(Clone, Copy)]
pub struct ApproxFloat {
    /// The lower boundary
    pub low: Float,

    /// The higher boundary
    pub high: Float,
}

impl From<Float> for ApproxFloat {
    fn from(value: Float) -> Self {
        Self::from_value_and_error(value, 0.0)
    }
}

impl ApproxFloat {
    /// Creates a new ApproxFloat
    pub fn from_value_and_error(value: Float, abs_error: Float) -> Self {
        Self {
            low: value - abs_error,
            high: value + abs_error,
        }
    }

    /// Creates a new ApproxFloat
    pub fn from_bounds(low: Float, high: Float) -> Self {
        debug_assert!(high >= low);
        Self { low, high }
    }

    /// Returns the midpoint
    pub fn midpoint(&self) -> Float {
        (self.low + self.high) / 2.
    }

    /// Returns the midpoint
    pub fn as_float(&self) -> Float {
        self.midpoint()
    }

    /// Calculates the square root
    pub fn sqrt(&self) -> Self {
        let exp_sqrt = self.midpoint().sqrt();
        let low_err = exp_sqrt - next_float_down(self.low.sqrt()).clamp(0., Float::MAX);
        let high_err = next_float_up(self.high.sqrt()) - exp_sqrt;
        let err = if low_err > high_err {
            low_err
        } else {
            high_err
        };
        Self::from_value_and_error(exp_sqrt, err)

        // let mut low = next_float_down(self.low.sqrt());
        // if low < 0. {
        //     low = 0.
        // }
        // let high = next_float_up(self.high.sqrt());

        // Self{
        //     low,high
        // }
    }

    // Retrieves the absolute error
    pub fn absolute_error(&self) -> Float {
        (self.high - self.low) / 2.
    }

    /// Identifies the values for $`ax^2+bx+c=0`$ problem,
    /// where $`$x = \frac{-b +- \sqrt{b^2-4ac}}{2a}`$
    /// The returned result contains the smallest value first.
    ///
    /// Source: PBRT-v3 source code.
    pub fn solve_quadratic(
        a: ApproxFloat,
        b: ApproxFloat,
        c: ApproxFloat,
    ) -> Option<(ApproxFloat, ApproxFloat)> {
        // Find quadratic discriminant
        let disc = b * b - a * c * 4.;
        if disc.low < 0. {
            // Does not intersect
            return None;
        }
        let discr_sqrt = disc.sqrt();

        // Use Muller's method for making this faster...
        let q: ApproxFloat;
        if b.as_float() < 0. {
            q = -(b - discr_sqrt) * 0.5;
        } else {
            q = -(b + discr_sqrt) * 0.5;
        }
        let mut x1 = q / a;
        let mut x2 = c / q;

        // Sort them
        if x1.low > x2.low {
            std::mem::swap(&mut x1, &mut x2);
        }

        Some((x1, x2))
    }
}

impl std::ops::Neg for ApproxFloat {
    type Output = Self;

    fn neg(self) -> Self::Output {
        Self {
            low: -self.low,
            high: -self.high,
        }
    }
}

impl std::ops::Add for ApproxFloat {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        Self {
            low: next_float_down(self.low + other.low),
            high: next_float_up(self.high + other.high),
        }
    }
}

impl std::ops::Add<Float> for ApproxFloat {
    type Output = Self;

    fn add(self, other: Float) -> Self {
        self + Self::from(other)
    }
}

impl std::ops::Sub for ApproxFloat {
    type Output = Self;

    fn sub(self, other: Self) -> Self {
        Self {
            low: next_float_down(self.low - other.low),
            high: next_float_up(self.high - other.high),
        }
    }
}

impl std::ops::Sub<Float> for ApproxFloat {
    type Output = Self;

    fn sub(self, other: Float) -> Self {
        self - Self::from(other)
    }
}

fn max_min(aux: &[Float; 4]) -> (Float, Float) {
    let mut min = aux[0];
    let mut max = aux[0];
    for i in 1..4 {
        if aux[i] < min {
            min = aux[i];
        } else if aux[i] > max {
            max = aux[i];
        }
    }
    (max, min)
}

impl std::ops::Mul for ApproxFloat {
    type Output = Self;

    fn mul(self, other: Self) -> Self {
        let exp = self.midpoint() * other.midpoint();

        let lp = [
            self.low * other.low,
            self.high * other.low,
            self.low * other.high,
            self.high * other.high,
        ];
        let hp = [
            self.low * other.low,
            self.high * other.low,
            self.low * other.high,
            self.high * other.high,
        ];
        let (_, min) = max_min(&lp);
        let (max, _) = max_min(&hp);
        let low_err = exp - next_float_down(min);
        let max_err = next_float_up(max) - exp;
        let err = if low_err > max_err { low_err } else { max_err };
        Self::from_value_and_error(exp, err)

        // let lp = [self.low * other.low, self.high * other.low, self.low * other.high, self.high * other.high];
        // let hp = [self.low * other.low, self.high * other.low, self.low * other.high, self.high * other.high];
        // let (_,min) = max_min(&lp);
        // let (max,_) = max_min(&hp);

        // Self {
        //     low:  next_float_down(min),
        //     high: next_float_up(max),
        // }
    }
}

impl std::ops::Mul<Float> for ApproxFloat {
    type Output = Self;

    fn mul(self, other: Float) -> Self {
        self * Self::from(other)
    }
}

impl std::ops::Div for ApproxFloat {
    type Output = Self;

    fn div(self, other: Self) -> Self {
        let exp = self.midpoint() / other.midpoint();

        let lp = [
            self.low / other.low,
            self.high / other.low,
            self.low / other.high,
            self.high / other.high,
        ];
        let hp = [
            self.low / other.low,
            self.high / other.low,
            self.low / other.high,
            self.high / other.high,
        ];
        let (_, min) = max_min(&lp);
        let (max, _) = max_min(&hp);
        let low_err = exp - next_float_down(min);
        let max_err = next_float_up(max) - exp;
        let err = if low_err > max_err { low_err } else { max_err };
        Self::from_value_and_error(exp, err)

        // let lp = [self.low / other.low, self.high / other.low, self.low / other.high, self.high * other.high];
        // let hp = [self.low / other.low, self.high / other.low, self.low / other.high, self.high * other.high];
        // let (_,min) = max_min(&lp);
        // let (max,_) = max_min(&hp);
        // Self {
        //     low:  next_float_down(min),
        //     high: next_float_up(max),
        // }
    }
}

impl std::ops::Div<Float> for ApproxFloat {
    type Output = Self;

    fn div(self, other: Float) -> Self {
        self / Self::from(other)
    }
}

impl std::ops::AddAssign for ApproxFloat {
    fn add_assign(&mut self, other: Self) {
        self.low = next_float_down(self.low + other.low);
        self.high = next_float_up(self.high + other.high);
    }
}

impl std::ops::AddAssign<Float> for ApproxFloat {
    fn add_assign(&mut self, other: Float) {
        *self += Self::from(other)
    }
}

impl std::ops::SubAssign for ApproxFloat {
    fn sub_assign(&mut self, other: Self) {
        self.low = next_float_down(self.low - other.low);
        self.high = next_float_up(self.high - other.high);
    }
}

impl std::ops::SubAssign<Float> for ApproxFloat {
    fn sub_assign(&mut self, other: Float) {
        *self -= Self::from(other)
    }
}

impl std::ops::MulAssign for ApproxFloat {
    fn mul_assign(&mut self, other: Self) {
        let exp = self.midpoint() * other.midpoint();

        let lp = [
            self.low * other.low,
            self.high * other.low,
            self.low * other.high,
            self.high * other.high,
        ];
        let hp = [
            self.low * other.low,
            self.high * other.low,
            self.low * other.high,
            self.high * other.high,
        ];
        let (_, min) = max_min(&lp);
        let (max, _) = max_min(&hp);
        let low_err = exp - next_float_down(min);
        let max_err = next_float_up(max) - exp;
        let err = if low_err > max_err { low_err } else { max_err };
        self.low = exp - err;
        self.high = exp + err;

        // let aux = [self.low * other.low, self.high * other.low, self.low * other.high, self.high * other.high];
        // let (max,min) = max_min(&aux);
        // assert!(false);
        // self.low =  next_float_down(min);
        // self.high = next_float_up(max);
    }
}

impl std::ops::MulAssign<Float> for ApproxFloat {
    fn mul_assign(&mut self, other: Float) {
        *self *= Self::from(other)
    }
}

impl std::ops::DivAssign for ApproxFloat {
    fn div_assign(&mut self, other: Self) {
        let exp = self.midpoint() / other.midpoint();

        let lp = [
            self.low / other.low,
            self.high / other.low,
            self.low / other.high,
            self.high / other.high,
        ];
        let hp = [
            self.low / other.low,
            self.high / other.low,
            self.low / other.high,
            self.high / other.high,
        ];
        let (_, min) = max_min(&lp);
        let (max, _) = max_min(&hp);
        let low_err = exp - next_float_down(min);
        let max_err = next_float_up(max) - exp;
        let err = if low_err > max_err { low_err } else { max_err };
        self.low = exp - err;
        self.high = exp + err;
        // let aux = [self.low * other.low, self.high * other.low, self.low * other.high, self.high * other.high];
        // let (max,min) = max_min(&aux);

        // assert!(false);
        // self.low =  next_float_down(min);
        // self.high = next_float_up(max);
    }
}

impl std::ops::DivAssign<Float> for ApproxFloat {
    fn div_assign(&mut self, other: Float) {
        *self /= Self::from(other)
    }
}

#[cfg(test)]
mod testing {

    use super::*;

    #[test]
    fn test_next_float() {
        fn check(a: Float) -> Result<(), String> {
            let aup = next_float_up(a);
            let adown = next_float_down(a);
            if aup < a {
                return Err(format!("'aup' ({}) is lower than 'a' ({})", aup, a));
            }
            if adown > a {
                return Err(format!("'adown' ({}) is larger than 'a' ({})", adown, a));
            }

            // if (aup - a) > 100.*Float::EPSILON{
            //     return Err(format!("Difference between 'aup' and 'a' ({}) is larger than Epsilon ({})", aup - a, Float::EPSILON))
            // }

            // if (a - adown) > 100.*Float::EPSILON{
            //     return Err(format!("Difference between 'a' and 'adown' ({}) is larger than Epsilon ({})", a - adown, Float::EPSILON))
            // }

            Ok(())
        }

        check(0.).unwrap();
        check(Float::EPSILON).unwrap();
        check(-Float::EPSILON).unwrap();
        check(-12312.).unwrap();

        check(Float::MAX).unwrap();
    }

    #[test]
    fn test_gamma_macro() {
        for n in 0..20 {
            let nm = Float::EPSILON / 2. * n as Float;
            let exp = (nm) / (1. - nm);
            let found = gamma!(n);
            // println!("exp  : {} \nfound: {}\n======", exp,found);
            assert!((exp - found).abs() < f64::EPSILON)
        }
    }

    #[test]
    fn test_from_float() {
        let a = 1.23;
        let approx = ApproxFloat::from(a);
        assert_eq!(a.to_bits(), approx.low.to_bits());
        assert_eq!(a.to_bits(), approx.high.to_bits());
    }

    #[test]
    fn test_from_value_error() {
        let v = 1.23;
        let err = 0.1231;
        let approx = ApproxFloat::from_value_and_error(v, err);

        // check that the difference is correct
        assert!(((approx.high - approx.low) - err * 2.).abs() < Float::EPSILON);
        // Check that the signs are correct
        assert!(approx.high > approx.low);
    }

    #[test]
    fn test_from_bounds() {
        let low = 1.23;
        let high = 2. * low;

        let approx = ApproxFloat::from_bounds(low, high);

        assert_eq!(low.to_bits(), approx.low.to_bits());
        assert_eq!(high.to_bits(), approx.high.to_bits());
    }

    #[test]
    fn test_midpoint_as_float() {
        let low = 1.1231;
        let high = 9872.123;
        let mid = (low + high) / 2.;
        let app = ApproxFloat { low, high };
        assert!((mid - app.midpoint()).abs() < Float::EPSILON);
        assert!((mid - app.as_float()).abs() < Float::EPSILON);
    }

    #[test]
    fn test_approx_sqrt() {
        // With zero error
        let a = 1.123123;
        let app = ApproxFloat::from(a);
        assert!((a.sqrt() - app.sqrt().as_float()).abs() < Float::EPSILON);

        // With error
        let a = 0.2;
        let err = 0.1;
        let app = ApproxFloat::from_value_and_error(a, err);
        assert_eq!(a.sqrt(), app.sqrt().as_float());
        assert!((a.sqrt() - app.sqrt().as_float()).abs() < Float::EPSILON);
    }

    #[test]
    fn test_absolute_error() {
        // With zero error
        let a = 1.123123;
        let app = ApproxFloat::from(a);
        assert!(app.absolute_error() < Float::EPSILON);

        // With error
        let a = 1.123123;
        let err = 0.2;
        let app = ApproxFloat::from_value_and_error(a, err);
        assert!((app.absolute_error() - err).abs() < Float::EPSILON);
    }

    #[test]
    fn test_approx_quadratic() {
        let a = ApproxFloat::from(1.);
        let b = ApproxFloat::from(-1.);
        let c = ApproxFloat::from(-6.);
        if let Some((x1, x2)) = ApproxFloat::solve_quadratic(a, b, c) {
            println!("x1 = {}, x2 = {}", x1.midpoint(), x2.midpoint());
            assert_eq!(x1.midpoint(), -2.);
            assert_eq!(x2.midpoint(), 3.);
        } else {
            panic!("Expected results!")
        }

        let a = ApproxFloat::from(-6.);
        let b = ApproxFloat::from(15.);
        let c = ApproxFloat::from(36.);
        if let Some((x1, x2)) = ApproxFloat::solve_quadratic(a, b, c) {
            println!("x1 = {}, x2 = {}", x1.midpoint(), x2.midpoint());
            assert_eq!(x1.midpoint(), -3. / 2.);
            assert_eq!(x2.midpoint(), 4.);
        } else {
            panic!("Expected results!")
        }

        let a = ApproxFloat::from(6.);
        let b = ApproxFloat::from(5.);
        let c = ApproxFloat::from(-6.);
        if let Some((x1, x2)) = ApproxFloat::solve_quadratic(a, b, c) {
            println!("x1 = {}, x2 = {}", x1.midpoint(), x2.midpoint());
            assert_eq!(x1.midpoint(), -3. / 2.);
            assert_eq!(x2.midpoint(), 2. / 3.);
        } else {
            panic!("Expected results!")
        }

        // One solution... not really possible with uncertainty. can Discriminant be == 0.?
        // let a = ApproxFloat::from(2.);
        // if let Some((x1,x2)) =  ApproxFloat::solve_quadratic(ApproxFloat::from(1.), a*2., a*a){
        //     assert_eq!(x1.midpoint(),-a.midpoint());
        //     assert_eq!(x2.midpoint(), x1.midpoint());
        // }else{
        //     panic!("Expected results!")
        // }

        // No solution
        let a = ApproxFloat::from(3.12312598712);
        let c = ApproxFloat::from(2.12312);

        let b = (a * c).sqrt() * 2. - 0.1; // b < 2 (a*c).sqrt() --> discriminant < 0
        if let Some((x1, x2)) = ApproxFloat::solve_quadratic(a, b, c) {
            panic!(
                "unexpected solitions: x1 = {}, x2 = {}",
                x1.midpoint(),
                x2.midpoint()
            );
        }
    }

    #[test]
    fn test_max_min() {
        let a = [1., 2., 3., 4.];
        let (max, min) = max_min(&a);
        assert_eq!(max, 4.);
        assert_eq!(min, 1.);

        let a = [4., 3., 2., 1.];
        let (max, min) = max_min(&a);
        assert_eq!(max, 4.);
        assert_eq!(min, 1.);

        let a = [1., 1.2, 5., 1.];
        let (max, min) = max_min(&a);
        assert_eq!(max, 5.);
        assert_eq!(min, 1.);

        let a = [-1., -1.2, -5., -1.];
        let (max, min) = max_min(&a);
        assert_eq!(max, -1.);
        assert_eq!(min, -5.);
    }

    #[test]
    fn test_approx_operations() {
        let value = 4.1231;
        let error = 0.123;
        let this = ApproxFloat::from_value_and_error(value, error);
        let other_value = 123.;
        let other_error = 0.4;
        let other = ApproxFloat::from_value_and_error(other_value, other_error);

        // Negate
        assert_eq!(-(this.as_float()), (-this).as_float());

        // Add Self
        let result = this + other;
        assert_eq!(result.as_float(), value + other_value);

        // Add Float
        let result = this + other_value;
        assert_eq!(result.as_float(), value + other_value);

        // Sub Self
        let result = this - other;
        assert_eq!(result.as_float(), value - other_value);

        // Sub Float
        let result = this - other_value;
        assert_eq!(result.as_float(), value - other_value);

        // Mul Self
        let result = this * other;
        assert_eq!(result.as_float(), value * other_value);

        // Mul Float
        let result = this * other_value;
        assert_eq!(result.as_float(), value * other_value);

        // Div Self
        let result = this / other;
        assert_eq!(result.as_float(), value / other_value);

        // Div Float
        let result = this / other_value;
        assert_eq!(result.as_float(), value / other_value);

        // Add Assign Self
        let mut new_this = this;
        new_this += other;
        assert_eq!(new_this.as_float(), value + other_value);

        // Add Assign Float
        let mut new_this = this;
        new_this += other_value;
        assert_eq!(new_this.as_float(), value + other_value);

        // Sub Assign Self
        let mut new_this = this;
        new_this -= other;
        assert_eq!(new_this.as_float(), value - other_value);

        // Sub Assign Float
        let mut new_this = this;
        new_this -= other_value;
        assert_eq!(new_this.as_float(), value - other_value);

        // Mul Assign Self
        let mut new_this = this;
        new_this *= other;
        assert_eq!(new_this.as_float(), value * other_value);

        // Mul Assign Float
        let mut new_this = this;
        new_this *= other_value;
        assert_eq!(new_this.as_float(), value * other_value);

        // Div Assign Self
        let mut new_this = this;
        new_this /= other;
        assert_eq!(new_this.as_float(), value / other_value);

        // Div Assign Float
        let mut new_this = this;
        new_this /= other_value;
        assert_eq!(new_this.as_float(), value / other_value);
    }
}
