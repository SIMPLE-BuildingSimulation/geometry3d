use crate::Float;

/// Identifies the values for $`ax^2+bx+c=0`$ problem,
/// where $`$x = \frac{-b +- \sqrt{b^2-4ac}}{2a}`$
/// The returned result contains the smallest value first.
///
/// Source: PBRT-v3 source code.
#[allow(dead_code)]
pub fn solve_quadratic(a: Float, b: Float, c: Float) -> Option<(Float, Float)> {
    // if a is Zero, then this is a line with slope b
    if a == 0. {
        if b == 0. {
            // If b is zero, then this line is horizontal... does not cross
            return None;
        }
        let x = -c / b;
        return Some((x, x));
    }

    // Find quadratic discriminant
    let disc = b * b - 4. * a * c;
    if disc < 0. {
        // Does not intersect
        return None;
    }
    let discr_sqrt = disc.sqrt();

    // Use Muller's method for making this faster...
    let q: Float;
    if b < 0. {
        q = -0.5 * (b - discr_sqrt);
    } else {
        q = -0.5 * (b + discr_sqrt);
    }
    let mut x1 = q / a;
    let mut x2 = c / q;

    // Sort them
    if x1 > x2 {
        std::mem::swap(&mut x1, &mut x2);
    }

    Some((x1, x2))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_quadratic() {
        if let Some((x1, x2)) = solve_quadratic(1., -1., -6.) {
            println!("x1 = {}, x2 = {}", x1, x2);
            assert_eq!(x1, -2.);
            assert_eq!(x2, 3.);
        } else {
            panic!("Expected results!")
        }

        if let Some((x1, x2)) = solve_quadratic(-6., 15., 36.) {
            println!("x1 = {}, x2 = {}", x1, x2);
            assert_eq!(x1, -3. / 2.);
            assert_eq!(x2, 4.);
        } else {
            panic!("Expected results!")
        }

        if let Some((x1, x2)) = solve_quadratic(6., 5., -6.) {
            println!("x1 = {}, x2 = {}", x1, x2);
            assert_eq!(x1, -3. / 2.);
            assert_eq!(x2, 2. / 3.);
        } else {
            panic!("Expected results!")
        }

        // One solution
        let a = 2.;
        if let Some((x1, x2)) = solve_quadratic(1., 2. * a, a * a) {
            assert_eq!(x1, -a);
            assert_eq!(x2, x1);
        } else {
            panic!("Expected results!")
        }

        // No solution
        let a: Float = 3.12312598712;
        let c: Float = 2.12312;

        let b = 2. * (a * c).sqrt() - 0.1; // b < 2 (a*c).sqrt() --> discriminant < 0
        if let Some((x1, x2)) = solve_quadratic(a, b, c) {
            panic!("unexpected solitions: x1 = {}, x2 = {}", x1, x2);
        }
    }
}
