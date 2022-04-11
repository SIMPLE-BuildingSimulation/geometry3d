use criterion::{black_box, criterion_group, criterion_main, Criterion};
#[cfg(feature = "texture")]
use geometry3d::round_error::ApproxFloat;

pub fn criterion_benchmark(c: &mut Criterion) {
    // c.bench_function("get_perpendicular", |b| b.iter(|| get_perpendicular(black_box(Vector3D::new(1., 2., 3.) )) ));

    #[cfg(feature = "texture")]
    let n1 = black_box(ApproxFloat::from_value_and_error(2.12312, 12.));
    #[cfg(feature = "texture")]
    let n2 = black_box(ApproxFloat::from_value_and_error(-2.12312, 2.));

    #[cfg(not(feature = "texture"))]
    let n1 = 12.12131;
    #[cfg(not(feature = "texture"))]
    let n2 = 0.12141;

    c.bench_function("mul", |b| {
        b.iter(|| {
            let _ = black_box(n1 * n2);
        })
    });
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);
