
use criterion::{black_box, criterion_group, criterion_main, Criterion};
use geometry3d::Vector3D;

fn get_perpendicular(v: Vector3D){
    v.get_perpendicular().unwrap();
}

fn test_trigonometry(a: f64, b: f64, c: f64)->f64{    
    // a.cos() + a.sin() + a.tan()
    // a.sqrt()
    a / b / c
}

pub fn criterion_benchmark(c: &mut Criterion) {

    // c.bench_function("get_perpendicular", |b| b.iter(|| get_perpendicular(black_box(Vector3D::new(1., 2., 3.) )) ));

    c.bench_function("cos_sin_tan", |b| b.iter(|| 
        test_trigonometry(
            black_box(3.12123),
            black_box(3.12123),
            black_box(3.12123)
        )
    ));
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);