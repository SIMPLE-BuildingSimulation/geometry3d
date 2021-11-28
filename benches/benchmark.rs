
use criterion::{black_box, criterion_group, criterion_main, Criterion};
use geometry3d::Vector3D;

fn get_perpendicular(v: Vector3D){
    v.get_perpendicular().unwrap();
}

pub fn criterion_benchmark(c: &mut Criterion) {

    c.bench_function("get_perpendicular", |b| b.iter(|| get_perpendicular(black_box(Vector3D::new(1., 2., 3.) ))));
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);