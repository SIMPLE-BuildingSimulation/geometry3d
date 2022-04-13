use criterion::{black_box, criterion_group, criterion_main, Criterion};
#[cfg(feature = "texture")]
use geometry3d::round_error::ApproxFloat;

pub fn criterion_benchmark(c: &mut Criterion) {
    // c.bench_function("get_perpendicular", |b| b.iter(|| get_perpendicular(black_box(Vector3D::new(1., 2., 3.) )) ));

    // #[cfg(feature = "texture")]
    // let n1 = black_box(ApproxFloat::from_value_and_error(2.12312, 12.));
    // #[cfg(feature = "texture")]
    // let n2 = black_box(ApproxFloat::from_value_and_error(-2.12312, 2.));

    // #[cfg(not(feature = "texture"))]
    // let n1 = 12.12131;
    // #[cfg(not(feature = "texture"))]
    // let n2 = 0.12141;

    // c.bench_function("mul", |b| {
    //     b.iter(|| {
    //         let _ = black_box(n1 * n2);
    //     })
    // });


    // c.bench_function("bbox_intersection", |b| {
    //     b.iter(|| {
    //         let _ = black_box(n1 * n2);
    //     })
    // });


    let bbox = black_box(geometry3d::BBox3D::new( 
        geometry3d::Point3D::new(0., 0., 0.) ,
        geometry3d::Point3D::new(1., 1., 1.) ,
    ));
    let (x, y, z) = (0.5, 0.5, -2.);
    let ray = black_box(geometry3d::Ray3D{
        origin: geometry3d::Point3D::new(x, y, z),
        direction: geometry3d::Vector3D::new(0., 0., 1.),
    });
    let inv_dir = black_box(geometry3d::Vector3D::new(1./x, 1./y, 1./z));

    c.bench_function("bbox_intersection", |b| {
        b.iter(|| {
            
            let  _ = black_box(bbox.intersect(&ray, &inv_dir));
        })
    });



    let v = black_box([1., 2., -9.1, 12.]);
    c.bench_function("max_min", |b| {
        b.iter(|| {
            
            let  _ = black_box(geometry3d::round_error::max_min(&v));
        })
    });




}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);
