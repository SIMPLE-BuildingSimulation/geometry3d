use crate::point3d::Point3D;
use crate::vector3d::Vector3D;


macro_rules! elem {    
    ( $row : expr, $col:expr ) => {{
        assert!($row < 4);
        // assert!($row >= 0);
        assert!($col < 4);
        // assert!($col >= 0);
        4*$row+$col
    }};
}

/// Represents a transformation matrix
pub struct Transform {
    elements : [f64;16],
    inv_elements:[f64;16]
}


impl Default for Transform {
    fn default() -> Self { Self::new() }
}

impl Transform {   
    
    /// Crates a new Identity [`Transform`] (i.e., does nothing to 
    /// the points being passed)
    pub fn new()->Self{
        let mut elements = [0.0;16];
        for i in 0..4 {
            elements[elem!(i,i)] = 1.0;
        };
        let inv_elements = elements;

        Self{
            elements,
            inv_elements,
        }
    }

    /// Creates a new Translation [`Transform`]
    pub fn translate(x:f64,y:f64,z:f64)->Self{
        // Get an identity
        let mut ret = Self::new();

        // set translation components
        ret.elements[elem!(0,3)]=x;
        ret.elements[elem!(1,3)]=y;
        ret.elements[elem!(2,3)]=z;
        
        // set inverse translation components
        ret.inv_elements[elem!(0,3)]=-x;
        ret.inv_elements[elem!(1,3)]=-y;
        ret.inv_elements[elem!(2,3)]=-z;

        ret
    }

    /// Creates a new Scale [`Transform`]
    pub fn scale(x:f64,y:f64,z:f64)->Self{
        let mut elements = [0.0;16];                
        elements[elem!(0,0)] = x;
        elements[elem!(1,1)] = y;
        elements[elem!(2,2)] = z;
        elements[elem!(3,3)] = 1.0;

        let mut inv_elements = [0.0;16];
        inv_elements[elem!(0,0)] = 1.0/x;
        inv_elements[elem!(1,1)] = 1.0/y;
        inv_elements[elem!(2,2)] = 1.0/z;
        inv_elements[elem!(3,3)] = 1.0;

        Self{
            elements,
            inv_elements,
        }
    }

    /// Creates a new [`Transform`] that rotates counterclockwise on the X axis. 
    /// The input is in degrees
    pub fn rotate_x(degrees:f64)->Self{
        let rad = degrees * std::f64::consts::PI/180.;
        let mut ret = Self::new();

        ret.elements[elem!(1,1)]=rad.cos();
        ret.inv_elements[elem!(1,1)]=rad.cos();

        ret.elements[elem!(1,2)]=-rad.sin();
        ret.inv_elements[elem!(2,1)]=-rad.sin();

        ret.elements[elem!(2,1)]=rad.sin();
        ret.inv_elements[elem!(1,2)]=rad.sin();

        ret.elements[elem!(2,2)]=rad.cos();
        ret.inv_elements[elem!(2,2)]=rad.cos();
        
        ret
    }

    /// Creates a new [`Transform`] that rotates counterclockwise on the Y axis. 
    /// The input is in degrees
    pub fn rotate_y(degrees:f64)->Self{
        let rad = degrees * std::f64::consts::PI/180.;
        let mut ret = Self::new();

        ret.elements[elem!(0,0)]=rad.cos();
        ret.inv_elements[elem!(0,0)]=rad.cos();

        ret.elements[elem!(0,2)]=rad.sin();
        ret.inv_elements[elem!(2,0)]=rad.sin();

        ret.elements[elem!(2,0)]=-rad.sin();
        ret.inv_elements[elem!(0,2)]=-rad.sin();

        ret.elements[elem!(2,2)]=rad.cos();
        ret.inv_elements[elem!(2,2)]=rad.cos();
        
        ret
    }

    /// Creates a new [`Transform`] that rotates counterclockwise on the Z axis. 
    /// The input is in degrees
    pub fn rotate_z(degrees:f64)->Self{
        let rad = degrees * std::f64::consts::PI/180.;
        let mut ret = Self::new();

        ret.elements[elem!(0,0)]=rad.cos();
        ret.inv_elements[elem!(0,0)]=rad.cos();

        ret.elements[elem!(0,1)]=-rad.sin();
        ret.inv_elements[elem!(1,0)]=-rad.sin();

        ret.elements[elem!(1,0)]=rad.sin();
        ret.inv_elements[elem!(0,1)]=rad.sin();

        ret.elements[elem!(1,1)]=rad.cos();
        ret.inv_elements[elem!(1,1)]=rad.cos();
        
        ret
    }

    /// Transforms a [`Point3D`] into another [`Point3D`]
    pub fn transform_pt(&self, pt: Point3D)->Point3D{
        let (x,y,z) = (pt.x, pt.y, pt.z);
        let new_x = self.elements[elem!(0,0)] * x + self.elements[elem!(0,1)] * y + self.elements[elem!(0,2)] * z + self.elements[elem!(0,3)];
        let new_y = self.elements[elem!(1,0)] * x + self.elements[elem!(1,1)] * y + self.elements[elem!(1,2)] * z + self.elements[elem!(1,3)];
        let new_z = self.elements[elem!(2,0)] * x + self.elements[elem!(2,1)] * y + self.elements[elem!(2,2)] * z + self.elements[elem!(2,3)];
        let w =     self.elements[elem!(3,0)] * x + self.elements[elem!(3,1)] * y + self.elements[elem!(3,2)] * z + self.elements[elem!(3,3)];

        Point3D::new(new_x, new_y, new_z)/w       
    }
    

    /// Transforms a [`Vector3D`] into another [`Point3D`]
    pub fn transform_vec(&self, vec: Vector3D)->Vector3D{
        let (x,y,z) = (vec.x, vec.y, vec.z);
        let new_x = self.elements[elem!(0,0)] * x + self.elements[elem!(0,1)] * y + self.elements[elem!(0,2)] * z;
        let new_y = self.elements[elem!(1,0)] * x + self.elements[elem!(1,1)] * y + self.elements[elem!(1,2)] * z;
        let new_z = self.elements[elem!(2,0)] * x + self.elements[elem!(2,1)] * y + self.elements[elem!(2,2)] * z;

        Vector3D::new(new_x, new_y, new_z)       
    }

    /// Transforms a [`Vector3D`] into another [`Point3D`]
    pub fn transform_normal(&self, vec: Vector3D)->Vector3D{
        let (x,y,z) = (vec.x, vec.y, vec.z);
        let new_x = self.inv_elements[elem!(0,0)]*x + self.inv_elements[elem!(1,0)]*y + self.inv_elements[elem!(2,0)]*z;
        let new_y = self.inv_elements[elem!(0,1)]*x + self.inv_elements[elem!(1,1)]*y + self.inv_elements[elem!(2,1)]*z;
        let new_z = self.inv_elements[elem!(0,2)]*x + self.inv_elements[elem!(1,2)]*y + self.inv_elements[elem!(2,2)]*z;

        Vector3D::new(new_x, new_y, new_z)       
    }

    
}




#[cfg(test)]
mod testing {
    use super::*;

    #[test]
    fn test_macro_i(){
        assert_eq!(elem!(0,0),0);
        assert_eq!(elem!(0,1),1);
        assert_eq!(elem!(0,2),2);
        assert_eq!(elem!(0,3),3);

        assert_eq!(elem!(1,0),4);
        assert_eq!(elem!(1,1),5);
        assert_eq!(elem!(1,2),6);
        assert_eq!(elem!(1,3),7);

        assert_eq!(elem!(2,0),8);
        assert_eq!(elem!(2,1),9);
        assert_eq!(elem!(2,2),10);
        assert_eq!(elem!(2,3),11);

        assert_eq!(elem!(3,0),12);
        assert_eq!(elem!(3,1),13);
        assert_eq!(elem!(3,2),14);
        assert_eq!(elem!(3,3),15);
    }

    #[test]
    #[should_panic]
    fn test_macro_panic_out_of_bounds(){
        assert_eq!(elem!(4,0),0);
    }
    #[test]
    #[should_panic]
    fn test_macro_panic_out_of_bounds_2(){
        assert_eq!(elem!(0,4),0);
    }
    #[test]
    #[should_panic]
    fn test_macro_panic_negative(){
        assert_eq!(elem!(-1,0),0);
    }
    #[test]
    #[should_panic]
    fn test_macro_panic_negative_2(){
        assert_eq!(elem!(0,-1),0);
    }

    #[test]
    fn test_default_transform(){
        let t = Transform::default();
        for row in 0..4{
            for col in 0..4{
                if row == col{
                    assert_eq!(t.elements[elem!(row,col)], 1.0);
                    assert_eq!(t.inv_elements[elem!(row,col)], 1.0);
                }else{
                    assert_eq!(t.elements[elem!(row,col)], 0.0);
                    assert_eq!(t.inv_elements[elem!(row,col)], 0.0);
                }
            }
        }

        fn check( x:f64, y: f64, z:f64)->Result<(),String>{
            let t = Transform::new();
            let pt = Point3D::new(x,y,z);
            let vec = Vector3D::new(x,y,z);
            if pt != t.transform_pt(pt){
                return Err(format!("point {} is not the same as transformed {}", pt, t.transform_pt(pt)))
            }            
            if vec != t.transform_vec(vec){
                return Err(format!("vec {} is not the same as transformed {}", vec, t.transform_vec(vec)))
            }                        
            Ok(())
        }

        check( 0., 0., 0.).unwrap();
        check( 0., 0., 1.).unwrap();
        check( 0., 1., 0.).unwrap();
        check( 1., 0., 0.).unwrap();
        check( 1., 1., 0.).unwrap();
        check( 1., 1., 1.).unwrap();
        check( -1., -1., 1.).unwrap();

        
    }

    #[test]
    fn test_translate_transform(){
        let (x,y,z) = (2.1, -4.2, 19.2);
        let t = Transform::translate(x,y,z);
        for row in 0..4{
            for col in 0..4{
                if row == col{
                    assert_eq!(t.elements[elem!(row,col)], 1.0);
                    assert_eq!(t.inv_elements[elem!(row,col)], 1.0);
                }else if col == 3 {
                    if row == 0{
                        assert_eq!(t.elements[elem!(row,col)], x);
                        assert_eq!(t.inv_elements[elem!(row,col)], -x);
                    }else if row == 1{
                        assert_eq!(t.elements[elem!(row,col)], y);
                        assert_eq!(t.inv_elements[elem!(row,col)], -y);
                    }else if row == 2{
                        assert_eq!(t.elements[elem!(row,col)], z);
                        assert_eq!(t.inv_elements[elem!(row,col)], -z);
                    }
                }else{                    
                    assert_eq!(t.elements[elem!(row,col)], 0.0);
                    assert_eq!(t.inv_elements[elem!(row,col)], 0.0)
                }
            }
        }

        fn check(x:f64, y: f64, z:f64)->Result<(),String>{
            let t = Transform::translate(x,y,z);

            let pt = Point3D::new(3.+2.*x,-1.*y+9.,z);
            let vec = Vector3D::new(x,21.*y,-33.*z);

            let pprime = pt + Vector3D::new(x,y,z);
            let vecprime = vec;

            if pprime != t.transform_pt(pt) {
                return Err(format!("point {} is not the same as transformed {}", t.transform_pt(pt), pprime))
            }
            if vecprime != t.transform_vec(vec) {
                return Err(format!("vector {} is not the same as transformed {}", t.transform_vec(vec), vecprime))
            }                   
            Ok(())
        }

        check( 0., 0., 0.).unwrap();
        check( 0., 0., 1.).unwrap();
        check( 0., 1., 0.).unwrap();
        check( 1., 0., 0.).unwrap();
        check( 1., 1., 0.).unwrap();
        check( 1., 1., 1.).unwrap();
        check( -1., -1., 1.).unwrap();        
    }

    #[test]
    fn test_scale_transform(){
        let (x,y,z) = (2.1, -4.2, 19.2);
        let t = Transform::scale(x,y,z);
        for row in 0..4{
            for col in 0..4{
                if row == col{                    
                    if row == 0{
                        assert_eq!(t.elements[elem!(row,col)], x);
                        assert_eq!(t.inv_elements[elem!(row,col)], 1./x);
                    }else if row == 1{
                        assert_eq!(t.elements[elem!(row,col)], y);
                        assert_eq!(t.inv_elements[elem!(row,col)], 1./y);
                    }else if row == 2{
                        assert_eq!(t.elements[elem!(row,col)], z);
                        assert_eq!(t.inv_elements[elem!(row,col)], 1./z);
                    }else{
                        assert_eq!(t.elements[elem!(row,col)], 1.);
                        assert_eq!(t.inv_elements[elem!(row,col)], 1.);
                    }                
                }else{                    
                    assert_eq!(t.elements[elem!(row,col)], 0.0);
                    assert_eq!(t.inv_elements[elem!(row,col)], 0.0)
                }
            }
        }

        fn check( x:f64, y: f64, z:f64)->Result<(),String>{
            let t = Transform::scale(x,y,z);
            
            // shufle them a bit
            let pt = Point3D::new(2.*x,y,z);
            let vec = Vector3D::new(x,-1.*y,3.*z);

            let pprime = Point3D::new(pt.x*x,pt.y*y,pt.z*z);
            let vecprime = Vector3D::new(vec.x*x,vec.y*y,vec.z*z);

            if pprime != t.transform_pt(pt) {
                return Err(format!("point {} is not the same as transformed {}", pt, pprime))
            }
            if vecprime != t.transform_vec(vec) {
                return Err(format!("vector {} is not the same as transformed {}", vec, vecprime))
            }                   
            Ok(())
        }

        check( 0., 0., 0.).unwrap();
        check( 0., 0., 1.).unwrap();
        check( 0., 1., 0.).unwrap();
        check( 1., 0., 0.).unwrap();
        check( 1., 1., 0.).unwrap();
        check( 1., 1., 1.).unwrap();
        check( -1., -1., 1.).unwrap();
    }

    fn compare_pts(pt1:Point3D, pt2:Point3D)->Result<(),String>{
        if (pt1-pt2).length() > 0.00000001{
            return Err(format!("Points {} and {} are not equal", pt1, pt2))
        }
        Ok(())
    }
    fn compare_vecs(vec1:Vector3D, vec2:Vector3D)->Result<(),String>{
        if (vec1-vec2).length() > 0.00000001{
            return Err(format!("Vectors {} and {} are not equal", vec1, vec2))
        }
        Ok(())
    }
    #[test]
    fn test_rotate_x_transform(){
        

        /* POINTS */
        let ex = Point3D::new(1.,0.,0.);
        let ey = Point3D::new(0.,1.,0.);
        let ez = Point3D::new(0.,0.,1.);
        
        
        let t = Transform::rotate_x(90.);
        compare_pts(t.transform_pt(ex),ex).unwrap();
        compare_pts(t.transform_pt(ey), ez).unwrap();
        compare_pts(t.transform_pt(ez),ey*-1.).unwrap();

        let t = Transform::rotate_x(-90.);
        compare_pts(t.transform_pt(ex),ex).unwrap();
        compare_pts(t.transform_pt(ey), ez*-1.).unwrap();
        compare_pts(t.transform_pt(ez),ey).unwrap();

        let t = Transform::rotate_x(180.);
        compare_pts(t.transform_pt(ex),ex).unwrap();
        compare_pts(t.transform_pt(ey), ey*-1.).unwrap();
        compare_pts(t.transform_pt(ez),ez*-1.).unwrap();

        let t = Transform::rotate_x(45.);
        compare_pts(t.transform_pt(ex),ex).unwrap();
        let sq2 = 1./(2 as f64).sqrt();
        compare_pts(t.transform_pt(ey), Point3D::new(0., sq2,sq2)).unwrap();
        compare_pts(t.transform_pt(ez),Point3D::new(0., -sq2,sq2)).unwrap();        

        /* VECTORS */
        let ex = Vector3D::new(1.,0.,0.);
        let ey = Vector3D::new(0.,1.,0.);
        let ez = Vector3D::new(0.,0.,1.);
        
        let t = Transform::rotate_x(90.);
        compare_vecs(t.transform_vec(ex),ex).unwrap();
        compare_vecs(t.transform_vec(ey), ez).unwrap();
        compare_vecs(t.transform_vec(ez),ey*-1.).unwrap();

        let t = Transform::rotate_x(-90.);
        compare_vecs(t.transform_vec(ex),ex).unwrap();
        compare_vecs(t.transform_vec(ey), ez*-1.).unwrap();
        compare_vecs(t.transform_vec(ez),ey).unwrap();

        let t = Transform::rotate_x(180.);
        compare_vecs(t.transform_vec(ex),ex).unwrap();
        compare_vecs(t.transform_vec(ey), ey*-1.).unwrap();
        compare_vecs(t.transform_vec(ez),ez*-1.).unwrap();

        let t = Transform::rotate_x(45.);
        compare_vecs(t.transform_vec(ex),ex).unwrap();
        let sq2 = 1./(2 as f64).sqrt();
        compare_vecs(t.transform_vec(ey), Vector3D::new(0., sq2,sq2)).unwrap();
        compare_vecs(t.transform_vec(ez),Vector3D::new(0., -sq2,sq2)).unwrap();        
    }


    #[test]
    fn test_rotate_y_transform(){
        

        /* POINTS */
        let ex = Point3D::new(1.,0.,0.);
        let ey = Point3D::new(0.,1.,0.);
        let ez = Point3D::new(0.,0.,1.);
        
       
        let t = Transform::rotate_y(90.);
        compare_pts(t.transform_pt(ex),ez * -1.).unwrap();
        compare_pts(t.transform_pt(ey), ey).unwrap();
        compare_pts(t.transform_pt(ez),ex).unwrap();

        let t = Transform::rotate_y(-90.);
        compare_pts(t.transform_pt(ex),ez).unwrap();
        compare_pts(t.transform_pt(ey), ey).unwrap();
        compare_pts(t.transform_pt(ez),ex*-1.).unwrap();

        let t = Transform::rotate_y(180.);
        compare_pts(t.transform_pt(ex),ex*-1.).unwrap();
        compare_pts(t.transform_pt(ey), ey).unwrap();
        compare_pts(t.transform_pt(ez),ez*-1.).unwrap();

        let t = Transform::rotate_y(45.);
        let sq2 = 1./(2 as f64).sqrt();
        compare_pts(t.transform_pt(ex),Point3D::new(sq2, 0., -sq2)).unwrap();        
        compare_pts(t.transform_pt(ey), ey).unwrap();
        compare_pts(t.transform_pt(ez),Point3D::new(sq2,0.,sq2)).unwrap();        

        /* VECTORS */

        let ex = Vector3D::new(1.,0.,0.);
        let ey = Vector3D::new(0.,1.,0.);
        let ez = Vector3D::new(0.,0.,1.);

        
        let t = Transform::rotate_y(90.);
        compare_vecs(t.transform_vec(ex),ez * -1.).unwrap();
        compare_vecs(t.transform_vec(ey), ey).unwrap();
        compare_vecs(t.transform_vec(ez),ex).unwrap();

        let t = Transform::rotate_y(-90.);
        compare_vecs(t.transform_vec(ex),ez).unwrap();
        compare_vecs(t.transform_vec(ey), ey).unwrap();
        compare_vecs(t.transform_vec(ez),ex*-1.).unwrap();

        let t = Transform::rotate_y(180.);
        compare_vecs(t.transform_vec(ex),ex*-1.).unwrap();
        compare_vecs(t.transform_vec(ey), ey).unwrap();
        compare_vecs(t.transform_vec(ez),ez*-1.).unwrap();

        let t = Transform::rotate_y(45.);
        let sq2 = 1./(2 as f64).sqrt();
        compare_vecs(t.transform_vec(ex),Vector3D::new(sq2, 0.,-sq2)).unwrap();
        compare_vecs(t.transform_vec(ey), ey).unwrap();
        compare_vecs(t.transform_vec(ez),Vector3D::new(sq2, 0.,sq2)).unwrap();        
    }

    #[test]
    fn test_rotate_z_transform(){
        

        /* POINTS */
        let ex = Point3D::new(1.,0.,0.);
        let ey = Point3D::new(0.,1.,0.);
        let ez = Point3D::new(0.,0.,1.);
        
        
        let t = Transform::rotate_z(90.);
        compare_pts(t.transform_pt(ex),ey).unwrap();
        compare_pts(t.transform_pt(ey), ex*-1.).unwrap();
        compare_pts(t.transform_pt(ez),ez).unwrap();

        let t = Transform::rotate_z(-90.);
        compare_pts(t.transform_pt(ex),ey*-1.).unwrap();
        compare_pts(t.transform_pt(ey), ex).unwrap();
        compare_pts(t.transform_pt(ez),ez).unwrap();

        let t = Transform::rotate_z(180.);
        compare_pts(t.transform_pt(ex),ex*-1.).unwrap();
        compare_pts(t.transform_pt(ey), ey*-1.).unwrap();
        compare_pts(t.transform_pt(ez),ez).unwrap();

        let t = Transform::rotate_z(45.);
        let sq2 = 1./(2 as f64).sqrt();
        compare_pts(t.transform_pt(ex),Point3D::new(sq2,sq2, 0.)).unwrap();
        compare_pts(t.transform_pt(ey), Point3D::new(-sq2,sq2, 0.)).unwrap();
        compare_pts(t.transform_pt(ez),ez).unwrap();        


        /* VECTORS */
        let ex = Vector3D::new(1.,0.,0.);
        let ey = Vector3D::new(0.,1.,0.);
        let ez = Vector3D::new(0.,0.,1.);

       
        let t = Transform::rotate_z(90.);
        compare_vecs(t.transform_vec(ex),ey).unwrap();
        compare_vecs(t.transform_vec(ey), ex*-1.).unwrap();
        compare_vecs(t.transform_vec(ez),ez).unwrap();

        let t = Transform::rotate_z(-90.);
        compare_vecs(t.transform_vec(ex),ey*-1.).unwrap();
        compare_vecs(t.transform_vec(ey), ex).unwrap();
        compare_vecs(t.transform_vec(ez),ez).unwrap();

        let t = Transform::rotate_z(180.);
        compare_vecs(t.transform_vec(ex),ex*-1.).unwrap();
        compare_vecs(t.transform_vec(ey), ey*-1.).unwrap();
        compare_vecs(t.transform_vec(ez),ez).unwrap();

        let t = Transform::rotate_z(45.);
        let sq2 = 1./(2 as f64).sqrt();
        compare_vecs(t.transform_vec(ex),Vector3D::new(sq2,sq2, 0.)).unwrap();
        compare_vecs(t.transform_vec(ey), Vector3D::new(-sq2,sq2, 0.)).unwrap();
        compare_vecs(t.transform_vec(ez),ez).unwrap();        
    }
}