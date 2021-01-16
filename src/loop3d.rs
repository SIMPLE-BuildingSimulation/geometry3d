

use crate::point3d::*;
use crate::vector3d::*;
use crate::segment3d::*;

#[derive(Debug, Copy, Clone)]
struct Vertex {
    pub valid: bool,
    pub position: Point3D
}

impl Vertex {
    fn new(p: Point3D)->Vertex{
        Vertex{
            valid: true,
            position: p
        }
    }
}


pub struct Loop3D{
    vertices : Vec<Vertex>,
    normal:Vector3D,    
    closed:bool,
    area:f64,
    n_valid_vertices:usize,
}



impl Loop3D {
    pub fn new()->Loop3D{
        Loop3D{
            vertices: vec![Vertex::new(Point3D::new(0.,0.,0.));0],            
            normal:Vector3D::new(0.,0.,0.,),
            closed:false,
            area:-1.0,
            n_valid_vertices:0,
        }
    }

    pub fn clone(&self)->Loop3D{
        Loop3D{
            normal:self.normal,
            closed: self.closed,
            area: self.area,
            n_valid_vertices: self.n_valid_vertices,
            vertices: self.vertices.clone(),
        }
    }




    pub fn push(&mut self,p:Point3D)->Result<usize,&'static str>{

        //check if it is closed
        if self.closed {
            return Err("Trying to add a point to a closed Loop3D");
        }

        
        // Check if we can already validate the plane.
        if !self.normal.is_zero(){
            // Normal should be there.
            let r = self.is_coplanar(p).unwrap();
            if !r {
                return Err("Trying to add a non-coplanar point to Loop3D");
            }
        }
       
        // Check if the new vertex will make the loop intersect with itself            
        let n = self.vertices.len();
        if n > 2 {
            let last_v = self.vertices[self.vertices.len()-1].position;        
            let new_edge = Segment3D::new(last_v,p);
            let mut intersect = Point3D::new(0.,0.,0.); // dummy variable
            for i in 0..n-2{
                let v = self.vertices[i].position;
                let v_p1 = self.vertices[i+1].position;
                let this_s = Segment3D::new(v,v_p1);
                // Check the point of intersection.
                let inter = new_edge.intersect(&this_s, &mut intersect);
                // If it is different from the 
                if inter {
                    return Err("Trying to push a point that would make the Loop3D intersect with itself");
                }
            }
        }

        // If all looks all right, push
        self.vertices.push(Vertex::new(p));
        self.n_valid_vertices += 1;

        
        // Then, check if it is collinear with the previous two
        let n = self.vertices.len();
        if n > 2{
            // Check the one before the new addition
            let a = self.vertices[n-3].position;
            let b = self.vertices[n-2].position;
            let c = self.vertices[n-1].position;
            
            
            if a.is_collinear(b,c).unwrap() {
                self.vertices[n-2].valid=false;
                self.n_valid_vertices -= 1;
            }                                       
        }// else, don't bother.

                
        // Calcualte the normal if we already can.
        if self.n_valid_vertices() == 3 {
            self.set_normal();
        }

        return Ok(self.vertices.len());
    }

    pub fn n_vertices(&self)->usize{
        self.vertices.len()
    }

     
    pub fn is_diagonal(&self, s:Segment3D)->bool{
        let mut inter = Point3D::new(0.,0.,0.);
        let n = self.n_vertices();
        // It cannot intercept any
        for i in 0..n+1 {
            let (a,_valid) = self.vertex(i%n).unwrap();
            let (b,_valid) = self.vertex((i+1)%n).unwrap();

            let poly_s = Segment3D::new(a,b);
            if s.intersect(&poly_s, &mut inter){
                return false;
            }                
        }
        // And the midpoint must be in the loop.
        if !self.test_point(s.midpoint()).unwrap() {
            return false;
        }
        return true;
    }

    pub fn next_valid(&self, start : & mut usize)->Point3D{
        let mut count = 0;
        let n = self.vertices.len();
        let mut i = *start%n;
        loop {
            let v = self.vertices[i%n];
            if v.valid {
                *start = i%n+1;
                return v.position;
            }
            i+=1;
            count+=1;

            if count > self.n_vertices(){
                panic!("No valid points in Loop3D");
            }
        }
    }

    pub fn n_valid_vertices(&self)->usize{            
        return self.n_valid_vertices;
    }

    pub fn vertex(&self, i:usize)->Result<(Point3D,bool), &'static str>{
        if i < self.vertices.len() {
            let v = self.vertices[i];
            return Ok((v.position,v.valid));
        }else{
            return Err("Trying to get vertex out of Loop's bounds");
        }
    }

    pub fn invalidate_vertex(&mut self, i:usize)->Result<(),String>{
        if i < self.vertices.len() {
            self.vertices[i].valid = false;
            return Ok(());
        }else{
            return Err("Trying to get vertex out of Loop's bounds".to_string());
        }
    }

    pub fn close(&mut self)->Result<(),&'static str>{
                
        // Check if we can try to close now...         
        if self.n_vertices() < 3 {
            return Err("Trying to close a Loop3D with less than 3 vertices");
        }

        // Check the last vertex for collinearity
        let n = self.vertices.len();
        let a = self.vertices[n-2].position;
        let b = self.vertices[n-1].position;
        let c = self.vertices[0].position;
        
        if a.is_collinear(b,c).unwrap() {
            // collinear. Mark the last element as invalid.
            self.vertices[n-1].valid=false;
            self.n_valid_vertices -= 1;
        }

       
        // Check if we can really close now... 
        let n_valid = self.n_valid_vertices();
        if n_valid < 3 {
            return Err("Trying to close a Loop3D with less than 3 valid vertices");
        }

        // Close
        // No need to set normal. 
        // It is done after third valid point
        //self.set_normal();
        self.closed = true;
        let _a = self.set_area().unwrap();
        return Ok(())
            
    }
    
    
    fn set_normal(&mut self){        
                        
        let mut i : usize = 0;

        let a = self.next_valid(&mut i); 
        let b = self.next_valid(&mut i);  
        let c = self.next_valid(&mut i); 

        let ab = b-a;
        let bc = c-b;
        
        self.normal = ab.cross(bc);
        self.normal.normalize();        
        
    }

    pub fn normal(&self)->Vector3D{
        self.normal
    }

    pub fn is_coplanar(&self,p:Point3D)->Result<bool,String>{
        // This should not happen, but you never know..
        if self.vertices.len() < 1{
            let msg = format!("Trying to check whether point is coplanar in a Loop3D without any vertices");
            return Err(msg);
        }
        
        if self.normal.is_zero() {
            let msg = format!("Trying to check whether point is coplanar in a Loop3D without normal");
            return Err(msg);
        }

        let first_point = self.vertices[0].position;
        let d = first_point - p;        

        return Ok((self.normal*d).abs() < 1E-10);
    }

    pub fn test_point(&self, p: Point3D)-> Result<bool,String> {
        let tiny = 1E-10;
        let n_vertex = self.n_vertices();

        let first_point = self.vertices[0].position;
        // Get the relative axes
        
        let mut xp = first_point-p;
        xp.normalize();
        let yp = self.normal.cross(xp); 
        // yp and normal are unit vectors, thus yp also

        // Check if coplanar
        let c = self.is_coplanar(p);
        match c{
            Ok(b)=>{
                if !b{
                    return Ok(false);
                }
            },
            Err(e)=>return Err(e),
        }

        

        /*
            Alciatore, D., & Miranda, R. (1995). 
            A winding number and point-in-polygon algorithm. 
            Glaxo Virtual Anatomy Project Research Report, 
            Department of Mechanical Engineering, Colorado State University.
        */

        // 1. Initialize the winding number to zero
        let mut w_number = 0 as f64; 
        let mut previous_y = 0 as f64;
        let mut previous_x = xp*(first_point - p);
        
        // 2. For each segment of the polygon
        for i in 1..(n_vertex+1) {
            let vertex = self.vertices[i%n_vertex];            

            // in case it was cleaned
            if !vertex.valid {
                continue;
            }

            // 2.1 Determine whether the segment crosses 'dir'	
            let v = vertex.position - p;

            // determine x and y
            let y = v*yp;
            let x = v*xp;
            
            if y.abs() < tiny && previous_y.abs() < tiny  {
                continue; // no rotation! Moving along the X-axis

            }else if previous_y*y < -tiny {

                // Crosses the axis
                let r = previous_x + previous_y*(x - previous_x) / (previous_y-y);
                if r > 0.0 { // if crosses on the correct side
                    if previous_y < 0.0 {
                        // crosses positive
                        w_number += 1.0;
                    }else {
                        w_number -= 1.0;
                    }
                }
            }else if previous_y.abs() < tiny && previous_x > 0.0 {
                // previous V was on the positive X-axis
                if y > 0.0 {
                    w_number += 0.5;
                }else {
                    w_number -= 0.5;
                }
            }else if y.abs() < tiny && x > 0.0 {
                // current V is on positive X-axis
                if previous_y < 0.0 {
                    w_number += 0.5;
                }else {
                    w_number -= 0.5;
                }
            }
            previous_y = y;
            previous_x = x;
        }

        return Ok(w_number.abs() > tiny);        

    }// end of test_point
    
    fn set_area(&mut self)->Result<f64,String>{
        
        if !self.closed {
            let msg = format!("Trying to calculate the area of a Loop3D that is not closed");
            return Err(msg);
        }

        if self.normal.is_zero(){
            let msg = format!("Trying to calculate the area of a Loop3D with Zero normal");
            return Err(msg);
        }

        
        // Set the right hand side vector
        let mut rhs = Vector3D::new(0.0,0.0,0.0);                

        let n = self.n_valid_vertices();  
        if n < 3 {
            let msg = format!("Trying to calculate the area of a Loop3D with less than three valid vertices");
            return Err(msg);
        }      

        // We need to go around from 0 to N vertices,
        // ... so, n+1 valid vertices.        
        let mut i = 0; // inded in real vertices
        let mut v = self.next_valid(& mut i).vector3d();
        let mut v_p1 = self.next_valid(& mut i).vector3d();
        for _i in 2..n+2 {
            rhs += v.cross(v_p1);
            v = v_p1;
            v_p1 = self.next_valid(& mut i).vector3d();
        }
        

        let area = self.normal*rhs/2.0;  
        if area < 0. {
            self.normal *= -1.;
        }      
        self.area = area.abs();
        return Ok(self.area);
    }

    pub fn area(&self)->f64{                
        return self.area;
    }

    pub fn is_closed(&self) -> bool{
        self.closed
    }
}





/***********/
/* TESTING */
/***********/



#[cfg(test)]
mod testing{
    use super::*;

    #[test]
    fn test_new(){
        let l = Loop3D::new();
        assert_eq!(l.vertices.len(),0);
    }

    #[test]
    fn test_push(){
        let mut l = Loop3D::new();
        l.push(Point3D::new(1.,2.,3.)).unwrap();

        assert_eq!(l.vertices.len(),1);

        assert_eq!(l.vertices[0].position.x(),1.);
        assert_eq!(l.vertices[0].position.y(),2.);
        assert_eq!(l.vertices[0].position.z(),3.);


        // n_vertices
        assert_eq!(l.n_vertices(),1);
        assert_eq!(l.n_valid_vertices(),1);

        l.push(Point3D::new(4.,5.,6.)).unwrap();
        assert_eq!(l.n_vertices(),2);
        assert_eq!(l.n_valid_vertices(),2);

        // vertex
        let mut result = l.vertex(0);
        match result{
            Ok(r) => {
                let (p,_valid)=r;
                assert_eq!(p.x(),1.);
                assert_eq!(p.y(),2.);
                assert_eq!(p.z(),3.);
            },
            Err(e) =>{
                panic!(e);
            }
        }

        result = l.vertex(1);
        match result{
            Ok(r) => {
                let (p,_valid)=r;
                assert_eq!(p.x(),4.);
                assert_eq!(p.y(),5.);
                assert_eq!(p.z(),6.);
            },
            Err(e) =>{
                panic!(e);
            }
        }

        result = l.vertex(2);
        match result{
            Ok(_p) => {
                assert!(false)
            },
            Err(_e) =>{
                assert!(true)// expected to fail
            }
        }


        // Another one.
        let mut l = Loop3D::new();
        let result = l.push(Point3D::new(-2., -2., 0.)); // 0
        match result {
            Ok(_n) => {assert!(true)},
            Err(e)=>{panic!(e)}
        }
        let result = l.push(Point3D::new(0.,  -2., 0.));  // 1 -- collinear point
        match result {
            Ok(_n) => {assert!(true)},
            Err(e)=>{panic!(e)}
        }
        let result = l.push(Point3D::new(2.,  -2., 0.));  // 2
        match result {
            Ok(_n) => {assert!(true)},
            Err(e)=>{panic!(e)}
        }
        let result = l.push(Point3D::new(2.,   2., 0.));   // 3
        match result {
            Ok(_n) => {assert!(true)},
            Err(e)=>{panic!(e)}
        }
        let result = l.push(Point3D::new(-2.,  2., 0.));  // 4
        match result {
            Ok(_n) => {assert!(true)},
            Err(e)=>{panic!(e)}
        }

        l.close().unwrap();

        for i in 0..l.n_vertices() {
            let vertex = l.vertices.get(i).unwrap();
                                    
            if i == 1 {
                // Test collinear point... should not be valid
                // after clean
                assert!(!vertex.valid)    
            }else {
                assert!(vertex.valid);
            }
        }

        assert_eq!(l.n_vertices(),5);
        assert_eq!(l.n_valid_vertices(),4);


        let mut l = Loop3D::new();
        l.push(Point3D::new(-2., -2., 0.)).unwrap(); // 0 collinear
        l.push(Point3D::new(2., -2., 0.)).unwrap(); // 1
        l.push(Point3D::new(2., 2., 0.)).unwrap(); // 2        
        l.push(Point3D::new(-2., 2., 0.)).unwrap(); // 3 collinear point
        l.push(Point3D::new(-2., 1., 0.)).unwrap(); // 4. collinear point... removed
        l.push(Point3D::new(-2., 0., 0.)).unwrap(); // 5. collinear point... removed
        l.push(Point3D::new(-2., -1., 0.)).unwrap(); // 6. collinear point... removed

        l.close().unwrap();
        assert_eq!(l.n_valid_vertices(),4);

        for i in 0..l.n_vertices() {
            let vertex = l.vertices.get(i).unwrap();

            if i > 3 {
                // Test collinear point
                assert!(!vertex.valid);
            }
            else {
                assert!(vertex.valid);
            }
        }

        // INTERSECT WITH ITSELF.
        
        let mut l = Loop3D::new();
        assert!(l.push(Point3D::new(-2., -2., 0.)).is_ok()); // 0 collinear
        assert!(l.push(Point3D::new(2., 2., 0.)).is_ok()); // 1
        assert!(l.push(Point3D::new(-2., 2., 0.)).is_ok()); // 2        

        // This should fail.
        assert!(l.push(Point3D::new(2., -2., 0.)).is_err()); // 3        
        




    }// end of test_push

    #[test]
    fn test_is_coplanar(){
        
        let mut l = Loop3D::new();
        l.push(Point3D::new(-2., -2., 0.)).unwrap(); // 0
        l.push(Point3D::new(2., -2., 0.)).unwrap(); // 1
        l.push(Point3D::new(2., 2., 0.)).unwrap(); // 2        
        l.push(Point3D::new(-2., 2., 0.)).unwrap(); // 3
        l.close().unwrap();

        let r = l.is_coplanar(Point3D::new(0.,0.,0.));
        match r {
            Ok(b)=>{assert!(b)},
            Err(e)=>panic!(e)
        }
        
        let r = l.is_coplanar(Point3D::new(0.,0.,10.1));
        match r {
            Ok(b)=>{assert!(!b)},
            Err(e)=>panic!(e)
        }

    }

    #[test]
    fn test_point_convex_loop_interior(){
        //let normal = Vector3D::new(0., 0., 1.);
        let mut the_loop = Loop3D::new();
        let l = 1. / (2 as f64).sqrt();
        the_loop.push( Point3D::new(-l, -l, 0.)).unwrap();
        the_loop.push( Point3D::new(l, -l, 0.)).unwrap();
        the_loop.push( Point3D::new(l, l, 0.)).unwrap();
        the_loop.push( Point3D::new(-l, l, 0.)).unwrap();
        the_loop.close().unwrap();

        let r = the_loop.test_point(Point3D::new(0.,0.,0.)).unwrap();
        assert!(r);	
    }

    #[test]
    fn test_point_convex_loop_exterior(){
        //Vector3D normal = Vector3D(0, 0, 1);
        let mut the_loop = Loop3D::new();
        let l = 1. / (2 as f64).sqrt();
        the_loop.push( Point3D::new(-l, -l, 0.)).unwrap();
        the_loop.push( Point3D::new(l, -l, 0.)).unwrap();
        the_loop.push( Point3D::new(l, l, 0.)).unwrap();
        the_loop.push( Point3D::new(-l, l, 0.)).unwrap();
        the_loop.close().unwrap();

        let r = the_loop.test_point(Point3D::new(-10., 0., 0.)).unwrap();
        assert!(!r);
    }

    #[test]
    fn test_point_concave_loop_exterior1(){
        //Vector3D normal = Vector3D(0, 0, 1);
        let mut the_loop = Loop3D::new();

        let l = 1.0 / (2 as f64).sqrt();
        let bigl = 2. / (2 as f64).sqrt();

        the_loop.push( Point3D::new(-bigl, -bigl, 0.)).unwrap();
        the_loop.push( Point3D::new(0.0, -bigl, 0.)).unwrap(); // collinear point
        the_loop.push( Point3D::new(bigl, -bigl, 0.)).unwrap();
        the_loop.push( Point3D::new(bigl, bigl, 0.)).unwrap();
        the_loop.push( Point3D::new(-bigl, bigl, 0.)).unwrap();
        the_loop.push( Point3D::new(-bigl, -bigl, 0.)).unwrap();

        the_loop.push( Point3D::new(-l, -l, 0.)).unwrap();
        the_loop.push( Point3D::new(-l, l, 0.)).unwrap();
        the_loop.push( Point3D::new(l, l, 0.)).unwrap();
        the_loop.push( Point3D::new(l, -l, 0.)).unwrap();
        the_loop.push( Point3D::new(-l, -l, 0.)).unwrap();

        the_loop.close().unwrap();

        let r = the_loop.test_point(Point3D::new(0., 0., 0.)).unwrap();
        assert!(!r);	
    }

    #[test]
    fn test_point_concave_loop_exterior2(){
        //Vector3D normal = Vector3D(0, 0, 1);
        let mut the_loop = Loop3D::new();
        
        let l = 1. / (2 as f64).sqrt();
        let bigl = 2. / (2 as f64).sqrt();

        the_loop.push( Point3D::new(-bigl, -bigl, 0.)).unwrap();
        the_loop.push( Point3D::new(0.0, -bigl, 0.)).unwrap(); // collinear point
        the_loop.push( Point3D::new(bigl, -bigl, 0.)).unwrap();
        the_loop.push( Point3D::new(bigl, bigl, 0.)).unwrap();
        the_loop.push( Point3D::new(-bigl, bigl, 0.)).unwrap();
        the_loop.push( Point3D::new(-bigl, -bigl, 0.)).unwrap();

        the_loop.push( Point3D::new(-l, -l, 0.)).unwrap();
        the_loop.push( Point3D::new(-l, l, 0.)).unwrap();
        the_loop.push( Point3D::new(l, l, 0.)).unwrap();
        the_loop.push( Point3D::new(l, -l, 0.)).unwrap();
        the_loop.push( Point3D::new(-l, -l, 0.)).unwrap();

        the_loop.close().unwrap();

        let r = the_loop.test_point(Point3D::new(-10., 0., 0.)).unwrap();
        assert!(!r);	
    }

    #[test]
    fn test_point_concave_loop_interior(){
        //Vector3D normal = Vector3D(0, 0, 1);
        let mut the_loop = Loop3D::new();
        let l = 1. / (2 as f64).sqrt();
        let bigl = 2. / (2 as f64).sqrt();

        the_loop.push( Point3D::new(-bigl, -bigl, 0.)).unwrap();
        the_loop.push( Point3D::new(0., -bigl, 0.)).unwrap(); // collinear point
        the_loop.push( Point3D::new(bigl, -bigl, 0.)).unwrap();
        the_loop.push( Point3D::new(bigl, bigl, 0.)).unwrap();
        the_loop.push( Point3D::new(-bigl, bigl, 0.)).unwrap();
        the_loop.push( Point3D::new(-bigl, -bigl, 0.)).unwrap();

        the_loop.push( Point3D::new(-l, -l, 0.)).unwrap();
        the_loop.push( Point3D::new(-l, l, 0.)).unwrap();
        the_loop.push( Point3D::new(l, l, 0.)).unwrap();
        the_loop.push( Point3D::new(l, -l, 0.)).unwrap();
        the_loop.push( Point3D::new(-l, -l, 0.)).unwrap();

        the_loop.close().unwrap();

        let r = the_loop.test_point(Point3D::new(-1.5/(2 as f64).sqrt(), -1.5 / (2 as f64).sqrt(), 0.)).unwrap();
        assert!(r);
    }

    #[test]
    fn test_point_concave_loop_interior_with_clean(){
        //Vector3D normal = Vector3D(0, 0, 1);
        let mut the_loop = Loop3D::new();
        let l = 1. / (2 as f64).sqrt();
        let bigl = 2. / (2 as f64).sqrt();

        the_loop.push( Point3D::new(-bigl, -bigl, 0.)).unwrap();
        the_loop.push( Point3D::new(0., -bigl, 0.)).unwrap(); // collinear point
        the_loop.push( Point3D::new(bigl, -bigl, 0.)).unwrap();
        the_loop.push( Point3D::new(bigl, bigl, 0.)).unwrap();
        the_loop.push( Point3D::new(-bigl, bigl, 0.)).unwrap();
        the_loop.push( Point3D::new(-bigl, -bigl, 0.)).unwrap();

        the_loop.push( Point3D::new(-l, -l, 0.)).unwrap();
        the_loop.push( Point3D::new(-l, l, 0.)).unwrap();
        the_loop.push( Point3D::new(l, l, 0.)).unwrap();
        the_loop.push( Point3D::new(l, -l, 0.)).unwrap();
        the_loop.push( Point3D::new(-l, -l, 0.)).unwrap();

        the_loop.close().unwrap();

        let r = the_loop.test_point(Point3D::new(-1.5 / (2 as f64).sqrt(), -1.5 / (2 as f64).sqrt(), 0.)).unwrap();
        assert!(r);
    }

    #[test]
    fn test_area_1(){
        //Vector3D normal = Vector3D(0, 0, 1);
        let mut the_loop = Loop3D::new();
        let l = 1. / (2 as f64);        

        the_loop.push( Point3D::new(-l, -l, 0.)).unwrap();
        the_loop.push( Point3D::new(-l, l, 0.)).unwrap();
        the_loop.push( Point3D::new(l, l, 0.)).unwrap();
        the_loop.push( Point3D::new(l, -l, 0.)).unwrap();        

        the_loop.close().unwrap();

        let a = the_loop.area();        
        assert_eq!(4.*l*l,a);
    }

    #[test]
    fn test_area_2(){        
        let l = 1.;        
        
        // Counterclock wise
        let mut the_loop = Loop3D::new();
        the_loop.push( Point3D::new(l, 0., 0.)).unwrap(); //1
        the_loop.push( Point3D::new(l, l, 0.)).unwrap(); //2
        the_loop.push( Point3D::new(2.0*l, l, 0.)).unwrap(); //3
        the_loop.push( Point3D::new(2.0*l, 2.0*l, 0.)).unwrap(); //4
        the_loop.push( Point3D::new(0.0, 2.0*l, 0.)).unwrap(); //5
        the_loop.push( Point3D::new(0., 0., 0.)).unwrap(); //0
        
        the_loop.close().unwrap();
    
        let a = the_loop.area();        
        assert_eq!(3.0,a);
        assert!(the_loop.normal().compare(Vector3D::new(0.,0.,1.)));

        // Clockwise
        let l = 1.;        
        
        // Counterclock wise
        let mut the_loop = Loop3D::new();
        the_loop.push( Point3D::new(l, 0., 0.)).unwrap(); //1
        the_loop.push( Point3D::new(0., 0., 0.)).unwrap(); //0
        the_loop.push( Point3D::new(0.0, 2.0*l, 0.)).unwrap(); //5
        the_loop.push( Point3D::new(2.0*l, 2.0*l, 0.)).unwrap(); //4
        the_loop.push( Point3D::new(2.0*l, l, 0.)).unwrap(); //3
        the_loop.push( Point3D::new(l, l, 0.)).unwrap(); //2
        
        the_loop.close().unwrap();
    
        let a = the_loop.area();        
        assert_eq!(3.0,a);
        assert!(the_loop.normal().compare(Vector3D::new(0.,0.,-1.)));
    }

    #[test]
    fn test_close(){
        let l = 1.;        
        
        // Two elements... cannot close
        let mut the_loop = Loop3D::new();
        the_loop.push( Point3D::new(0., 0., 0.)).unwrap();
        the_loop.push( Point3D::new(l, 0., 0.)).unwrap();
        assert!(the_loop.close().is_err());

        
        // Three elements... can close
        let mut the_loop = Loop3D::new();
        the_loop.push( Point3D::new(0., 0., 0.)).unwrap();
        the_loop.push( Point3D::new(l, 0., 0.)).unwrap();
        the_loop.push( Point3D::new(0., l, 0.)).unwrap();
        assert!(the_loop.close().is_ok());

        // Three elements, but in the same line... cannot close
        let mut the_loop = Loop3D::new();
        the_loop.push( Point3D::new(0., 0., 0.)).unwrap();
        the_loop.push( Point3D::new(l, 0., 0.)).unwrap();
        the_loop.push( Point3D::new(2.0*l, 0., 0.)).unwrap();
        assert!(!the_loop.close().is_ok());


        // four elements, two in the same line... can close
        let mut the_loop = Loop3D::new();
        the_loop.push( Point3D::new(0., 0., 0.)).unwrap();
        the_loop.push( Point3D::new(l, 0., 0.)).unwrap();
        the_loop.push( Point3D::new(2.0*l, 0., 0.)).unwrap();
        the_loop.push( Point3D::new(0., l, 0.)).unwrap();
        assert!(the_loop.close().is_ok());


    }

    #[test]
    fn test_next_valid(){
        let mut l = Loop3D::new();
        l.push(Point3D::new(-2., -2., 0.)).unwrap(); // 0 collinear
        l.push(Point3D::new(2., -2., 0.)).unwrap(); // 1
        l.push(Point3D::new(2., 2., 0.)).unwrap(); // 2        
        l.push(Point3D::new(-2., 2., 0.)).unwrap(); // 3 collinear point
        l.push(Point3D::new(-2., 1., 0.)).unwrap(); // 4. collinear point... removed
        l.push(Point3D::new(-2., 0., 0.)).unwrap(); // 5. collinear point... removed
        l.push(Point3D::new(-2., -1., 0.)).unwrap(); // 6. collinear point... removed

        l.close().unwrap();
        assert_eq!(l.n_valid_vertices(),4);

        let mut i : usize = 0;        
        let mut p : Point3D;        

        p = l.next_valid(& mut i);
        assert_eq!(i,1);
        assert!(p.compare(Point3D::new(-2.,-2.,0.)));
        
        p = l.next_valid(& mut i);
        assert_eq!(i,2);
        assert!(p.compare(Point3D::new(2.,-2.,0.)));

        p = l.next_valid(& mut i);
        assert_eq!(i,3);
        assert!(p.compare(Point3D::new(2.,2.,0.)));

        p = l.next_valid(& mut i);
        assert_eq!(i,4);
        assert!(p.compare(Point3D::new(-2.,2.,0.)));

        p = l.next_valid(& mut i);
        assert_eq!(i,1);
        assert!(p.compare(Point3D::new(-2.,-2.,0.)));
    }


}