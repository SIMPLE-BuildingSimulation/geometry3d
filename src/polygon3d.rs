use crate::vector3d::*;
use crate::loop3d::*;
use crate::point3d::*;

pub struct Polygon3D{
    outer: Loop3D,
    inner: Vec<Loop3D>,
    area:f64,
    normal: Vector3D
}

impl Polygon3D{
    pub fn new(outer: Loop3D )->Result<Polygon3D,&'static str>{

        if !outer.is_closed() {
            return Err("Trying to create a Polygon3D from a loop that is not closed");
        }
        
        let area = outer.area();
        let normal = outer.normal();

        Ok(Polygon3D{
            outer: outer,
            inner: Vec::new(),
            area: area,
            normal: normal,
        })        
    }

    pub fn outer(&self)->&Loop3D{
        &self.outer
    }

    pub fn n_inner_loops(&self) -> usize {
        self.inner.len()
    }

    pub fn inner(&self, i:usize)->Result<&Loop3D,String>{
        if i < self.inner.len(){
            return Ok(&self.inner[i]);
        }        
        let msg = format!("Index out of bounds when trying to retrieve inner loop");
        return Err(msg);
    }

    pub fn test_point(&self, p:Point3D) -> Result<bool,String> {
        
        // Must be within the outer loop
        let result = self.outer.test_point(p);        
        let is_in = match result{
            Ok(b) => b ,
            Err(e) => return Err(e),
        };
        // If it is not in the outer loop, then it is outside.
        if !is_in {
            return Ok(false);
        }        

        // And outside all inner loops.
        for i in 0..self.inner.len(){
            let lp = &self.inner[i];

            let result = lp.test_point(p);        

            let is_in = match result{
                Ok(b) => b ,
                Err(e) => return Err(e),
            };
            // if it is in one of the inner loops, then it is outside
            // the shape.
            if is_in {
                return Ok(false);
            } 
        }
        // If it was not in any of the inner loops, then it is in.
        Ok(true)
    }

    
    pub fn cut_hole(&mut self, hole:Loop3D) -> Result<(),String>{
        // Check that the normals are the same
        
        if !self.normal.is_parallel(hole.normal()) {            
            let msg = format!("The hole you are trying to cut and the Polygon3D that should receive it do not have parallel normals");
            return Err(msg);
        }
        

        // Check that all the points are inside the polygon.point3d
        for i in 0..hole.n_vertices(){
            let r = hole.vertex(i);
            let p = match r {
                Ok(point_valid) => {
                    let (point,_valid) = point_valid;
                    point
                }
                Err(e) => return Err(e.to_string()),
            };
            let r = self.test_point(p);
            let is_in = match r{
                Ok(x) => x,
                Err(e) => return Err(e.to_string()),
            };
            if !is_in {
                let msg = format!("At least one of the points in your hole are not inside the polygon");
                return Err(msg);
            }
        }


        // Check that this hole does not contain another hole in it.        
        let n_inner = self.inner.len();        
        for n_loop in 0..n_inner{
            let inner = &self.inner[n_loop];

            let mut i : usize = 0;
            while i < inner.n_valid_vertices(){
                // get point from inner loop
                let inner_p = inner.next_valid(&mut i);
                
                // Check point in the hole we are making
                let is_inside = match hole.test_point(inner_p) {
                    Ok(b) => b,
                    Err(e) => return Err(e),
                };

                if is_inside {
                    let msg = format!("Apparently another hole in your Polygon3D would be inside the new hole you are making");
                    return Err(msg);
                }
            }
                        
        }
        
        // All good now! Add it.
        /*** */
        let hole_area = hole.area();
        
        // reduce area
        self.area -= hole_area;

        // Add it to the inner
        self.inner.push(hole);

        Ok(())
    }

    pub fn area(&self)->f64{
        self.area
    }

    pub fn get_closed_loop(&self) -> Loop3D {
        // This is a pretty terible algorithm... but I doubt
        // it is the bottleneck in my applications.

        // It is also not 100% guaranteed to work all the time, but 
        // it has proven to be pretty robust for relatively normal 
        // geometries.

        //get the number of interior loops
        let n_inner_loops = self.inner.len();
        
        //initialize the loop by cloning the current outer loop
        let mut ret_loop = Loop3D::new();
        let mut i:usize = 0;
        while ret_loop.n_vertices() < self.outer.n_valid_vertices(){
            let p = self.outer.next_valid(&mut i);
            ret_loop.push(p).unwrap();            
        }                

        // We will use this for checking whether the inner
        // loop should be reversed or not.
        let outer_normal = self.outer.normal();


        let mut processed_inner_loops : Vec<usize>= Vec::new();
        
        let mut inner_loop_id = 0;
        let mut inner_vertex_id = 0;

        // This is done once per inner loop
        for _i in 0..n_inner_loops {
            // find the minimum distance 
            // from interior to exterior
            let mut min_distance = 9E14;
            let mut min_inner_loop_id = 0;
            let mut min_ext_vertex_id = 0;
            //let mut min_int_vertex_id = 0;

            let n_ext_vertices = ret_loop.n_vertices();                        
            for j in 0..n_ext_vertices {

                let (ext_vertex, is_valid) = ret_loop.vertex(j).unwrap();
                
                // skip invalid vertices.
                if !is_valid {
                    continue;			
                }

                for k in 0..n_inner_loops {

                    // continue if already processed
                    if processed_inner_loops.contains(&k) {
                        continue;
                    }

                    let inner_loop = &self.inner[k];
                    let n_inner_vertices = inner_loop.n_vertices();
                    for l in 0..n_inner_vertices {

                        let (inner_vertex,is_valid) = inner_loop.vertex(l).unwrap();

                        if !is_valid{ 
                            continue;
                        }

                        // we work with squared distances... the result is
                        // the same but the calculation is faster.
                        let distance = ext_vertex.squared_distance(inner_vertex);
                        
                        if distance < min_distance {
                            min_distance = distance;
                            min_ext_vertex_id = j;
                            //min_int_vertex_id = l;
                            min_inner_loop_id = k;
                            inner_loop_id = k;
                            inner_vertex_id = l;
                        }
                    } //end iterating inner vertices
                } // end iterating inner loops		
            } // end iterating exterior vertices

            
            // Now, pass the inner loop to the exterior loop
            // by connecting them            
            let mut aux = Loop3D::new();

            for i in 0..n_ext_vertices {
                
                // Sequentially add all exterior vertices.
                let (ext_vertex,is_valid) = ret_loop.vertex(i).unwrap();

                if !is_valid{
                    continue;
                }

                // Add
                aux.push(ext_vertex).unwrap();
                
                // If we are in the vertex through which we want 
                // to connect the interior loop, then go inside.
                if  i == min_ext_vertex_id {			

                    // add the interior loop... adding the first 
                    // point twice (hence the +1)
                    let n_inner_loop_vertices =  self.inner[min_inner_loop_id].n_vertices();
                    let inner_normal = self.inner[min_inner_loop_id].normal();
                    
                    for j in 0..n_inner_loop_vertices+1 {                        					
                        let vertex_to_add; 
                        if outer_normal.is_same_direction(inner_normal){
                            // If both in the same direction, then we need to 
                            // add the interior in reverse.
                            vertex_to_add = (inner_vertex_id as i32 - j as i32) as usize % n_inner_loop_vertices;
                        }else{
                            vertex_to_add = (inner_vertex_id as i32 + j as i32) as usize % n_inner_loop_vertices;
                        }

                        let (inner_vertex, is_valid) = self.inner[min_inner_loop_id].vertex(vertex_to_add).unwrap();
                        if !is_valid {
                            continue;
                        }

                        let x = inner_vertex.x();
                        let y = inner_vertex.y();
                        let z = inner_vertex.z();
                        
                        aux.push(Point3D::new(x,y,z)).unwrap();
                    }

                    //return to exterior ret_loop
                    aux.push(ext_vertex).unwrap();
                    
                }
            }
                        
            ret_loop = aux;

            // flag ret_loop as processed (instead of deleting it)
            processed_inner_loops.push(inner_loop_id);		


        } // end iterating inner loops

        return ret_loop;
        
    }// end of get_closed_polygon
    
}


/***********/
/* TESTING */
/***********/



#[cfg(test)]
mod testing{
    use super::*;

    #[test]
    fn test_new(){

        // It should not work if we don't close it.
        let mut the_loop = Loop3D::new();
        let l = 2. as f64;
        the_loop.push( Point3D::new(-l, -l, 0.)).unwrap();
        the_loop.push( Point3D::new(l, -l, 0.)).unwrap();
        the_loop.push( Point3D::new(l, l, 0.)).unwrap();
        the_loop.push( Point3D::new(-l, l, 0.)).unwrap();
        
        assert!(!Polygon3D::new(the_loop).is_ok());
        
        // It should work if we close it.
        let mut the_loop = Loop3D::new();
        let l = 2. as f64;
        the_loop.push( Point3D::new(-l, -l, 0.)).unwrap();
        the_loop.push( Point3D::new(l, -l, 0.)).unwrap();
        the_loop.push( Point3D::new(l, l, 0.)).unwrap();
        the_loop.push( Point3D::new(-l, l, 0.)).unwrap();
        the_loop.close().unwrap();

        let poly = Polygon3D::new(the_loop).unwrap();
        assert_eq!(2.*l*2.*l,poly.area);                    
        assert!(!poly.normal.is_zero());
    }

    #[test]
    fn test_test_cut_hole(){

        // A square with the center at the origin.
        /*****/
        let mut outer_loop = Loop3D::new();
        let l = 2. as f64;
        outer_loop.push( Point3D::new(-l, -l, 0.)).unwrap();
        outer_loop.push( Point3D::new(l, -l, 0.)).unwrap();
        outer_loop.push( Point3D::new(l, l, 0.)).unwrap();
        outer_loop.push( Point3D::new(-l, l, 0.)).unwrap();
        outer_loop.close().unwrap();

        
        let mut poly = Polygon3D::new(outer_loop).unwrap();
        assert_eq!(2.*l*2.*l,poly.area);
        assert_eq!(poly.inner.len(),0);

        // Add hole
        let mut hole = Loop3D::new();
        let l = 2. as f64;
        hole.push( Point3D::new(-l/2., -l/2., 0.)).unwrap();
        hole.push( Point3D::new(l/2., -l/2., 0.)).unwrap();
        hole.push( Point3D::new(l/2., l/2., 0.)).unwrap();
        hole.push( Point3D::new(-l/2., l/2., 0.)).unwrap();
        hole.close().unwrap();

        poly.cut_hole(hole).unwrap();

        assert_eq!(poly.area, 2.*l*2.*l - l*l);
        assert_eq!(poly.inner.len(),1);


        /* Add hole in a different plane */
        /*****/
        
        let mut outer_loop = Loop3D::new();
        let l = 2. as f64;
        outer_loop.push( Point3D::new(-l, -l, 0.)).unwrap();
        outer_loop.push( Point3D::new(l, -l, 0.)).unwrap();
        outer_loop.push( Point3D::new(l, l, 0.)).unwrap();
        outer_loop.push( Point3D::new(-l, l, 0.)).unwrap();
        outer_loop.close().unwrap();
        
        let mut poly = Polygon3D::new(outer_loop).unwrap();
        
        let mut hole = Loop3D::new();
        let l = 2. as f64;
        hole.push( Point3D::new(-l/2., -l/2., 0.)).unwrap();
        hole.push( Point3D::new(l/2., -l/2., 0.)).unwrap();
        hole.push( Point3D::new(0., l/2., l)).unwrap();        
        hole.close().unwrap();

        // should fail        
        assert!(!poly.cut_hole(hole).is_ok());


        /* Add hole that contains another whole */
        /*****/
        
        let mut outer_loop = Loop3D::new();
        let l = 2. as f64;
        outer_loop.push( Point3D::new(-l, -l, 0.)).unwrap();
        outer_loop.push( Point3D::new(l, -l, 0.)).unwrap();
        outer_loop.push( Point3D::new(l, l, 0.)).unwrap();
        outer_loop.push( Point3D::new(-l, l, 0.)).unwrap();
        outer_loop.close().unwrap();        
        let mut poly = Polygon3D::new(outer_loop).unwrap();
        

        let mut hole = Loop3D::new();
        let l = 2. as f64;
        hole.push( Point3D::new(-l/2., -l/2., 0.)).unwrap();
        hole.push( Point3D::new(l/2., -l/2., 0.)).unwrap();
        hole.push( Point3D::new(l/2., l/2., 0.)).unwrap();
        hole.push( Point3D::new(-l/2., l/2., 0.)).unwrap();
        hole.close().unwrap();
        poly.cut_hole(hole).unwrap();


        let mut hole = Loop3D::new();
        let l = 2. as f64;
        hole.push( Point3D::new(-l/1.5, -l/1.5, 0.)).unwrap();
        hole.push( Point3D::new(l/1.5, -l/1.5, 0.)).unwrap();
        hole.push( Point3D::new(l/1.5, l/1.5, 0.)).unwrap();
        hole.push( Point3D::new(-l/1.5, l/1.5, 0.)).unwrap();
        hole.close().unwrap();

        // should fail        
        assert!(!poly.cut_hole(hole).is_ok());
                
    }

    #[test]
    fn test_get_closed_loop(){
        
        
        
        let mut outer = Loop3D::new();

        outer.push(Point3D::new(-2., -2., 0.)).unwrap();
        outer.push(Point3D::new( 6., -2., 0.)).unwrap();
        outer.push(Point3D::new( 6.,  6., 0.)).unwrap();
        outer.push(Point3D::new(-2.,  6., 0.)).unwrap();

        outer.close().unwrap();
        
        let mut p = Polygon3D::new(outer).unwrap();

        
        let mut inner = Loop3D::new();
        inner.push(Point3D::new(-1., -1., 0.)).unwrap(); // 3
        inner.push(Point3D::new( 1., -1., 0.)).unwrap(); // 2
        inner.push(Point3D::new( 1.,  1., 0.)).unwrap(); // 1
        inner.push(Point3D::new(-1.,  1., 0.)).unwrap(); // 0
        inner.close().unwrap();

        p.cut_hole(inner).unwrap();

        let closed = p.get_closed_loop();
        

        assert_eq!(closed.n_vertices(), 10);
        assert_eq!(closed.n_valid_vertices(), 10);
        
        // 0
        let (p,is_valid) = closed.vertex(0).unwrap();        
        assert!(p.compare( Point3D::new(-2., -2., 0.)));
        assert!(is_valid);

        // 1
        let (p,is_valid) = closed.vertex(1).unwrap();        
        assert!(p.compare( Point3D::new(-1., -1., 0.)));
        assert!(is_valid);

        // 2
        let (p,is_valid) = closed.vertex(2).unwrap();        
        assert!(p.compare( Point3D::new(-1.,  1., 0.)));
        assert!(is_valid);

        // 3
        let (p,is_valid) = closed.vertex(3).unwrap();        
        assert!(p.compare( Point3D::new( 1.,  1., 0.)));
        assert!(is_valid);

        // 4
        let (p,is_valid) = closed.vertex(4).unwrap();        
        assert!(p.compare( Point3D::new( 1., -1., 0.)));
        assert!(is_valid);

        // 5
        let (p,is_valid) = closed.vertex(5).unwrap();        
        assert!(p.compare( Point3D::new(-1., -1., 0.)));
        assert!(is_valid);
        
        // 6... Back to exterior
        let (p,is_valid) = closed.vertex(6).unwrap();        
        assert!(p.compare( Point3D::new(-2., -2., 0.)));
        assert!(is_valid);

        // 7.
        let (p,is_valid) = closed.vertex(7).unwrap();        
        assert!(p.compare( Point3D::new( 6., -2., 0.)));
        assert!(is_valid);

        // 8
        let (p,is_valid) = closed.vertex(8).unwrap();        
        assert!(p.compare( Point3D::new( 6.,  6., 0.)));
        assert!(is_valid);

        // 9
        let (p,is_valid) = closed.vertex(9).unwrap();        
        assert!(p.compare( Point3D::new(-2.,  6., 0.)));
        assert!(is_valid);

        

        
    }


    #[test]
    fn test_get_closed_loop_with_clean(){
        
        let mut outer = Loop3D::new();
        outer.push( Point3D::new(-2., -2., 0.)).unwrap();
        outer.push( Point3D::new(0., -2., 0.)).unwrap(); // colinear point
        outer.push( Point3D::new(6., -2., 0.)).unwrap();
        outer.push( Point3D::new(6., 6., 0.)).unwrap();
        outer.push( Point3D::new(-2., 6., 0.)).unwrap();
        outer.close().unwrap();
        
        let mut p = Polygon3D::new(outer).unwrap();
        
        
        let mut inner_loop = Loop3D::new();
        inner_loop.push( Point3D::new(1., 1., 0.)).unwrap(); // 0
        inner_loop.push( Point3D::new(1., -1., 0.)).unwrap(); // 1
        inner_loop.push( Point3D::new(0., -1., 0.)).unwrap(); // 2. colinear point
        inner_loop.push( Point3D::new(-1., -1., 0.)).unwrap(); // 3
        inner_loop.push( Point3D::new(-1., 1., 0.)).unwrap(); // 4
        inner_loop.push( Point3D::new(0., 1., 0.)).unwrap(); // 5. colinear point
        inner_loop.close().unwrap();

        p.cut_hole(inner_loop).unwrap();

        let closed = p.get_closed_loop();

        assert_eq!(closed.n_vertices(), 10);
        assert_eq!(closed.n_valid_vertices(), 10);
        
        // 0
        let (p,is_valid) = closed.vertex(0).unwrap();        
        assert!(p.compare( Point3D::new(-2., -2., 0.)));
        assert!(is_valid);

        // 1
        let (p,is_valid) = closed.vertex(1).unwrap();        
        assert!(p.compare( Point3D::new(-1., -1., 0.)));
        assert!(is_valid);

        // 2
        let (p,is_valid) = closed.vertex(2).unwrap();        
        assert!(p.compare( Point3D::new(-1.,  1., 0.)));
        assert!(is_valid);

        // 3
        let (p,is_valid) = closed.vertex(3).unwrap();        
        assert!(p.compare( Point3D::new( 1.,  1., 0.)));
        assert!(is_valid);

        // 4
        let (p,is_valid) = closed.vertex(4).unwrap();        
        assert!(p.compare( Point3D::new( 1., -1., 0.)));
        assert!(is_valid);

        // 5
        let (p,is_valid) = closed.vertex(5).unwrap();        
        assert!(p.compare( Point3D::new(-1., -1., 0.)));
        assert!(is_valid);
        
        // 6... Back to exterior
        let (p,is_valid) = closed.vertex(6).unwrap();        
        assert!(p.compare( Point3D::new(-2., -2., 0.)));
        assert!(is_valid);

        // 7.
        let (p,is_valid) = closed.vertex(7).unwrap();        
        assert!(p.compare( Point3D::new( 6., -2., 0.)));
        assert!(is_valid);

        // 8
        let (p,is_valid) = closed.vertex(8).unwrap();        
        assert!(p.compare( Point3D::new( 6.,  6., 0.)));
        assert!(is_valid);

        // 9
        let (p,is_valid) = closed.vertex(9).unwrap();        
        assert!(p.compare( Point3D::new(-2.,  6., 0.)));
        assert!(is_valid);
        
    }
    


    

}