/*
    This section was heavily influenced by
    the following source:

    Ruppert, J. (1995). A Delaunay refinement algorithm for quality 2-dimensional mesh generation. J. Algorithms, 18(3), 548-585.

*/

use crate::point3d::*;
use crate::polygon3d::*;
use crate::segment3d::*;
use crate::triangle3d::*;

#[derive(Clone,Copy, Eq, PartialEq, Debug)]
#[repr(u8)]
enum Edge{
    Ab,Bc,Ca,
}

impl Edge{
    fn from_i(i:usize)->Self{
        match i{
            0=>Self::Ab,
            1=>Self::Bc,
            2=>Self::Ca,
            _ => {panic!("Index given for Edge is out of bound (given was {})", i)}
        }
    }

    fn to_i(&self)->usize{
        match self {
            Self::Ab=>0,
            Self::Bc=>1,
            Self::Ca=>2,
        }
    }
}

impl std::ops::Add<usize> for Edge {
    type Output = Self;

    fn add(self, other: usize) -> Self {
        let i = (self.to_i() + other)%3;
        Self::from_i(i)
    }
}

struct TriPiece{
    triangle:Triangle3D,

    // Segments
    // s0: Segment3D,
    // s1: Segment3D,
    // s2: Segment3D,

    // Neighbours
    n0: Option<usize>,
    n1: Option<usize>,
    n2: Option<usize>,

    // Constraints
    c0: bool,
    c1: bool,
    c2: bool,

    // Other info about the triangle.
    
    // circumradius: f64,
    aspect_ratio: f64,
    circumcenter: Point3D,
    centroid: Point3D,
    
    valid: bool,

    // This will be
    // assigned when pushing it into a
    // triangulation.
    index: usize,
}

/// A set of [`Triangle3D`] that, together, cover completely
/// a [`Polygon3D`].
pub struct Triangulation3D {
    /// The triangles
    triangles: Vec<TriPiece>,

    /// The number of valid triangles.
    ///
    /// Check whether triangles are valid or not
    /// before using them. Invalidating triangles help
    /// accelerate the algorithms in this module.
    n_valid_triangles: usize,
}

impl TriPiece{

    pub fn new(
        vertex_a: Point3D,
        vertex_b: Point3D,
        vertex_c: Point3D,
        i: usize,
    ) -> Result<Self, String> {
      
        // Build the triangle
        let triangle = Triangle3D::new(vertex_a,vertex_b,vertex_c)?;

        let t = Self {
            
            // s0: Segment3D::new(vertex_a, vertex_b),
            // s1: Segment3D::new(vertex_b, vertex_c),
            // s2: Segment3D::new(vertex_c, vertex_a),

            n0: None,
            n1: None,
            n2: None,

            c0: false,
            c1: false,
            c2: false,

            aspect_ratio: triangle.aspect_ratio(),
            // circumradius: triangle.circumradius(),
            circumcenter: triangle.circumcenter(),
            centroid: triangle.centroid(),            

            index: i,
            valid: true,
            triangle,
        };

        

        Ok(t)
    }

    
    
    fn invalidate(&mut self) {
        self.valid = false
    }
    
    fn set_neighbour(&mut self, edge: Edge, i: usize) {
        match edge {
            Edge::Ab => self.n0 = Some(i),
            Edge::Bc => self.n1 = Some(i),
            Edge::Ca => self.n2 = Some(i)
        }
    }

    pub fn neighbour(&self, edge: Edge) -> Option<usize> {
        match edge {
            Edge::Ab => self.n0 ,
            Edge::Bc => self.n1 ,
            Edge::Ca => self.n2 ,
        }
    }


    pub fn constrain(&mut self, edge: Edge) {
        match edge {
            Edge::Ab => self.c0 = true,
            Edge::Bc => self.c1 = true,
            Edge::Ca => self.c2 = true,            
        }
    }

    pub fn is_constrained(&self, edge:Edge) -> bool {
        match edge {
            Edge::Ab => self.c0,
            Edge::Bc => self.c1,
            Edge::Ca => self.c2,            
        }        
    }
}


impl Default for Triangulation3D {
    fn default() -> Self {
        Self::new()
    }
}


impl Triangulation3D {
    /// Creates  a new empty [`Triangulation3D`]
    pub fn new() -> Triangulation3D {
        Triangulation3D {
            n_valid_triangles: 0,
            triangles: Vec::new(),
        }
    }

    
    /// Creates  a new empty [`Triangulation3D`] with a certain capacity
    pub fn with_capacity(i:usize) -> Triangulation3D {
        Triangulation3D {
            n_valid_triangles: 0,
            triangles: Vec::with_capacity(i),
        }
    }
    

    /// Triangulates a [`Polygon3D`] using the simple ear
    /// clipping algorithm and then progresively refines the mesh in order to get
    /// a relatively healthy set of Delaunay triangles
    pub fn mesh_polygon(
        poly: &Polygon3D,
        max_area: f64,
        max_aspect_ratio: f64,
    ) -> Result<Triangulation3D, String> {
        println!("Mesh polygon");
        let mut t = Self::from_polygon(poly)?;
        t.refine(max_area, max_aspect_ratio)?;
        Ok(t)
    }

    /// Loops over all [`Triangle3D`], setting the neighbours. THis is 
    /// quite slow, but does the job.
    fn mark_neighbourhouds(&mut self)->Result<(),String>{
        let n = self.triangles.len();
        for this_i in 0..n{
            for other_i in this_i+1..n{
                

                for edge_i in 0..3{
                    let edge = self.triangles[this_i].triangle.segment(edge_i).unwrap();
                    
                    
                    if let Some(_) = self.triangles[other_i].triangle.get_edge_index_from_segment(&edge) {
                        self.mark_as_neighbours(this_i, Edge::from_i(edge_i), other_i)?;
                        break; // dont test other edges
                    }
                }
            }
        }
        Ok(())
    }

    /// Triangulates a [`Polygon3D`] without refining it, using the simple ear
    /// clipping algorithm.
    pub fn from_polygon(poly: &Polygon3D) -> Result<Triangulation3D, String> {
        
        let mut the_loop = poly.get_closed_loop();
        // This is a theorem, apparently.
        let mut t = Triangulation3D::with_capacity(the_loop.n_vertices() - 2);

        let mut count = 0; // keep note of the number of triangles
        let mut i = 0;
        loop {
            let n = the_loop.n_vertices();
            if n == 2 {                
                // return
                t.mark_neighbourhouds()?;
                return Ok(t);
            }
            
            let v0 = the_loop[i%n];            
            let v1 = the_loop[(i+1)%n];
            let v2 = the_loop[(i+2)%n];

            let potential_diag = Segment3D::new(v0, v2);
            if the_loop.is_diagonal(potential_diag) {
                // Do not add (i.e., just drop) ears that are too small.
                if potential_diag.length()>0.0001{
                    // Add triangle
                    t.push(v0, v1, v2, count).unwrap();
    
                    // Set contrain... there must be a smarter way of doing this
                    let s01 = Segment3D::new(v0,v1);
                    if poly.contains_segment(&s01){
                        t.triangles[count].constrain(Edge::from_i(0));
                    }
                    let s12 = Segment3D::new(v1,v2);
                    if poly.contains_segment(&s12){
                        t.triangles[count].constrain(Edge::from_i(1));
                    }
                    let s20 = Segment3D::new(v2,v0);
                    if poly.contains_segment(&s20){
                        t.triangles[count].constrain(Edge::from_i(2));
                    }
                }
                
                //remove v1
                the_loop.remove((i+1)%n);


                count += 1;
            }else{
                i+=1;
            }
        } //end of loop{}
    }

    /// Returns the number of [`Triangle3D`] in the [`Triangulation3D`]
    /// including invalid ones.
    pub fn n_triangles(&self) -> usize {
        self.triangles.len()
    }

    /// Returns the number of valid [`Triangle3D`] in the [`Triangulation3D`]     
    pub fn n_valid_triangles(&self) -> usize {
        self.n_valid_triangles
    }

    

    fn invalidate(&mut self, i: usize) -> Result<(), String> {
        let n = self.triangles.len();
        if i < n {
            self.triangles[i].invalidate();
            self.n_valid_triangles -= 1;
            return Ok(());
        }
        let msg = format!(
            "Trying to invalidate triangle {} in a triangulation that has only {} triangles",
            i, n
        );
        Err(msg)
    }

    fn get_first_invalid(&self, start: usize) -> Option<usize> {
        let n = self.triangles.len();
        if start < n {
            for i in start..n {
                if !self.triangles[i].valid {
                    return Some(i);
                }
            }
        }
        None
    }

    /// Marks two triangles as neighbours
    fn mark_as_neighbours(&mut self, i1: usize, edge_1: Edge, i2: usize) -> Result<(), String> {
        

        if i1 == i2 {
            let msg = format!("Trying to make a triangle its own neighbour (index {})", i1);
            return Err(msg);
        }

        
        // Check if this is valid
        if !self.triangles[i1].valid{
            return Err("Accessing invalid triangle within mark_as_neighbours()".to_string())
        }            
        // Get segment... we should panic here, as this is not a user error
        let seg1 =  &self.triangles[i1].triangle.segment(edge_1.to_i()).unwrap();

        // Check neighbour            
        if !self.triangles[i2].valid{
            return Err("Accessing invalid triangle within mark_as_neighbours()".to_string())
        }
        
        // Get edge segment from the other triangle... panic if does not work.
        let edge_2 = match self.triangles[i2].triangle.get_edge_index_from_segment(&seg1){
            Some(e)=>e,
            None => panic!("The triangles that you are trying to mark as neighbours don't share a segment.")
        };
        let edge_2 = Edge::from_i(edge_2);
        

        // We have already tested all the indexes.
        self.triangles[i1].set_neighbour(edge_1, i2);
        self.triangles[i2].set_neighbour(edge_2, i1);

        Ok(())
    }

    pub fn push(
        &mut self,
        vertex_a: Point3D,
        vertex_b: Point3D,
        vertex_c: Point3D,
        last_added: usize,
    ) -> Result<usize, String> {
        let extend: bool;
        let n = match self.get_first_invalid(last_added) {
            Some(i) => {
                extend = false;
                i
            }
            None => {
                extend = true;
                self.triangles.len()
            }
        };

        let result = TriPiece::new(vertex_a, vertex_b, vertex_c, n);
        match result {
            Ok(t) => {
                if extend {
                    self.triangles.push(t);
                } else {
                    self.triangles[n] = t;
                }
                self.n_valid_triangles += 1;
                Ok(n)
            }
            Err(e) => Err(e),
        }
    }

    /// Gets the vertex that is opposite to a [`Segment3D`] in a [`Triangle3D`]
    fn get_opposite_vertex(
        &self,
        triangle: &Triangle3D,
        segment: Segment3D,
    ) -> Result<Point3D, String> {
        

        match triangle.get_edge_index_from_segment(&segment) {
            Some(i) => match i {
                0 => Ok(triangle.vertex(2).unwrap()),
                1 => Ok(triangle.vertex(0).unwrap()),
                2 => Ok(triangle.vertex(1).unwrap()),
                _ => {
                    Err("Something really strange happened when get_opposite_vertex()".to_string())
                }
            },
            None => {
                let msg =
                    "Trying to get opposite verted in a triangle that does not have the given edge"
                        .to_string();
                Err(msg)
            }
        }
    }

    /// Returns the aspect ration that would result after flipping the diagonal of [`Triangle3D`] 
    /// (located at `index`)
    fn get_flipped_aspect_ratio(&self, index: usize, edge: Edge) -> Option<f64> {
        

        // do not modify constraints
        let tripiece = &self.triangles[index];
        if !tripiece.valid {            
            panic!("Found an invalid triangle when getting flipped aspect ratio")
        }

        if tripiece.is_constrained(edge) {
            return None
        }

        // get neighbor
        let neighbour_i = match tripiece.neighbour(edge){
            None=>{return None},
            Some(i)=>i
        };
        
        let neighbour = &self.triangles[neighbour_i];
        if !neighbour.valid {            
            panic!("Found an invalid neighbour when getting flipped aspect ratio");
        }

        if neighbour.index == tripiece.index {            
            panic!("Triangle is its own neighbor!");
        }

        // get vertices
        let vertex_a = tripiece.triangle.vertex(edge.to_i() % 3).unwrap();
        let vertex_b = tripiece.triangle.vertex((edge.to_i() + 1) % 3).unwrap();
        let vertex_c = tripiece.triangle.vertex((edge.to_i() + 2) % 3).unwrap();

        // Get the oposite side
        let s = Segment3D::new(vertex_a, vertex_b);
        let opposite = self.get_opposite_vertex(&neighbour.triangle, s).unwrap();

        if !is_convex(vertex_a, opposite, vertex_b, vertex_c) {
            return None
        }

        // get the other situation
        let flipped1 = TriPiece::new(vertex_a, opposite, vertex_c, 0).unwrap();
        let flipped2 = TriPiece::new(opposite, vertex_b, vertex_c, 0).unwrap();

        let f1_aspect = flipped1.aspect_ratio;
        let f2_aspect = flipped2.aspect_ratio;

        // Return the maximum
        if f1_aspect > f2_aspect {
            Some(f1_aspect)
        } else {
            Some(f2_aspect)
        }
    }

    fn flip_diagonal(&mut self, index: usize, edge: Edge) {
        // The transformation is as follows:
        //         C                C
        //        /\               /|\
        //     A /__\ B   -->   A / | \ B
        //       \  /             \ | /
        //        \/               \|/
        //         opposite          opposite
        //
        // The neighbours in AC, BC and so on need to be restored at the end.        

        
        // Check if valid
        if !&self.triangles[index].valid {            
            panic!("Trying to flip diagonal with an invalid triangle");
        }

        // get vertices
        let vertex_a = self.triangles[index].triangle.vertex(edge.to_i() % 3).unwrap();
        let vertex_b = self.triangles[index].triangle.vertex((edge.to_i() + 1) % 3).unwrap();
        let vertex_c = self.triangles[index].triangle.vertex((edge.to_i() + 2) % 3).unwrap();

        // get neighbour index... There needs to be one.
        let neighbour_index = match self.triangles[index].neighbour(edge) {
            None => panic!("Trying to flip diagonal of Triangle with no neighbour"),
            Some(i) => i,
        };

        // Get other neighbours... these are optional
        let ac_i = match &self.triangles[index].triangle.get_edge_index_from_points(vertex_a, vertex_c) {
            Some(i) => *i,
            None => {
                panic!("Segment AC not found on triangle when flipping diagonals.")
            }
        };
        let neighbour_ac_index = self.triangles[index].neighbour(Edge::from_i(ac_i));

        let cb_i = match &self.triangles[index].triangle.get_edge_index_from_points(vertex_c, vertex_b) {
            Some(i) => *i,
            None => {
                panic!("Segment CB not found on triangle when flipping diagonals.")
            }
        };
        let neighbour_bc_i = self.triangles[index].neighbour(Edge::from_i(cb_i));
        


        // Now process the rest...

        // check if neighbour is valid            
        if !self.triangles[neighbour_index].valid {            
            panic!("Trying to flip diagonal with an invalid neighbour triangle");
        }
        

        // Get the oposite side
        let s = Segment3D::new(vertex_a, vertex_b); 

        let opposite = self
            .get_opposite_vertex(&self.triangles[neighbour_index].triangle, s)
            .unwrap();

        // Get other neighbours... these are optional
        let bopp_i = match self.triangles[neighbour_index].triangle.get_edge_index_from_points(vertex_b, opposite) {
            Some(i) => i,
            None => {
                panic!("Segment B-Opposite not found on triangle when flipping diagonals.")
            }
        };

        let neighbour_bopp_i = self.triangles[neighbour_index].neighbour(Edge::from_i(bopp_i));

        let aopp_i = match self.triangles[neighbour_index].triangle.get_edge_index_from_points(vertex_a, opposite) {
            Some(i) => i,
            None => {
                panic!("Segment A-Opposite not found on triangle when flipping diagonals.")
            }
        };
        let neighbour_aopp_i = self.triangles[neighbour_index].neighbour(Edge::from_i(aopp_i));
        

        // invalidate both triangles.
        self.invalidate(index).unwrap();
        self.invalidate(neighbour_index).unwrap();

        // Push both triangles, in the places where
        // the original ones used to be
        self.push(vertex_a, opposite, vertex_c, index).unwrap();
        self.push(vertex_c, opposite, vertex_b, neighbour_index).unwrap();

        // Reorganize neighbours
        if let Some(ni) = neighbour_aopp_i{
            self.mark_as_neighbours(index, Edge::Ab, ni).unwrap();
        }

        self.mark_as_neighbours(index, Edge::Bc, neighbour_index).unwrap();

        if let Some(ni) = neighbour_ac_index{
            self.mark_as_neighbours(index, Edge::Ca, ni ).unwrap();
        }
        

        // done by reciprocity
        // self.mark_as_neighbours(neighbour_index, Edge::Ab, index);

        if let Some(ni) = neighbour_bopp_i{
            self.mark_as_neighbours(neighbour_index, Edge::Bc, ni).unwrap();
        }        
        if let Some(ni) = neighbour_bc_i{
            self.mark_as_neighbours(neighbour_index, Edge::Ca, ni).unwrap();
        }

    } // end of flip_diagonal()

    /// Adds a point to an edge of a [`Triangle3D`], splitting 
    /// it and its neighbour
    fn split_edge(
        &mut self,
        triangle_index: usize,
        edge_to_split: Edge,
        p: Point3D,
    ) -> Result<(), String> {        
        // We are doing the following:
        //              C                    C
        //             /\                   /|\
        //            /  \                 / | \
        //           /    \      -->      /  |  \
        //        A /______\ B         A /___|___\ B
        //          \      /             \   |p  /
        //           \    /               \  |  /
        //            \  /                 \ | /
        //             \/                   \|/
        //              opposite             opposite

        

        if !self.triangles[triangle_index].valid {
            let msg = "Trying to split edge of an invalid Triangle3D".to_string();
            return Err(msg);
        }


        // get points
        let ab = self.triangles[triangle_index].triangle.segment(edge_to_split.to_i()).unwrap();
        let vertex_a = ab.start();
        let vertex_b = ab.end();

        // Neighbour... this is not necessarily there        
        let nei_i = self.triangles[triangle_index].neighbour(edge_to_split);

        let mut process_hemisphere = |index: usize| -> (usize, usize) {
            let edge = self.triangles[index].triangle.get_edge_index_from_segment(&ab).unwrap();            
            let edge = Edge::from_i(edge);

            // let s = self.triangles[index].segment(edge).unwrap();

            // Get points
            let vertex_c = self.get_opposite_vertex(&self.triangles[index].triangle, ab).unwrap();

            // invalidate base triangle.
            self.invalidate(index).unwrap();

            // Get neighbours in AC and BC and Constrains
            let bc_n = self.triangles[index].neighbour(edge+1);
            let ac_n = self.triangles[index].neighbour(edge+2);

            let bc_c = self.triangles[index].is_constrained(edge+1);
            let ac_c = self.triangles[index].is_constrained(edge+2);
            let ab_c = self.triangles[index].is_constrained(edge+3);

            // Replace base triangle with two new triangles.
            // Create and add the new triangles.
            let apc_i = self.push(vertex_a, p, vertex_c, index).unwrap(); // replace the original with this one.
            let pbc_i = self.push(p, vertex_b, vertex_c, 0).unwrap(); // check from the start.

            // Set neighbours and constraints

            // APC TRIANGLE
            // -------------------

            // Segment 0: AP -> Don't know if there is any neighbour yet... will solve this
            // when adding the neighbour.
            // We do know, however, that it might be contrained
            if ab_c {
                self.triangles[apc_i].constrain(Edge::Ab);
            }

            // Segment 1: PC
            self.mark_as_neighbours(apc_i, Edge::Bc, pbc_i).unwrap();

            // Segment 2: CA
            if let Some(ni) = ac_n{
                self.mark_as_neighbours(apc_i, Edge::Ca, ni).unwrap();
            }             
            if ac_c {
                self.triangles[apc_i].constrain(Edge::Ca);
            }

            // PBC Triangle
            // -------------------

            // Segment 0: PB -> Don't know if there is any neighbour yet... will solve this
            // when adding the neighbour.
            // We do know, however, that it might be contrained
            if ab_c {
                self.triangles[pbc_i].constrain(Edge::Ab);
            }

            // Segment 1: BC
            if let Some(ni) = bc_n{
                self.mark_as_neighbours(pbc_i, Edge::Bc, ni).unwrap();
            }            
            if bc_c {
                self.triangles[pbc_i].constrain(Edge::Bc);
            }

            // Segment 2: CP... matches the other triangle. Reciprocity

            // Return positions of new triangles.
            (apc_i, pbc_i)
        }; // end of process_hemisphere

        // PROCESS BASE TRIANGLE
        // ==============================

        let (top_left_i, top_right_i) = process_hemisphere(triangle_index);

        // PROCESS NEIGHBOUR
        // =================

        // If there is any neighbour, process as well.
        if let Some(nei_i) = nei_i{
            let (bottom_left_i, bottom_right_i) = process_hemisphere(nei_i as usize);
    
            // Mark the upper and lower hemispheres as neighbours.
            self.mark_as_neighbours(top_left_i, Edge::Ab, bottom_left_i).unwrap();
            self.mark_as_neighbours(top_right_i, Edge::Ab, bottom_right_i).unwrap();
        }
        

        Ok(())
    }

    /// Adds a point to the interior of a [`Triangle3D`], splitting 
    /// it into three
    fn split_triangle(&mut self, i: usize, point: Point3D) -> Result<(), String> {
        // The transformation is as follows (puts P in the middle,
        // and splits the triangle into three triangles):
        //             A                A
        //             /\              /:\
        //            /  \            / : \
        //           /    \          /  x p\
        //          /      \        / .  .  \
        //       c /________\B   c /._______.\B
        //
        // The neighbours in AC, BC and AB need to be restored at the end.

                    

        if !self.triangles[i].valid {
            let msg = format!("Trying to split an invalid triangle (index {})", i);
            return Err(msg);
        }

        // get vertices
        let vertex_a = self.triangles[i].triangle.a();
        let vertex_b = self.triangles[i].triangle.b();
        let vertex_c = self.triangles[i].triangle.c();

        // Get neighbours and constraints
        let edge = self.triangles[i].triangle.get_edge_index_from_points(vertex_a, vertex_b).unwrap();
        let edge = Edge::from_i(edge);
        let neighbour_ab_i = self.triangles[i].neighbour(edge);
        let constrain_ab = self.triangles[i].is_constrained(edge);

        let edge = self.triangles[i].triangle.get_edge_index_from_points(vertex_b, vertex_c).unwrap();
        let edge = Edge::from_i(edge);
        let neighbour_bc_i = self.triangles[i].neighbour(edge);
        let constrain_bc = self.triangles[i].is_constrained(edge);

        let edge = self.triangles[i].triangle.get_edge_index_from_points(vertex_c, vertex_a).unwrap();
        let edge = Edge::from_i(edge);
        let neighbour_ca_i = self.triangles[i].neighbour(edge);
        let constrain_ca = self.triangles[i].is_constrained(edge);

        // Invalidate this triangle.
        self.invalidate(i).unwrap();
    

        // Add new triangles, noting their indices
        // Indices of the new triangles
        let apc_i = self.push(vertex_a, point, vertex_c, 0).unwrap();
        let abp_i = self.push(vertex_a, vertex_b, point, 0).unwrap();
        let cpb_i = self.push(vertex_c, point, vertex_b, 0).unwrap();

        /* FIRST TRIANGLE: CPA -- set contraints and neighbours */

        // Edge 0 -> CP
        self.mark_as_neighbours(apc_i, Edge::Ab, abp_i).unwrap();
        
        // Edge 1 -> PA
        self.mark_as_neighbours(apc_i, Edge::Bc, cpb_i).unwrap();

        // Edge 2 -> AC

        // constrain and neighbour
        if constrain_ca {
            self.triangles[apc_i].constrain(Edge::Ca);          
        } 
        if let Some(ni) = neighbour_ca_i {
            self.mark_as_neighbours(apc_i, Edge::Ca, ni).unwrap();
        } 

        /* SECOND TRIANGLE: ABP -- set contraints and neighbours */

        // Edge 0 -> AB

        // constrain
        if constrain_ab {
            self.triangles[abp_i].constrain(Edge::Ab);
        } 
        if let Some(ni) = neighbour_ab_i {
            self.mark_as_neighbours(abp_i, Edge::Ab, ni).unwrap();
        } 
        
        // Edge 1 -> BP
        self.mark_as_neighbours(abp_i, Edge::Bc, cpb_i).unwrap();
        
        // Edge 2 -> PA
        // Done by reciprocity
        //self.mark_as_neighbours(bpa_i, Edge::Bc, cpa_i).unwrap();


        /* THIRD TRIANGLE: CPB -- set contraints and neighbours */

        // Edge 0 -> CP
        // Done by reciprocity
        //self.mark_as_neighbours(cpb_i, 0, bpa_i).unwrap();

        // Edge 1 -> PB
        // Done by reciprocity
        //self.mark_as_neighbours(cpb_i, 1, cpa_i).unwrap();

        // Edge 2 -> BC

        // constrain
        if constrain_bc {
            self.triangles[cpb_i].constrain(Edge::Ca);            
        } 
        if let Some(ni) = neighbour_bc_i {
            self.mark_as_neighbours(cpb_i, Edge::Ca, ni).unwrap();
        } 

        // return
        Ok(())
    }

    fn restore_delaunay(&mut self, max_aspect_ratio: f64) {
        // Flipping diagonals does not change the number of triangles.        
        let n = self.triangles.len();

        let mut any_changes = true;
        let max_loops: usize = 15;
        let mut n_loops: usize = 0;

        while any_changes && n_loops < max_loops {
            // Count this loop
            n_loops += 1;

            // Reset changes.
            any_changes = false;

            // Loop
            for i in 0..n {
                // Skip invalids
                if !self.triangles[i].valid {
                    continue;
                }

                // get the current aspect ratio
                let current_ar = self.triangles[i].aspect_ratio;

                // If it is acceptable, skip.
                if current_ar < max_aspect_ratio {
                    continue;
                }

                // Otherwise, check if it is worth flipping the diagonal
                let mut best_edge: Option<Edge> = None;
                let mut best_aspect_ratio = f64::MAX;

                // Search in three directions (i.e. three edges)
                for j in 0..3 {
                    let this_edge = Edge::from_i(j);
                    // calculate possible aspect ratio... 
                    if let Some(ar) = self.get_flipped_aspect_ratio(i, this_edge){
                        if current_ar > ar && best_aspect_ratio > ar {
                            best_edge = Some(this_edge);
                            best_aspect_ratio = ar;
                        }
                    }                    
                }

                // Flip if best neighbour actually exists and it is better than
                // the current situation (i.e. >= 0)
                if let Some(best) = best_edge {
                    // if at least one of them is better than the current
                    self.flip_diagonal(i, best);
                    any_changes = true;
                } // else... do nothing
            }
        } // End of while any_changes
    } // end of fn restore_delaunay

    /// Adds a point on a specific [`Triangle3D`]
    fn add_point_to_triangle(
        &mut self,
        index: usize,
        point: Point3D,
        p_location: PointInTriangle,
    ) -> Result<(), String> {
        if !self.triangles[index].valid {
            let msg = "Trying to add point into an obsolete triangle".to_string();
            return Err(msg);
        }

        if p_location.is_vertex() {
            // Point is a vertex... ignore, but pretend we did something
            Ok(())
        } else if p_location.is_edge() {
            let edge = match p_location{
                PointInTriangle::EdgeAB=>Edge::Ab,
                PointInTriangle::EdgeBC=>Edge::Bc,
                PointInTriangle::EdgeAC=>Edge::Ca,
                _ =>{panic!("Trying to split edge with a point that does not fall on an edge")}
            };
            self.split_edge(index, edge, point)
        } else if p_location == PointInTriangle::Inside {
            self.split_triangle(index, point)
        } else {
            unreachable!();
        }
    }

    /// Finds the location of a [`Point3D`] within the triangulation
    /// and inserts it
    fn add_point(&mut self, point: Point3D) -> Result<(), String> {
        // Iterate through triangles to check
        
        for (i,tripiece) in self.triangles.iter().enumerate() {
            // skip triangle if it has been deleted
            if !tripiece.valid {
                continue;
            }

            let p_location = tripiece.triangle.test_point(point);
            if p_location != PointInTriangle::Outside {
                return self.add_point_to_triangle(i, point, p_location);
            }
        }

        Err(
            "No Triangle3D in Triangulation3D contained the point that you wanted to add."
                .to_string(),
        )
    }

    fn refine(&mut self, max_area: f64, max_aspect_ratio: f64) -> Result<(), String> {
        println!("Refine");
        let n_triangles = self.n_triangles();

        for i in 0..n_triangles {
            if !self.triangles[i].valid {
                continue;
            }

            let area = self.triangles[i].triangle.area();

            // if area is already too small, just ignore
            if area < 9e-3 {
                continue;
            }

            // If it is an issue of aspect ratio
            if self.triangles[i].aspect_ratio > max_aspect_ratio {
                // find the longest edge... assume it is the first one
                let mut s = self.triangles[i].triangle.ab();
                let mut s_i = 0;
                // but check the other two.
                for j in 1..3 {
                    if s.length() < self.triangles[i].triangle.segment(j).unwrap().length() {
                        s_i = j;
                        s = self.triangles[i].triangle.segment(j).unwrap();
                    }
                }
                let edge_to_split = Edge::from_i(s_i);

                // add midpoint
                let mid_s = s.midpoint();
                self.split_edge(i, edge_to_split, mid_s).unwrap();
                self.restore_delaunay(max_aspect_ratio);
            } else if area > max_area {
                // try to add the circumcenter
                let c_center = self.triangles[i].circumcenter;

                // Since the circumcenter may be in another triangle, we need
                // to search for it.
                let res = self.add_point(c_center);
                if res.is_ok() {
                    // What should I do if the circumcenter is out of the polygon?
                    // For now just add the centroid of it...
                    // This one we KNOW will be inside the polygon... of itself, actually
                    let g_center = self.triangles[i].centroid;

                    //addPoint(gCenter,false);
                    self.split_triangle(i, g_center).unwrap();
                }
                self.restore_delaunay(max_aspect_ratio);
            }
        }

        Ok(())
    }
} // end of impl Triangulation3D

pub fn is_convex(a: Point3D, b: Point3D, c: Point3D, d: Point3D) -> bool {
    // The points are aligned like this, if it is convex.
    //         B
    //        /\
    //     A /  \ C
    //       \  /
    //        \/
    //         D

    // 1st... ABC
    let ab = b - a;
    let bc = c - b;
    let n_abc = ab.cross(bc);

    if n_abc.is_zero() {
        // Points are collinear
        return false;
    }

    // 2nd... BCD
    let cd = d - c;
    let n_bcd = bc.cross(cd);
    if n_bcd.is_zero() {
        return false;
    }
    if !n_abc.is_same_direction(n_bcd) {
        return false;
    }

    // 3rd... CDA
    let da = a - d;
    let n_cda = cd.cross(da);
    if n_cda.is_zero() {
        return false;
    }
    if !n_abc.is_same_direction(n_cda) {
        return false;
    }

    // 4th... DAB
    let n_dab = da.cross(ab);
    if n_dab.is_zero() {
        return false;
    }
    if !n_abc.is_same_direction(n_dab) {
        return false;
    }

    // Passed all the checks
    true
}

/***********/
/* TESTING */
/***********/

#[cfg(test)]
mod testing {
    use super::*;
    use crate::loop3d::*;

    fn test_triangulation_results(
        t: &Triangulation3D,
        p: &Polygon3D,
        max_area: f64,
        max_aspect_ratio: f64,
    ) -> Result<(),String> {
        // Test area.
        let mut triangulation_area = 0.;        

        // Checks triangles one by one
        for (i,tripiece) in t.triangles.iter().enumerate() {            

            // get area            
            triangulation_area += tripiece.triangle.area();

            // Check proportions of triangle.
            if tripiece.triangle.area() >= max_area {
                return Err(format!("Triangle {} in the triangulation has area ({}) >= max_area ({}) ",i, tripiece.triangle.area(),max_area));                
            }

            if tripiece.aspect_ratio > max_aspect_ratio {
                return Err(format!("Triangle {} in the triangulation has aspect_ratio ({}) >= max_aspect_ratio ({}) ",i, tripiece.aspect_ratio,max_aspect_ratio));                
            }

            // Check if any of the three vertices is
            // an edge in the polygon... in such a case,
            // it should be constrained.
            for e in 0..3 {
                
                let edge = tripiece.triangle.segment(e).unwrap();

                // Check neighbour in this edge
                let neighbour_i = tripiece.neighbour(Edge::from_i(e));
                if let Some(neighbour_i) = neighbour_i{
                    let neigh = &t.triangles[neighbour_i];
                    let neigh_edge = neigh.triangle.get_edge_index_from_points(edge.start, edge.end).unwrap();
                    let neighbours_neighbour = neigh.neighbour(Edge::from_i(neigh_edge)).unwrap() as usize;
                    if neighbours_neighbour != i {
                        panic!(" ... This({}).neighbour({})={} != Neighbour({}).neighbour({})={}",i,e,neighbour_i,neighbour_i,neigh_edge,neighbours_neighbour);
                    }

                }

                

                // Go through outer loop.
                let outer = p.outer();
                let n_out = outer.n_vertices();
                for j in 0..n_out{
                    let a = outer[j];
                    let b = outer[(j+1)%n_out];
                    let outer_segment = Segment3D::new(a, b);

                    if outer_segment.contains(&edge).unwrap() {
                        if !tripiece.is_constrained(Edge::from_i(e)) {
                            return Err(format!("Edge {} of Triangle {} in triangulation should be constrained",e,i));
                        }
                    }
                }
                

                // Go through inner loops.
                for l_i in 0..p.n_inner_loops() {
                    let inner = p.inner(l_i).unwrap();
                    let n_in = inner.n_vertices();
                    
                    for j in 0..n_in{
                        let a = inner[j];
                        let b = inner[(j+1)%n_in];
                        let inner_segment = Segment3D::new(a, b);

                        if inner_segment.contains(&edge).unwrap() {
                            if !tripiece.is_constrained(Edge::from_i(e)) {
                                return Err(format!("Edge {} of Triangle3D {} in triangulation should be constrained",e,i));
                            }
                        }
                    }
                }// end of iterating inner loops
            }
        }

        // Must have the same area.
        if (triangulation_area - p.area()).abs() > 1E-9 {
            return Err(format!("Areas of orginal polygon (A = {}) and triangulation (A = {}) do not match", triangulation_area,p.area()));
        }
        // return
        Ok(())
    }

    #[test]
    fn test_test_triangulation_results() {
        let a = Point3D::new(0., 0., 0.);
        let b = Point3D::new(1., 0., 0.);
        let c = Point3D::new(1., 1., 0.);
        

        let mut outer = Loop3D::new();
        outer.push(a).unwrap();
        outer.push(b).unwrap();
        outer.push(c).unwrap();
        outer.close().unwrap();

        let poly = Polygon3D::new(outer).unwrap();

        let mut t = Triangulation3D::new();
        t.push(a, b, c, 0).unwrap();

        // No contraints... should fail
        assert!(test_triangulation_results(&t, &poly, 1000., 1000.).is_err());

        // Add contraints and test again.
        t.triangles[0].constrain(Edge::Ab);
        t.triangles[0].constrain(Edge::Bc);
        t.triangles[0].constrain(Edge::Ca);
        assert!(test_triangulation_results(&t, &poly, 1000., 1000.).is_ok());

        // Add another triangle... should not work.
        let d = Point3D::new(-1., 1., 0.);
        t.push(a, c, d, 0).unwrap();

        assert!(test_triangulation_results(&t, &poly, 1000., 1000.).is_err());
    }

    use std::fs::File;
    use std::io::prelude::*;
    // use crate::vector3d::Vector3D;

    fn draw_triangulation(filename:&str, cases:Vec<(&str,String)>){
        let mut file = File::create(format!("./test_data/{}", filename)).unwrap();
        file.write(b"<html><head></head><body>").unwrap();

        for (title,svg) in cases.iter(){
            file.write(b"<div>").unwrap();

            file.write(format!("<h1>{}</h1>", title).as_bytes()).unwrap();
            file.write(svg.as_bytes()).unwrap();
            file.write(b"</div>").unwrap();
        }

        file.write(b"</body></html>").unwrap();
    }

    fn get_triangulation_svg(t:&Triangulation3D, min_x:f64, max_x:f64, min_y:f64, max_y:f64)->String{
        
        // Arrange a transformation        
        const SCALE : f64 = 50.;
        const PADDING : f64 = 5.; // PErcentage of BBOX
        let width = max_x - min_x;
        let height = max_y - min_y;
        let padding_x = PADDING * width/100.;
        let padding_y = PADDING * height/100.;

        let into_svg_coordinates = |pt:Point3D|->Point3D{
            Point3D::new(pt.x - min_x+padding_x, padding_y + max_y-pt.y, 0.0)
        };
        
        /* DRAW TRIANGULATION */
        let mut ret = format!("<svg width='{}' height='{}'>\n\n", SCALE*(2.*padding_x + width), SCALE*(2.*padding_y+height));

        let n_tri = t.n_triangles();
        // Triangles
        for i in 0..n_tri{
            let tri = &t.triangles[i];
            if !tri.valid{
                continue;
            }
            ret+=&"<polygon points='";
            for vertex_i in 0..3{
                let v = tri.triangle.vertex(vertex_i).unwrap();
                let svg_point = into_svg_coordinates(v);
                ret+=&format!("{},{} ",SCALE*svg_point.x, SCALE*svg_point.y);            
            }
            ret+="' style='fill:#b9b9b9;stroke:#b9b9b9;stroke-width:0;fill-rule:evenodd;' />\n\n";
        }

        // Edges and neighbours
        for i in 0..n_tri{
            let tri = &t.triangles[i];
            if !tri.valid{
                continue;
            }                  

            for vertex_i in 0..3{                
                

                let this_edge = Edge::from_i(vertex_i);
                // Edges
                let v = tri.triangle.vertex(vertex_i).unwrap();
                let next_v = tri.triangle.vertex((vertex_i+1)%3).unwrap() ;
                let svg_point = into_svg_coordinates(v)*SCALE;
                let next_svg_point = into_svg_coordinates(next_v)*SCALE;

                let style = if tri.is_constrained(this_edge) {
                    "stroke:black;stroke-width:2"
                }else{
                    "stroke:black;stroke-width:1;stroke-dasharray:6,12"
                };
                let line_string = format!("<line x1='{}' y1='{}' x2='{}' y2='{}' style='{}' />\n\n",svg_point.x, svg_point.y,next_svg_point.x,next_svg_point.y, style);
                ret+=&line_string; 
                
                // Neighbour
                let neighbour_i = tri.neighbour(this_edge);
                if let Some(neighbour_i) = neighbour_i{
                    let neighbour = &t.triangles[neighbour_i];
                    if !neighbour.valid{
                        panic!("Triangle {} is neighbour of an invalid Triangle, {}", i, neighbour_i);
                    }
                    
                    let this_center = into_svg_coordinates(tri.centroid )*SCALE;
                    let other_center = into_svg_coordinates(neighbour.centroid )*SCALE;
                    let style = "stroke:red;stroke-width:2;fill:red";
                    let line_string = format!("<line x1='{}' y1='{}' x2='{}' y2='{}' style='{}' />\n\n",this_center.x, this_center.y,other_center.x,other_center.y, style);
                    ret+=&line_string; 

                    let circle_string = format!("<circle cx='{}' cy='{}' r='3.5' style='{}' />", this_center.x, this_center.y, style);
                    ret+=&circle_string; 
                }


            }            
        }
        ret+="</svg>";
        ret
    }

    fn get_svg(t: &Triangulation3D,p: &Polygon3D)->String{
        
        // Get polygon bounding box
        let mut min_x = f64::MAX;
        let mut max_x = f64::MIN;
        let mut min_y = f64::MAX;
        let mut max_y = f64::MIN;
        for point in p.outer().vertices().iter(){
            if point.x > max_x{                
                max_x = point.x;
            }
            if point.x < min_x{
                min_x = point.x;
            }
            if point.y > max_y{                
                max_y = point.y;
            }
            if point.y < min_y{
                min_y = point.y;
            }
        }                
        let height = max_y-min_y;
        let width = max_x-min_x;

        // Arrange a transformation
        const SCALE : f64 = 50.;
        const PADDING : f64 = 5.; // PErcentage of BBOX
        let padding_x = PADDING * width/100.;
        let padding_y = PADDING * height/100.;

        let into_svg_coordinates = |pt:Point3D|->Point3D{
            Point3D::new(pt.x - min_x+padding_x, padding_y + max_y-pt.y, 0.0)
        };


        let mut ret = format!("<svg width='{}' height='{}'>\n\n", SCALE*(2.*padding_x + width), SCALE*(2.*padding_y+height));

        /* DRAW POLYGON */
        ret += "<polygon points='";

        for v in p.outer().vertices().iter(){
            let svg_point = into_svg_coordinates(*v);            
            ret += &format!("{},{} ",SCALE*svg_point.x, SCALE*svg_point.y).to_string();
        }
        ret+=&"' style='fill:#b9b9b9;stroke:black;stroke-width:2;fill-rule:evenodd;' />\n\n";
        
        let n_inner = p.n_inner_loops();

        for i in 0..n_inner{
            let inner = p.inner(i).unwrap();
            ret+=&"<polygon points='";
            for v in inner.vertices().iter(){                  
                let svg_point = into_svg_coordinates(*v);            
                ret+=&format!("{},{} ",SCALE*svg_point.x, SCALE*svg_point.y);            
            }
            ret+=&"' style='fill:white;stroke:black;stroke-width:2;fill-rule:evenodd;' />\n\n";
        }        
        

        ret+="</svg>";

        ret += &get_triangulation_svg(t, min_x, max_x, min_y, max_y);

        ret
    }

    #[test]
    fn test_new() {
        let t = Triangulation3D::new();
        assert_eq!(t.n_valid_triangles, 0);
        assert_eq!(t.n_valid_triangles(), 0);

        assert_eq!(t.n_triangles(), 0);
        assert_eq!(t.triangles.len(), 0);
    }

    #[test]
    fn test_default() {
        let t = Triangulation3D::default();
        assert_eq!(t.n_valid_triangles, 0);
        assert_eq!(t.n_valid_triangles(), 0);

        assert_eq!(t.n_triangles(), 0);
        assert_eq!(t.triangles.len(), 0);
    }

    

    #[test]
    fn test_add_point_to_triangle() {
        
        let a = Point3D::new(-1., 0., 0.);
        let b = Point3D::new(1., 0., 0.);
        let c = Point3D::new(0., 1., 0.);
        
        
        // Point inside (calls split_triangle())
        let mut t = Triangulation3D::new();
        let i = t.push(a, b, c, 0).unwrap();
        t.add_point_to_triangle(i, Point3D::new(0.0, 0.1, 0.), PointInTriangle::Inside).unwrap();
        assert_eq!(3,t.triangles.len());

        // Point on AB edge (calls split_edge())
        let mut t = Triangulation3D::new();
        let i = t.push(a, b, c, 0).unwrap();
        t.add_point_to_triangle(i, Point3D::new(0.0, 0.0, 0.), PointInTriangle::EdgeAB).unwrap();
        assert_eq!(2,t.triangles.len());

        // Point on Vertex (does nothing)
        let mut t = Triangulation3D::new();
        let i = t.push(a, b, c, 0).unwrap();
        t.add_point_to_triangle(i, a, PointInTriangle::VertexA).unwrap();
        assert_eq!(1,t.triangles.len());

    }

    #[test]
    fn test_add_point() {
        let a = Point3D::new(-1., 0., 0.);
        let b = Point3D::new(1., 0., 0.);
        let c = Point3D::new(0., 1., 0.);
        
        
        // Point inside (calls split_triangle())
        let mut t = Triangulation3D::new();
        t.push(a, b, c, 0).unwrap();
        t.add_point(Point3D::new(0.0, 0.1, 0.)).unwrap();
        assert_eq!(3,t.triangles.len());

        // Point on AB edge (calls split_edge())
        let mut t = Triangulation3D::new();
        t.push(a, b, c, 0).unwrap();
        t.add_point(Point3D::new(0.0, 0.0, 0.)).unwrap();
        assert_eq!(2,t.triangles.len());

        // Point on Vertex (does nothing)
        let mut t = Triangulation3D::new();
        t.push(a, b, c, 0).unwrap();
        t.add_point(a).unwrap();
        assert_eq!(1,t.triangles.len());
    }


    #[test]
    fn test_is_convex() {
        let a = Point3D::new(-1., -1., 0.);
        let b = Point3D::new(1., -1., 0.);
        let c = Point3D::new(1., 1., 0.);
        let d = Point3D::new(-1., 1., 0.);

        assert!(is_convex(a, b, c, d));
        assert!(!is_convex(a, b, d, c));
        assert!(!is_convex(a, a, a, a));

        let b = Point3D::new(-0.1, 0.1, 0.);
        assert!(!is_convex(a, b, c, d));

        let b = Point3D::new(-0., 0., 0.);
        assert!(!is_convex(a, b, c, d));

        let b = Point3D::new(0., -0.1, 0.);
        assert!(is_convex(a, b, c, d));
    }

    #[test]
    fn test_push() {
        let a = Point3D::new(-1., -1., 0.);
        let b = Point3D::new(1., -1., 0.);
        let c = Point3D::new(1., 1., 0.);
        let d = Point3D::new(-1., 1., 0.);

        let e = Point3D::new(0., 0., 0.);

        let mut t = Triangulation3D::new();

        assert_eq!(t.push(a, b, c, 0).unwrap(), 0);
        assert_eq!(t.n_valid_triangles, 1);
        assert_eq!(t.n_valid_triangles(), 1);
        assert_eq!(t.n_triangles(), 1);
        assert_eq!(t.triangles.len(), 1);

        assert_eq!(t.push(b, c, d, 0).unwrap(), 1);
        assert_eq!(t.n_valid_triangles, 2);
        assert_eq!(t.n_valid_triangles(), 2);
        assert_eq!(t.n_triangles(), 2);
        assert_eq!(t.triangles.len(), 2);

        // invalidate the first one
        t.invalidate(0).unwrap();
        // Check that there is only one valid,
        // but two in total
        assert_eq!(t.n_valid_triangles(), 1);
        assert_eq!(t.n_triangles(), 2);

        // Add a new one.. it should be in the
        // first position (replaced the old one)
        assert_eq!(t.push(b, c, e, 0).unwrap(), 0);

        // there should be two valid now, and two
        // in total
        assert_eq!(t.n_valid_triangles(), 2);
        assert_eq!(t.n_triangles(), 2);

        // Push the first one again
        assert_eq!(t.push(a, b, c, 0).unwrap(), 2);
        assert_eq!(t.n_valid_triangles(), 3);
        assert_eq!(t.n_triangles(), 3);
    }

    #[test]
    fn test_invalidate() {
        let a = Point3D::new(-1., -1., 0.);
        let b = Point3D::new(1., -1., 0.);
        let c = Point3D::new(1., 1., 0.);
        let d = Point3D::new(-1., 1., 0.);

        let mut t = Triangulation3D::new();

        t.push(a, b, c, 0).unwrap();
        t.push(b, c, d, 0).unwrap();
        assert_eq!(t.n_valid_triangles(), 2);

        assert!(t.invalidate(0).is_ok());
        assert_eq!(t.n_valid_triangles(), 1);
        assert!(t.invalidate(2).is_err());
    }

    #[test]
    fn test_get_first_invalid() {
        let a = Point3D::new(-1., -1., 0.);
        let b = Point3D::new(1., -1., 0.);
        let c = Point3D::new(1., 1., 0.);
        let d = Point3D::new(-1., 1., 0.);

        let mut t = Triangulation3D::new();

        t.push(a, b, c, 0).unwrap();
        t.push(b, c, d, 0).unwrap();

        match t.get_first_invalid(0) {
            Some(_i) => assert!(false),
            None => assert!(true),
        };

        t.invalidate(1).unwrap();
        match t.get_first_invalid(0) {
            Some(i) => assert_eq!(1, i),
            None => assert!(false),
        };

        t.invalidate(0).unwrap();
        match t.get_first_invalid(0) {
            Some(i) => assert_eq!(0, i),
            None => assert!(false),
        };

        match t.get_first_invalid(1) {
            Some(i) => assert_eq!(1, i),
            None => assert!(false),
        };
    }

    #[test]
    fn test_borrow_triangle() {
        let a = Point3D::new(-1., -1., 0.);
        let b = Point3D::new(1., -1., 0.);
        let c = Point3D::new(1., 1., 0.);
        let d = Point3D::new(-1., 1., 0.);

        let mut t = Triangulation3D::new();

        let mut last_added: usize = 0;
        last_added = t.push(a, b, c, last_added).unwrap();
        t.push(b, c, d, last_added).unwrap();

        // Borrow the first one.
        &t.triangles[0];        
    }

    #[test]
    fn test_mark_as_neighbours() {
        // This should work.
        let a = Point3D::new(-1., -1., 0.);
        let b = Point3D::new(1., -1., 0.);
        let c = Point3D::new(1., 1., 0.);
        let d = Point3D::new(-1., 1., 0.);
        let e = Point3D::new(0., -5., 0.);

        let mut t = Triangulation3D::new();

        let mut last_added: usize = 0;
        last_added = t.push(a, b, c, last_added).unwrap();
        last_added = t.push(c, a, d, last_added).unwrap();
        t.push(a, b, e, last_added).unwrap();

        // This should work.
        assert!(t.mark_as_neighbours(0, Edge::Ca, 1).is_ok());
        assert_eq!(t.triangles[0].neighbour(Edge::Ca).unwrap(), 1);
        assert_eq!(t.triangles[1].neighbour(Edge::Ab).unwrap(), 0);

        // This should not work... the edge is incorrect
        //assert!(t.mark_as_neighbours(0, Edge::Ab, 1).is_err());

        // This should not work... they are not neighbours.
        //assert!(t.mark_as_neighbours(0, Edge::Bc, 2).is_err());
    }

    #[test]
    fn test_get_opposite_vertex() {
        // This should work.
        let a = Point3D::new(-1., -1., 0.);
        let b = Point3D::new(1., -1., 0.);
        let c = Point3D::new(1., 1., 0.);

        let mut t = Triangulation3D::new();

        t.push(a, b, c, 0).unwrap();

        let s = Segment3D::new(a, b);
        let p = t.get_opposite_vertex(&t.triangles[0].triangle, s).unwrap();
        assert!(p.compare(c));

        let s = Segment3D::new(b, c);
        let p = t.get_opposite_vertex(&t.triangles[0].triangle, s).unwrap();
        assert!(p.compare(a));

        let s = Segment3D::new(a, c);
        let p = t.get_opposite_vertex(&t.triangles[0].triangle, s).unwrap();
        assert!(p.compare(b));
    }

    #[test]
    fn test_flip_diagonal() {

        let l = 5.;
         
        // This should work.
        let a = Point3D::new(-l, 0., 0.);
        let b = Point3D::new(l, 0., 0.);
        let c = Point3D::new(0., l, 0.);
        let opp = Point3D::new(0., -l, 0.);

        let p1 = Point3D::new(-l, -l, 0.);
        let p2 = Point3D::new(l, -l, 0.);
        let p3 = Point3D::new(l, l, 0.);
        let p4 = Point3D::new(-l, l, 0.);

        let mut t = Triangulation3D::new();

        t.push(a, b, c, 0).unwrap(); // 0
        t.push(a, b, opp, 0).unwrap(); // 1

        t.push(p1, opp, a, 0).unwrap(); // 2
        t.push(opp, p2, b, 0).unwrap(); // 3
        t.push(b, p3, c, 0).unwrap(); // 4
        t.push(c, p4, a, 0).unwrap(); // 5        

        // Expected to fail... they are not
        // neighbours yet
        // t.flip_diagonal(0, Edge::Ab);// does it panic?

        t.mark_as_neighbours(0, Edge::Ab, 1).unwrap();
        t.mark_as_neighbours(0, Edge::Bc, 4).unwrap();
        t.mark_as_neighbours(0, Edge::Ca, 5).unwrap();

        //t.mark_as_neighbours(1, 0, 0).unwrap();//reciprocity
        t.mark_as_neighbours(1, Edge::Bc, 3).unwrap();
        t.mark_as_neighbours(1, Edge::Ca, 2).unwrap();

        let original = get_triangulation_svg(&t, -l, l, -l, l);

        //there should still be 2 triangles.
        assert_eq!(t.n_triangles(), 6);
        // all valid
        assert_eq!(t.n_valid_triangles(), 6);

        // Should work now.
        t.flip_diagonal(0, Edge::Ab);

        let first_flip = get_triangulation_svg(&t, -l, l, -l, l);

        //there should still be 2 triangles.
        assert_eq!(t.n_triangles(), 6);
        // all valid
        assert_eq!(t.n_valid_triangles(), 6);

        // Test triangles.
        let a_opp_c = Triangle3D::new(a, opp, c).unwrap();
        let c_opp_b = Triangle3D::new(c, opp, b).unwrap();
        assert!(t.triangles[0].triangle.compare(&a_opp_c));
        assert!(t.triangles[1].triangle.compare(&c_opp_b));

        // Test neighbours
        assert_eq!(t.triangles[0].neighbour(Edge::Ab).unwrap(), 2);
        assert_eq!(t.triangles[0].neighbour(Edge::Bc).unwrap(), 1);
        assert_eq!(t.triangles[0].neighbour(Edge::Ca).unwrap(), 5);

        assert_eq!(t.triangles[1].neighbour(Edge::Ab).unwrap(), 0);
        assert_eq!(t.triangles[1].neighbour(Edge::Bc).unwrap(), 3);
        assert_eq!(t.triangles[1].neighbour(Edge::Ca).unwrap(), 4);

        draw_triangulation("flip_diagonal.html", vec![
            ("original", original),
            ("after", first_flip)
        ]);
    }

    #[test]
    fn test_get_flipped_aspect_ratio() {
        // This should work.
        let a = Point3D::new(-1., -1., 0.);
        let b = Point3D::new(1., -1., 0.);
        let c = Point3D::new(1., 1., 0.);
        let d = Point3D::new(-1., 1., 0.);

        let mut t = Triangulation3D::new();

        let mut last_added: usize = 0;
        let tri0 = Triangle3D::new(a, b, c).unwrap();
        last_added = t.push(a, b, c, last_added).unwrap();
        t.push(c, a, d, last_added).unwrap();
        let tri1 = Triangle3D::new(c, a, d).unwrap();

        // mark as neighbours
        assert!(t.mark_as_neighbours(0, Edge::Ca, 1).is_ok());
        assert_eq!(t.triangles[0].neighbour(Edge::Ca).unwrap(), 1);
        assert_eq!(t.triangles[1].neighbour(Edge::Ab).unwrap(), 0);

        // These should return -1... no neighbours there.
        assert!(t.get_flipped_aspect_ratio(0, Edge::Ab).is_none());
        assert!(t.get_flipped_aspect_ratio(0, Edge::Bc).is_none());

        // constrain side 0 of triangle 0
        t.triangles[0].constrain(Edge::Ab);
        assert!(t.get_flipped_aspect_ratio(0, Edge::Ab).is_none());

        // This should work.
        let tri0_ar = tri0.aspect_ratio();
        let tri1_ar = tri1.aspect_ratio();
        assert_eq!(tri0_ar, tri1_ar);
        assert_eq!(tri0_ar, t.get_flipped_aspect_ratio(0, Edge::Ca).unwrap());
        assert_eq!(tri1_ar, t.get_flipped_aspect_ratio(1, Edge::Ab).unwrap());
    }

    #[test]
    fn test_split_triangle() {

        let l = 4.;
        // This should work.
        let a = Point3D::new(-l/2., -l/2., 0.);
        let b = Point3D::new(l/2., -l/2., 0.);
        let c = Point3D::new(l/2., l/2., 0.);
        let d = Point3D::new(-l/2., l/2., 0.);
        let e = Point3D::new(-l/2., -l, 0.);
        let f = Point3D::new(l, -l/2., 0.);

        let mut t = Triangulation3D::new();
        t.push(a, b, c, 0).unwrap(); // 0
        t.push(a, c, d, 0).unwrap(); // 1
        t.push(e, a, b, 0).unwrap(); // 2
        t.push(f, b, c, 0).unwrap(); // 3

        t.mark_as_neighbours(1, Edge::Ab, 0).unwrap();
        t.mark_as_neighbours(0, Edge::Ca, 1).unwrap();
        t.mark_as_neighbours(0, Edge::Ab, 2).unwrap();
        t.mark_as_neighbours(0, Edge::Bc, 3).unwrap();

        // constrain AB
        t.triangles[1].constrain(Edge::Ca);
        
        let original = get_triangulation_svg(&t, -l, l, -l,l);

        // SPLIT!
        let p = Point3D::new(l/6., -l/6., 0.);
        
        // Should work.
        assert!(t.split_triangle(0, p).is_ok());

        let after = get_triangulation_svg(&t, -l, l, -l,l);

        // There should be a total of 4 valid triangles now.
        // Also, 4 triangles in total (the invalid one should
        // have been replaced by a new one.)
        assert_eq!(t.n_valid_triangles(), 6);
        assert_eq!(t.n_triangles(), 6);

        // Check neighbours and constraints.         // INDEX:
        // let abc = Triangle3D::new(a, b, c, 0).unwrap(); // removed
        // let acd = Triangle3D::new(a, c, d, 0).unwrap(); // 1
        // let cpa = Triangle3D::new(c, p, a, 0).unwrap(); // 0 (replacing ABC)
        // let bpa = Triangle3D::new(b, p, a, 0).unwrap(); // 2
        // let cpb = Triangle3D::new(c, p, b, 0).unwrap(); // 3

        // assert!(cpa.compare(&t.triangles[0]));
        // assert!(acd.compare(&t.triangles[1]));
        // assert!(bpa.compare(&t.triangles[2]));
        // assert!(cpb.compare(&t.triangles[3]));

        // for i in 0..t.n_triangles() {
        //     let tri = &t.triangles[i];

        //     if tri.compare(&abc) {
        //         // This should no be aywhere.
        //         assert_eq!(i, 100);
        //     } else if tri.compare(&acd) {
        //         assert_eq!(tri.index, i);

        //         assert_eq!(tri.neighbour(0).unwrap(), 0);
        //         assert_eq!(tri.neighbour(1).unwrap(), -1);
        //         assert_eq!(tri.neighbour(2).unwrap(), -1);

        //         assert!(!tri.is_constrained(0).unwrap());
        //         assert!(!tri.is_constrained(1).unwrap());
        //         assert!(!tri.is_constrained(2).unwrap());
        //     } else if tri.compare(&cpa) {
        //         assert_eq!(tri.index, i);

        //         assert_eq!(tri.neighbour(0).unwrap(), 3);
        //         assert_eq!(tri.neighbour(1).unwrap(), 2);
        //         assert_eq!(tri.neighbour(2).unwrap(), 1);

        //         assert!(!tri.is_constrained(0).unwrap());
        //         assert!(!tri.is_constrained(1).unwrap());
        //         assert!(!tri.is_constrained(2).unwrap());
        //     } else if tri.compare(&bpa) {
        //         assert_eq!(tri.index, i);

        //         assert_eq!(tri.neighbour(0).unwrap(), 3);
        //         assert_eq!(tri.neighbour(1).unwrap(), 0);
        //         assert_eq!(tri.neighbour(2).unwrap(), -1);

        //         // AB is constrained!
        //         assert!(!tri.is_constrained(0).unwrap());
        //         assert!(!tri.is_constrained(1).unwrap());
        //         assert!(tri.is_constrained(2).unwrap());
        //     } else if tri.compare(&cpb) {
        //         assert_eq!(tri.index, i);

        //         assert_eq!(tri.neighbour(0).unwrap(), 0);
        //         assert_eq!(tri.neighbour(1).unwrap(), 2);
        //         assert_eq!(tri.neighbour(2).unwrap(), -1);

        //         assert!(!tri.is_constrained(0).unwrap());
        //         assert!(!tri.is_constrained(1).unwrap());
        //         assert!(!tri.is_constrained(2).unwrap());
        //     }
        // }

        t.restore_delaunay(1.6);
        let after_delaunay = get_triangulation_svg(&t, -l, l, -l,l);

        draw_triangulation("split_triangle.html", vec![
            ("original", original),
            ("after", after),
            ("after restoring", after_delaunay)
        ]);
        
    }

    #[test]
    fn test_split_edge() {
        let l = 4.;
        // ONLY ONE HEMISPHERE
        // =====================
        // This should work.
        let a = Point3D::new(-l, 0., 0.);
        let b = Point3D::new(l, 0., 0.);
        let c = Point3D::new(0., l, 0.);
        let d = Point3D::new(-l, l, 0.);

        let p = Point3D::new(0., 0., 0.);

        let mut t = Triangulation3D::new();
        t.push(a, b, c, 0).unwrap(); //0
        t.push(a, c, d, 0).unwrap(); //1

        assert!(t.mark_as_neighbours(0, Edge::Ca, 1).is_ok());

        // Constrain BC and AB
        t.triangles[0].constrain(Edge::Bc);
        t.triangles[0].constrain(Edge::Ab);

        assert_eq!(2, t.n_triangles());
        assert_eq!(2, t.n_valid_triangles());

        let one_hemisphere_before = get_triangulation_svg(&t, -l, l, -l, l);

        // SPLIT
        t.split_edge(0, Edge::Ab, p).unwrap();

        let one_hemisphere_after = get_triangulation_svg(&t, -l, l, -l, l);
        assert_eq!(3, t.n_triangles());
        assert_eq!(3, t.n_valid_triangles());

        // // Check triangles.                          // INDEX:
        // let abc = Triangle3D::new(a, b, c, 0).unwrap(); // Should not be there
        // let apc = Triangle3D::new(a, p, c, 0).unwrap(); // 0
        // let acd = Triangle3D::new(a, c, d, 1).unwrap(); // 1
        // let pbc = Triangle3D::new(p, b, c, 2).unwrap(); // 2

        // // Check indexes (assigned above)... makes it
        // // easier to check the neighbourhood.
        // assert!(apc.compare(&t.triangles[0]));
        // assert!(acd.compare(&t.triangles[1]));
        // assert!(pbc.compare(&t.triangles[2]));

        // for i in 0..t.n_triangles() {
        //     let tri = &t.triangles[i];

        //     if tri.compare(&abc) {
        //         // This should no be aywhere.
        //         assert_eq!(i, 100);
        //     } else if tri.compare(&apc) {
        //         assert_eq!(i, tri.index);

        //         assert_eq!(tri.neighbour(0).unwrap(), -1);
        //         assert_eq!(tri.neighbour(1).unwrap(), pbc.index as i32);
        //         assert_eq!(tri.neighbour(2).unwrap(), acd.index as i32);

        //         assert!(tri.is_constrained(0).unwrap());
        //         assert!(!tri.is_constrained(1).unwrap());
        //         assert!(!tri.is_constrained(2).unwrap());
        //     } else if tri.compare(&acd) {
        //         assert_eq!(i, tri.index);

        //         assert_eq!(tri.neighbour(0).unwrap(), apc.index as i32);
        //         assert_eq!(tri.neighbour(1).unwrap(), -1);
        //         assert_eq!(tri.neighbour(2).unwrap(), -1);

        //         assert!(!tri.is_constrained(0).unwrap());
        //         assert!(!tri.is_constrained(1).unwrap());
        //         assert!(!tri.is_constrained(2).unwrap());
        //     } else if tri.compare(&pbc) {
        //         assert_eq!(i, tri.index);

        //         assert_eq!(tri.neighbour(0).unwrap(), -1);
        //         assert_eq!(tri.neighbour(1).unwrap(), -1);
        //         assert_eq!(tri.neighbour(2).unwrap(), apc.index as i32);

        //         assert!(tri.is_constrained(0).unwrap());
        //         assert!(tri.is_constrained(1).unwrap());
        //         assert!(!tri.is_constrained(2).unwrap());
        //     }
        // }

        // NOW WITH TWO HEMISPHERES
        // ========================
        // This should work.
        let a = Point3D::new(-l, 0., 0.);
        let b = Point3D::new(l, 0., 0.);
        let c = Point3D::new(0., l, 0.);
        let d = Point3D::new(-l, l, 0.);
        let e = Point3D::new(-l, -l, 0.);
        let opp = Point3D::new(0., -l, 0.);

        let p = Point3D::new(0., 0., 0.);

        let mut t = Triangulation3D::new();
        t.push(a, b, c, 0).unwrap(); //0
        t.push(a, c, d, 0).unwrap(); //1
        t.push(a, b, opp, 0).unwrap(); //2
        t.push(a, opp, e, 0).unwrap(); //3

        assert!(t.mark_as_neighbours(0, Edge::Ca, 1).is_ok());
        assert!(t.mark_as_neighbours(0, Edge::Ab, 2).is_ok());
        assert!(t.mark_as_neighbours(2, Edge::Ca, 3).is_ok());

        // Constrain BC and OPP-B
        t.triangles[0].constrain(Edge::Bc);
        t.triangles[2].constrain(Edge::Bc);

        assert_eq!(4, t.n_triangles());
        assert_eq!(4, t.n_valid_triangles());

        let two_hemispheres_before = get_triangulation_svg(&t, -l, l, -l, l);

        // SPLIT
        t.split_edge(0, Edge::Ab, p).unwrap();


        assert_eq!(6, t.n_triangles());
        assert_eq!(6, t.n_valid_triangles());

        let two_hemispheres_after = get_triangulation_svg(&t, -l, l, -l, l);

        // // Check triangles.                          // INDEX:
        // let abc = Triangle3D::new(a, b, c, 0).unwrap(); // Should not be there... replaced by APC
        // let apc = Triangle3D::new(a, p, c, 0).unwrap(); // 0
        // let acd = Triangle3D::new(a, c, d, 1).unwrap(); // 1
        // let abo = Triangle3D::new(a, b, opp, 0).unwrap(); // Should not be there... replaced by BPO
        // let apo = Triangle3D::new(p, a, opp, 2).unwrap(); // 2
        // let aoe = Triangle3D::new(a, opp, e, 3).unwrap(); // 3
        // let pbc = Triangle3D::new(p, b, c, 4).unwrap(); // 4
        // let pbo = Triangle3D::new(b, p, opp, 5).unwrap(); // 5

        // // Check indexes (assigned above)... makes it
        // // easier to check the neighbourhood.
        // assert!(apc.compare(&t.triangles[0]));
        // assert!(acd.compare(&t.triangles[1]));
        // assert!(apo.compare(&t.triangles[2]));
        // assert!(aoe.compare(&t.triangles[3]));
        // assert!(pbc.compare(&t.triangles[4]));
        // assert!(pbo.compare(&t.triangles[5]));

        // for i in 0..t.n_triangles() {
        //     let tri = &t.triangles[i];

        //     if tri.compare(&abc) {
        //         // This should no be aywhere.
        //         assert_eq!(i, 100);
        //     } else if tri.compare(&abo) {
        //         // This should no be aywhere.
        //         assert_eq!(i, 333);
        //     } else if tri.compare(&apc) {
        //         assert_eq!(i, tri.index);

        //         assert_eq!(tri.neighbour(0).unwrap(), apo.index as i32);
        //         assert_eq!(tri.neighbour(1).unwrap(), pbc.index as i32);
        //         assert_eq!(tri.neighbour(2).unwrap(), acd.index as i32);

        //         assert!(!tri.is_constrained(0).unwrap());
        //         assert!(!tri.is_constrained(1).unwrap());
        //         assert!(!tri.is_constrained(2).unwrap());
        //     } else if tri.compare(&acd) {
        //         assert_eq!(i, tri.index);

        //         assert_eq!(tri.neighbour(0).unwrap(), apc.index as i32);
        //         assert_eq!(tri.neighbour(1).unwrap(), -1);
        //         assert_eq!(tri.neighbour(2).unwrap(), -1);

        //         assert!(!tri.is_constrained(0).unwrap());
        //         assert!(!tri.is_constrained(1).unwrap());
        //         assert!(!tri.is_constrained(2).unwrap());
        //     } else if tri.compare(&pbo) {
        //         // Constrain BC and OPP-B
        //         assert_eq!(i, tri.index);

        //         assert_eq!(tri.neighbour(0).unwrap(), pbc.index as i32);
        //         assert_eq!(tri.neighbour(1).unwrap(), -1);
        //         assert_eq!(tri.neighbour(2).unwrap(), apo.index as i32);

        //         assert!(!tri.is_constrained(0).unwrap());
        //         assert!(tri.is_constrained(1).unwrap());
        //         assert!(!tri.is_constrained(2).unwrap());
        //     } else if tri.compare(&aoe) {
        //         assert_eq!(i, tri.index);

        //         assert_eq!(tri.neighbour(0).unwrap(), apo.index as i32);
        //         assert_eq!(tri.neighbour(1).unwrap(), -1);
        //         assert_eq!(tri.neighbour(2).unwrap(), -1);

        //         assert!(!tri.is_constrained(0).unwrap());
        //         assert!(!tri.is_constrained(1).unwrap());
        //         assert!(!tri.is_constrained(2).unwrap());
        //     } else if tri.compare(&pbc) {
        //         // Constrain BC and OPP-B
        //         assert_eq!(i, tri.index);

        //         assert_eq!(tri.neighbour(0).unwrap(), pbo.index as i32);
        //         assert_eq!(tri.neighbour(1).unwrap(), -1);
        //         assert_eq!(tri.neighbour(2).unwrap(), apc.index as i32);

        //         assert!(!tri.is_constrained(0).unwrap());
        //         assert!(tri.is_constrained(1).unwrap());
        //         assert!(!tri.is_constrained(2).unwrap());
        //     } else if tri.compare(&apo) {
        //         assert_eq!(i, tri.index);

        //         assert_eq!(tri.neighbour(0).unwrap(), apc.index as i32);
        //         assert_eq!(tri.neighbour(1).unwrap(), pbo.index as i32);
        //         assert_eq!(tri.neighbour(2).unwrap(), aoe.index as i32);

        //         assert!(!tri.is_constrained(0).unwrap());
        //         assert!(!tri.is_constrained(1).unwrap());
        //         assert!(!tri.is_constrained(2).unwrap());
        //     }
        // }

        draw_triangulation("split_edge.html", vec![
            ("One hemisphere - original", one_hemisphere_before),
            ("One hemisphere - after", one_hemisphere_after),
            ("Two hemispheres - original", two_hemispheres_before),
            ("Two hemispheres - after", two_hemispheres_after),
        ]);
    }

    

    #[test]
    fn test_mesh_polygon() {
        /* SIMPLE CASE ... three vertices */
        let a = Point3D::new(0., 0., 0.);
        let b = Point3D::new(4., 0., 0.);
        let c = Point3D::new(4., 4., 0.);
        

        let mut outer = Loop3D::new();
        outer.push(a).unwrap();
        outer.push(b).unwrap();
        outer.push(c).unwrap();
        outer.close().unwrap();

        let poly = Polygon3D::new(outer).unwrap();
        let max_area = 0.3;
        let max_aspect_ratio = 1.6;
        let t = Triangulation3D::mesh_polygon(&poly, max_area, max_aspect_ratio).unwrap();
        let case0 = get_svg(&t, &poly);
        // test_triangulation_results(&t, &poly, max_area, max_aspect_ratio).unwrap();
        

        /* SOMEHOW MORE COMPLICATED CASE ... 6 vertices */
        let p0 = Point3D::new(0., 0., 0.);
        let p1 = Point3D::new(0., 3., 0.);
        let p2 = Point3D::new(3., 3., 0.);
        let p3 = Point3D::new(5., 5., 0.);
        let p4 = Point3D::new(3., 6., 0.);
        let p5 = Point3D::new(0., 5., 0.);
        let mut outer = Loop3D::new();
        outer.push(p0).unwrap();
        outer.push(p1).unwrap();
        outer.push(p2).unwrap();
        outer.push(p3).unwrap();
        outer.push(p4).unwrap();
        outer.push(p5).unwrap();
        outer.close().unwrap();

        let poly = Polygon3D::new(outer).unwrap();
        let max_area = 0.3;
        let max_aspect_ratio = 1.6;
        let t = Triangulation3D::mesh_polygon(&poly, max_area, max_aspect_ratio).unwrap();
        let case1 = get_svg(&t, &poly);
        // test_triangulation_results(&t, &poly, max_area, max_aspect_ratio).unwrap();

        draw_triangulation("test_mesh_polygon.html", vec![
            ("Case 0",case0),
            ("Case 1",case1),
        ])
    }

    #[test]
    fn test_from_polygon() {
        // /* SIMPLE CASE ... three vertices */
        let a = Point3D::new(0., 0., 0.);
        let b = Point3D::new(1., 0., 0.);
        let c = Point3D::new(1., 1., 0.);
        

        let mut outer = Loop3D::new();
        outer.push(a).unwrap();
        outer.push(b).unwrap();
        outer.push(c).unwrap();
        outer.close().unwrap();

        let poly = Polygon3D::new(outer).unwrap();
        let t = Triangulation3D::from_polygon(&poly).unwrap();
        let case0 = get_svg(&t, &poly);
        test_triangulation_results(&t, &poly, 1000., 1000.).unwrap();
        

        // /* SOMEHOW MORE COMPLICATED CASE ... 6 vertices */
        let p0 = Point3D::new(0., 0., 0.);
        let p1 = Point3D::new(3., 3., 0.); // Collinear... disapears
        let p2 = Point3D::new(5., 5., 0.);
        let p3 = Point3D::new(3., 6., 0.);
        let p4 = Point3D::new(0., 5., 0.);
        let p5 = Point3D::new(0., 3., 0.);// Collinear, disappear

        let mut outer = Loop3D::new();
        outer.push(p0).unwrap();
        outer.push(p1).unwrap();
        outer.push(p2).unwrap();
        outer.push(p3).unwrap();
        outer.push(p4).unwrap();
        outer.push(p5).unwrap();
        outer.close().unwrap();

        let poly = Polygon3D::new(outer).unwrap();
        let t = Triangulation3D::from_polygon(&poly).unwrap();
        
        let case1 = get_svg(&t, &poly);
        test_triangulation_results(&t, &poly, 1000., 1000.).unwrap();


        /* MORE COMPLICATED CASE  */
        /* 
         In this case, the triangulation needs to drop one of
         the "ears"; specifically, the one that goes through points
         [(0,5),(0,0),(0,3)]
        */
        
        let p0 = Point3D::new(0., 0., 0.);
        let p1 = Point3D::new(0., 3., 0.);
        let p2 = Point3D::new(3., 3., 0.);
        let p3 = Point3D::new(5., 5., 0.);
        let p4 = Point3D::new(3., 6., 0.);
        let p5 = Point3D::new(0., 5., 0.);
        let mut outer = Loop3D::new();
        outer.push(p0).unwrap();
        outer.push(p1).unwrap();
        outer.push(p2).unwrap();
        outer.push(p3).unwrap();
        outer.push(p4).unwrap();
        outer.push(p5).unwrap();
        outer.close().unwrap();

        let poly = Polygon3D::new(outer).unwrap();        
        let t = Triangulation3D::from_polygon(&poly).unwrap();
        let case2 = get_svg(&t, &poly);
        test_triangulation_results(&t, &poly, 9999999., 9999999.).unwrap();

        draw_triangulation("test_from_polygon.html", vec![
            ("Case 0",case0),
            ("Case 1",case1),
            ("Case 2",case2),
        ])

    }

    #[test]
    fn test_mark_neighbourhoods(){
        let p0 = Point3D::new(0., 0., 0.);
        let p1 = Point3D::new(0., 3., 0.);
        let p2 = Point3D::new(3., 3., 0.);
        let p3 = Point3D::new(5., 5., 0.);
        let p4 = Point3D::new(3., 6., 0.);
        let p5 = Point3D::new(0., 5., 0.);

        let mut t = Triangulation3D::new();
        t.push(p0, p1, p2, 0).unwrap();
        t.push(p2, p4, p3, 0).unwrap();
        t.push(p2, p1, p4, 0).unwrap();
        t.push(p1, p5, p4, 0).unwrap();

        t.mark_neighbourhouds().unwrap();

        assert_eq!(t.triangles[0].neighbour(Edge::Ab),None);
        assert_eq!(t.triangles[0].neighbour(Edge::Bc),Some(2));
        assert_eq!(t.triangles[0].neighbour(Edge::Ca),None); // Fix this

        assert_eq!(t.triangles[1].neighbour(Edge::Ab),Some(2));
        assert_eq!(t.triangles[1].neighbour(Edge::Bc),None);
        assert_eq!(t.triangles[1].neighbour(Edge::Ca),None);

        assert_eq!(t.triangles[2].neighbour(Edge::Ab),Some(0));
        assert_eq!(t.triangles[2].neighbour(Edge::Bc),Some(3));
        assert_eq!(t.triangles[2].neighbour(Edge::Ca),Some(1));

        assert_eq!(t.triangles[3].neighbour(Edge::Ab),None);
        assert_eq!(t.triangles[3].neighbour(Edge::Bc),None);
        assert_eq!(t.triangles[3].neighbour(Edge::Ca),Some(2));
    }

    #[test]
    fn test_restore_delaunay() {
        // assert!(false)
    }

    #[test]
    fn test_constraints() {
        let l = 3.;
        let a = Point3D::new(-l, 0., 0.);
        let b = Point3D::new(l, 0., 0.);
        let c = Point3D::new(0., l, 0.);

        let mut t = TriPiece::new(a, b, c, 0).unwrap();

        assert!(!t.is_constrained(Edge::Ab));
        assert!(!t.is_constrained(Edge::Bc));
        assert!(!t.is_constrained(Edge::Ca));        

        t.constrain(Edge::Ab);

        assert!(t.is_constrained(Edge::Ab));
        assert!(!t.is_constrained(Edge::Bc));
        assert!(!t.is_constrained(Edge::Ca));
    }

    #[test]
    fn test_set_neighbour() {
        // This should work.
        let l = 3.;
        let a = Point3D::new(-l, 0., 0.);
        let b = Point3D::new(l, 0., 0.);
        let c = Point3D::new(0., l, 0.);

        let mut t1 = TriPiece::new(a, b, c, 0).unwrap();

        t1.set_neighbour(Edge::Ab, 1);
        t1.set_neighbour(Edge::Bc, 11);
        t1.set_neighbour(Edge::Ca, 111);

        assert_eq!(t1.n0.unwrap(), 1);
        assert_eq!(t1.n1.unwrap(), 11);
        assert_eq!(t1.n2.unwrap(), 111);
    }

    #[test]
    fn test_add_edge(){
        let edge = Edge::Ab;
        assert_eq!(Edge::Bc, edge+1);
        assert_eq!(Edge::Ca, edge+2);
        assert_eq!(Edge::Ab, edge+3);
    }

} // end of test module
