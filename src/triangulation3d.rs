/*
    This section was heavily influenced by
    the following source:

    Ruppert, J. (1995). A Delaunay refinement algorithm for quality 2-dimensional mesh generation. J. Algorithms, 18(3), 548-585.

*/

use crate::point3d::*;
use crate::polygon3d::*;
use crate::segment3d::*;
use crate::triangle3d::*;

/// A set of [`Triangle3D`] that, together, cover completely
/// a [`Polygon3D`].
pub struct Triangulation3D {
    /// The triangles
    triangles: Vec<Triangle3D>,

    /// The number of valid triangles.
    ///
    /// Check whether triangles are valid or not
    /// before using them. Invalidating triangles help
    /// accelerate the algorithms in this module.
    n_valid_triangles: usize,
}

impl Default for Triangulation3D {
    fn default() -> Self {
        Self::new()
    }
}

impl std::ops::Index<usize> for Triangulation3D {
    type Output = Triangle3D;

    fn index(&self, index: usize) -> &Self::Output {
        if index >= self.triangles.len(){
            panic!("Trying to get a vertex out of bounds... {} available, index was {}", self.triangles.len(), index);
        }
        &self.triangles[index]
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

    /// Loops over all [`Triangle3D`], setting the neighbours
    fn mark_neighbourhouds(&mut self)->Result<(),String>{
        let n = self.triangles.len();
        for this_i in 0..self.triangles.len(){
            for other_i in this_i+1..n{
                

                for edge_i in 0..3{
                    let start = self.triangles[this_i].vertex(edge_i)?;
                    let end = self.triangles[this_i].vertex((edge_i+1)%3)?;
                    
                    if let Some(_) = self.triangles[other_i].get_edge_index_by_points(start, end) {
                        self.mark_as_neighbours(this_i, edge_i, other_i)?;
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
                        t.triangles[count].constrain(0)?;
                    }
                    let s12 = Segment3D::new(v1,v2);
                    if poly.contains_segment(&s12){
                        t.triangles[count].constrain(1)?;
                    }
                    let s20 = Segment3D::new(v2,v0);
                    if poly.contains_segment(&s20){
                        t.triangles[count].constrain(2)?;
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
                if !self.triangles[i].is_valid() {
                    return Some(i);
                }
            }
        }
        None
    }

    /// Marks two triangles as neighbours
    fn mark_as_neighbours(&mut self, i1: usize, edge_1: usize, i2: usize) -> Result<(), String> {
        let start: Point3D;
        let end: Point3D;
        let edge_2: usize;

        if i1 == i2 {
            let msg = format!("Trying to make a triangle its own neighbour (index {})", i1);
            return Err(msg);
        }

        // First borrow... limit lifetime
        {
            // Get segment
            let t1 = &self[i1];
            if !t1.is_valid(){
                return Err("Accessing invalid triangle within mark_as_neighbours()".to_string())
            }
            

            let seg1 = match t1.segment(edge_1) {
                Ok(s) => s,
                Err(e) => return Err(e),
            };

            start = seg1.start();
            end = seg1.end();
        }

        // Second borrow... limit lifetime
        {
            // Get the second triangle.
            let t2 = &self[i2];
            if !t2.is_valid(){
                return Err("Accessing invalid triangle within mark_as_neighbours()".to_string())
            }
            
            // Get edge segment from the other triangle.
            edge_2 = match t2.get_edge_index_by_points(start, end) {
                Some(i) => i,
                None => {
                    let msg = format!("The specified edge ([{},{},{}] -> [{},{},{}]) is not in both Triangle3Ds that you want to mark as neighbours", start.x, start.y, start.z, end.x, end.y, end.z);
                    return Err(msg);
                }
            }
        }

        // We have already tested all the indexes.
        self.triangles[i1].set_neighbour(edge_1, i2).unwrap();
        self.triangles[i2].set_neighbour(edge_2, i1).unwrap();

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

        let result = Triangle3D::new(vertex_a, vertex_b, vertex_c, n);
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
        edge: Segment3D,
    ) -> Result<Point3D, String> {
        let start = edge.start();
        let end = edge.end();

        match triangle.get_edge_index_by_points(start, end) {
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

    fn get_flipped_aspect_ratio(&self, index: usize, edge: usize) -> Result<f64, String> {
        println!("... Get flipped aspect raion");
        if edge > 2 {
            let msg = format!(
                "Impossible index when getting flipped aspect ratio... index was '{}'",
                edge
            );
            return Err(msg);
        }

        // do not modify constraints
        let triangle = &self.triangles[index];
        if !triangle.is_valid() {
            let msg = "Found an invalid triangle when getting flipped aspect ratio".to_string();
            return Err(msg);
        }

        if triangle.is_constrained(edge).unwrap() {
            return Ok(-1.);
        }

        // get neighbor
        let neighbour_i = triangle.neighbour(edge).unwrap();
        if neighbour_i < 0 {
            // No neighbour on this side
            return Ok(-1.);
        }
        let neighbour = &self.triangles[neighbour_i as usize];
        if !neighbour.is_valid() {
            let msg = "Found an invalid neighbour when getting flipped aspect ratio".to_string();
            return Err(msg);
        }

        if neighbour.index() == triangle.index() {
            let msg = "Triangle is its own neighbor!".to_string();
            return Err(msg);
        }

        // get vertices
        let vertex_a = triangle.vertex(edge % 3).unwrap();
        let vertex_b = triangle.vertex((edge + 1) % 3).unwrap();
        let vertex_c = triangle.vertex((edge + 2) % 3).unwrap();

        // Get the oposite side
        let s = Segment3D::new(vertex_a, vertex_b);
        let opposite = self.get_opposite_vertex(&neighbour, s).unwrap();

        if !is_convex(vertex_a, opposite, vertex_b, vertex_c) {
            return Ok(-1.);
        }

        // get the other situation
        let flipped1 = Triangle3D::new(vertex_a, opposite, vertex_c, 0).unwrap();
        let flipped2 = Triangle3D::new(opposite, vertex_b, vertex_c, 0).unwrap();

        let f1_aspect = flipped1.aspect_ratio();
        let f2_aspect = flipped2.aspect_ratio();

        // Return the maximum
        if f1_aspect > f2_aspect {
            Ok(f1_aspect)
        } else {
            Ok(f2_aspect)
        }
    }

    fn flip_diagonal(&mut self, index: usize, edge: usize) -> Result<(), String> {
        // The transformation is as follows:
        //         C                C
        //        /\               /|\
        //     A /__\ B   -->   A / | \ B
        //       \  /             \ | /
        //        \/               \|/
        //         opposite          opposite
        //
        // The neighbours in AC, BC and so on need to be restored at the end.

        println!("Flip diagonal");

        if edge > 2 {
            let msg = format!("Edge out of bounds ({}) when trying to flip diagonal", edge);
            return Err(msg);
        }

        // We need to determine these four points.
        let vertex_a: Point3D;
        let vertex_b: Point3D;
        let vertex_c: Point3D;
        let opposite: Point3D;

        // also, the neighbours need to be determined.
        let neighbour_index: i32;
        let n_ac: i32;
        let n_bc: i32;
        let n_bopp: i32;
        let n_aopp: i32;

        // Open scope for borrowing.
        {
            let t1 = &self[index];

            // Check if valid
            if !t1.is_valid() {
                let msg = "Trying to flip diagonal with an invalid triangle".to_string();
                return Err(msg);
            }

            // get vertices
            vertex_a = t1.vertex(edge % 3).unwrap();
            vertex_b = t1.vertex((edge + 1) % 3).unwrap();
            vertex_c = t1.vertex((edge + 2) % 3).unwrap();

            // get neighbour index
            neighbour_index = match t1.neighbour(edge) {
                Ok(v) => v,
                Err(e) => return Err(e),
            };

            // Get other neighbours
            let ac_i = match t1.get_edge_index_by_points(vertex_a, vertex_c) {
                Some(i) => i,
                None => {
                    return Err(
                        "Segment AC not found on triangle when flipping diagonals.".to_string()
                    )
                }
            };

            n_ac = match t1.neighbour(ac_i) {
                Ok(v) => v,
                Err(e) => return Err(e),
            };

            let cb_i = match t1.get_edge_index_by_points(vertex_c, vertex_b) {
                Some(i) => i,
                None => {
                    return Err(
                        "Segment CB not found on triangle when flipping diagonals.".to_string()
                    )
                }
            };
            n_bc = match t1.neighbour(cb_i) {
                Ok(v) => v,
                Err(e) => return Err(e),
            };
        }

        // Different scope
        {
            if neighbour_index < 0 {
                let msg = "When flipping diagonal... No neighbour triangle is in the chosen edge"
                    .to_string();
                return Err(msg);
            }

            // check if neighbour and neighbour_index are valid
            let neighbor = &self[neighbour_index as usize];

            if !neighbor.is_valid() {
                let msg = "Trying to flip diagonal with an invalid neighbour triangle".to_string();
                return Err(msg);
            }
        }
        {
            let neighbor = &self.triangles[neighbour_index as usize];

            // Get the oposite side
            let s = Segment3D::new(vertex_a, vertex_b); //self.triangles[index].segment(edge).unwrap();

            opposite = self
                .get_opposite_vertex(&self.triangles[neighbour_index as usize], s)
                .unwrap();

            // Get other neighbours
            let bopp_i = match neighbor.get_edge_index_by_points(vertex_b, opposite) {
                Some(i) => i,
                None => {
                    return Err(
                        "Segment B-Opposite not found on triangle when flipping diagonals."
                            .to_string(),
                    )
                }
            };

            n_bopp = match neighbor.neighbour(bopp_i) {
                Ok(v) => v,
                Err(e) => return Err(e),
            };

            let aopp_i = match neighbor.get_edge_index_by_points(vertex_a, opposite) {
                Some(i) => i,
                None => {
                    return Err(
                        "Segment A-Opposite not found on triangle when flipping diagonals."
                            .to_string(),
                    )
                }
            };
            n_aopp = match neighbor.neighbour(aopp_i) {
                Ok(v) => v,
                Err(e) => return Err(e),
            };
        }

        // invalidate both triangles.
        if let Err(msg) = self.invalidate(index) {
            return Err(msg);
        }

        if let Err(msg) = self.invalidate(neighbour_index as usize) {
            return Err(msg);
        }

        // Push both triangles, in the places where
        // the original ones used to be
        self.push(vertex_a, opposite, vertex_c, index).unwrap();
        self.push(vertex_c, opposite, vertex_b, neighbour_index as usize)
            .unwrap();

        // Reorganize neighbours
        if n_aopp >= 0 {
            self.mark_as_neighbours(index, 0, n_aopp as usize).unwrap();
        }

        self.mark_as_neighbours(index, 1, neighbour_index as usize)
            .unwrap();

        if n_ac >= 0 {
            self.mark_as_neighbours(index, 2, n_ac as usize).unwrap();
        }

        // done by reciprocity
        // self.mark_as_neighbours(neighbour_index as usize, 0, index);

        if n_bopp >= 0 {
            self.mark_as_neighbours(neighbour_index as usize, 1, n_bopp as usize)
                .unwrap();
        }

        if n_bc >= 0 {
            self.mark_as_neighbours(neighbour_index as usize, 2, n_bc as usize)
                .unwrap();
        }

        Ok(())
    } // end of flip_diagonal()

    /// Adds a point to an edge of a [`Triangle3D`], splitting 
    /// it and its neighbour
    fn split_edge(
        &mut self,
        triangle_index: usize,
        edge_to_split: PointInTriangle,
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

        

        if !self[triangle_index].is_valid() {
            let msg = "Trying to split edge of an invalid Triangle3D".to_string();
            return Err(msg);
        }

        let edge_to_split = match edge_to_split {
            PointInTriangle::EdgeAB => 0,
            PointInTriangle::EdgeBC => 1,
            PointInTriangle::EdgeAC => 2,
            _ => {
                unreachable!();
            }
        };

        println!("Splitting edge {}", edge_to_split);

        // get points
        let base_segment = self.triangles[triangle_index]
            .segment(edge_to_split)
            .unwrap();
        let vertex_a = base_segment.start();
        let vertex_b = base_segment.end();

        // Neighbour
        let nei_i = self.triangles[triangle_index]
            .neighbour(edge_to_split)
            .unwrap();

        let mut process_hemisphere = |index: usize| -> (usize, usize) {
            let edge = self.triangles[index]
                .get_edge_index_by_points(vertex_a, vertex_b)
                .unwrap();
            let s = self.triangles[index].segment(edge).unwrap();

            // Get points
            let vertex_c = self.get_opposite_vertex(&self.triangles[index], s).unwrap();

            // invalidate base triangle.
            self.invalidate(index).unwrap();

            // Get neighbours in AC and BC and Constrains
            let bc_n = self.triangles[index].neighbour((edge + 1) % 3).unwrap();
            let ac_n = self.triangles[index].neighbour((edge + 2) % 3).unwrap();

            let bc_c = self.triangles[index].is_constrained((edge + 1) % 3).unwrap();
            let ac_c = self.triangles[index].is_constrained((edge + 2) % 3).unwrap();
            let ab_c = self.triangles[index].is_constrained((edge + 3) % 3).unwrap();

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
                self.triangles[apc_i].constrain(0).unwrap();
            }

            // Segment 1: PC
            self.mark_as_neighbours(apc_i, 1, pbc_i).unwrap();

            // Segment 2: CA
            if ac_n >= 0 {
                self.mark_as_neighbours(apc_i, 2, ac_n as usize).unwrap();
            } 
            if ac_c {
                self.triangles[apc_i].constrain(2).unwrap();
            }

            // PBC Triangle
            // -------------------

            // Segment 0: PB -> Don't know if there is any neighbour yet... will solve this
            // when adding the neighbour.
            // We do know, however, that it might be contrained
            if ab_c {
                self.triangles[pbc_i].constrain(0).unwrap();
            }

            // Segment 1: BC
            if bc_n >= 0 {
                println!("Marking as neighbours Triangles {} and {} through edge 1", pbc_i, bc_n);
                self.mark_as_neighbours(pbc_i, 1, bc_n as usize).unwrap();
            } 
            if bc_c {
                self.triangles[pbc_i].constrain(1).unwrap();
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
        if nei_i >= 0. as i32 {
            let (bottom_left_i, bottom_right_i) = process_hemisphere(nei_i as usize);

            // Mark the upper and lower hemispheres as neighbours.
            self.mark_as_neighbours(top_left_i, 0, bottom_left_i)
                .unwrap();
            self.mark_as_neighbours(top_right_i, 0, bottom_right_i)
                .unwrap();
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

        println!("Print triangle");

        // main points
        let vertex_a: Point3D;
        let vertex_b: Point3D;
        let vertex_c: Point3D;

        // original neighbours... to be restored
        let ab_n: i32;
        let bc_n: i32;
        let ca_n: i32;

        // constraints, to be restored
        let ab_c: bool;
        let bc_c: bool;
        let ca_c: bool;

        {
            let t = &self[i];            

            if !t.is_valid() {
                let msg = format!("Trying to split an invalid triangle (index {})", i);
                return Err(msg);
            }

            // get vertices
            vertex_a = t.vertex(0).unwrap();
            vertex_b = t.vertex(1).unwrap();
            vertex_c = t.vertex(2).unwrap();

            // Get neighbours and constraints
            let edge = t.get_edge_index_by_points(vertex_a, vertex_b).unwrap();
            ab_n = t.neighbour(edge).unwrap();
            ab_c = t.is_constrained(edge).unwrap();

            let edge = t.get_edge_index_by_points(vertex_b, vertex_c).unwrap();
            bc_n = t.neighbour(edge).unwrap();
            bc_c = t.is_constrained(edge).unwrap();

            let edge = t.get_edge_index_by_points(vertex_c, vertex_a).unwrap();
            ca_n = t.neighbour(edge).unwrap();
            ca_c = t.is_constrained(edge).unwrap();

            // Invalidate this triangle.
            self.invalidate(i).unwrap();
        }

        // Add new triangles, noting their indices
        // Indices of the new triangles
        let apc_i = self.push(vertex_a, point, vertex_c, 0).unwrap();
        let abp_i = self.push(vertex_a, vertex_b, point, 0).unwrap();
        let cpb_i = self.push(vertex_c, point, vertex_b, 0).unwrap();

        /* FIRST TRIANGLE: CPA -- set contraints and neighbours */

        // Edge 0 -> CP
        self.mark_as_neighbours(apc_i, 0, abp_i).unwrap();
        
        // Edge 1 -> PA
        self.mark_as_neighbours(apc_i, 1, cpb_i).unwrap();

        // Edge 2 -> AC

        // constrain
        if ca_c {
            self.triangles[apc_i].constrain(2).unwrap();

            // If it was not constrained, then maybe it
            // had a neighbour.
        } else if ca_n >= 0 {
            self.mark_as_neighbours(apc_i, 2, ca_n as usize).unwrap();
        } // else, no constrain nor neighbour... ignore.

        /* SECOND TRIANGLE: ABP -- set contraints and neighbours */

        // Edge 0 -> AB

        // constrain
        if ab_c {
            self.triangles[abp_i].constrain(0).unwrap();

            // If it was not constrained, then maybe it
            // had a neighbour.
        } else if ab_n >= 0 {
            self.mark_as_neighbours(abp_i, 0, ab_n as usize).unwrap();
        } // else, no constrain nor neighbour... ignore.
        
        // Edge 1 -> BP
        self.mark_as_neighbours(abp_i, 1, cpb_i).unwrap();
        
        // Edge 2 -> PA
        // Done by reciprocity
        //self.mark_as_neighbours(bpa_i, 1, cpa_i).unwrap();


        

        /* THIRD TRIANGLE: CPB -- set contraints and neighbours */

        // Edge 0 -> CP
        // Done by reciprocity
        //self.mark_as_neighbours(cpb_i, 0, bpa_i).unwrap();

        // Edge 1 -> PB
        // Done by reciprocity
        //self.mark_as_neighbours(cpb_i, 1, cpa_i).unwrap();

        // Edge 2 -> BC

        // constrain
        if bc_c {
            self.triangles[cpb_i].constrain(2).unwrap();

            // If it was not constrained, then maybe it
            // had a neighbour.
        } else if bc_n >= 0 {
            self.mark_as_neighbours(cpb_i, 2, bc_n as usize).unwrap();
        } // else, no constrain nor neighbour... ignore.

        // return
        Ok(())
    }

    fn restore_delaunay(&mut self, max_aspect_ratio: f64) {
        // Flipping diagonals does not change the number of triangles.
        println!("Restore delaunay");
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
                if !self.triangles[i].is_valid() {
                    continue;
                }

                // get the current aspect ratio
                let current_ar = self.triangles[i].aspect_ratio();

                // If it is acceptable, skip.
                if current_ar < max_aspect_ratio {
                    continue;
                }

                // Otherwise, check if it is worth flipping the diagonal
                let mut best_edge: i32 = -1;
                let mut best_aspect_ratio = 9E9;

                // Search in three directions (i.e. three edges)
                for j in 0..3 {
                    // calculate possible aspect ratio
                    let ar = self.get_flipped_aspect_ratio(i, j).unwrap();

                    // Ignore invalid neighbours or constraints
                    if ar < 0. {
                        continue;
                    } else if current_ar > ar && best_aspect_ratio > ar {
                        best_edge = j as i32;
                        best_aspect_ratio = ar;
                    }
                }

                // Flip if best neighbour actually exists and it is better than
                // the current situation (i.e. >= 0)
                if best_edge >= 0 {
                    // if at least one of them is better than the current
                    self.flip_diagonal(i, best_edge as usize).unwrap();
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
        if !self.triangles[index].is_valid() {
            let msg = "Trying to add point into an obsolete triangle".to_string();
            return Err(msg);
        }

        if p_location.is_vertex() {
            // Point is a vertex... ignore, but pretend we did something
            Ok(())
        } else if p_location.is_edge() {
            self.split_edge(index, p_location, point)
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
        
        for (i,triangle) in self.triangles.iter().enumerate() {
            // skip triangle if it has been deleted
            if !triangle.is_valid() {
                continue;
            }

            let p_location = triangle.test_point(point);
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
            if !self.triangles[i].is_valid() {
                continue;
            }

            let area = self.triangles[i].area();

            // if area is already too small, just ignore
            if area < 9e-3 {
                continue;
            }

            // If it is an issue of aspect ratio
            if self.triangles[i].aspect_ratio() > max_aspect_ratio {
                // find the longest edge... assume it is the first one
                let mut s = self.triangles[i].segment(0).unwrap();
                let mut s_i = 0;
                // but check the other two.
                for j in 1..3 {
                    if s.length() < self.triangles[i].segment(j).unwrap().length() {
                        s_i = j;
                        s = self.triangles[i].segment(j).unwrap();
                    }
                }

                let edge_to_split = match s_i {
                    0 => PointInTriangle::EdgeAB,
                    1 => PointInTriangle::EdgeBC,
                    2 => PointInTriangle::EdgeAC,
                    _ => {
                        unreachable!();
                    }
                };
                // add midpoint
                let mid_s = s.midpoint();
                self.split_edge(i, edge_to_split, mid_s).unwrap();
                self.restore_delaunay(max_aspect_ratio);
            } else if area > max_area {
                // try to add the circumcenter
                let c_center = self.triangles[i].circumcenter();

                // Since the circumcenter may be in another triangle, we need
                // to search for it.
                let res = self.add_point(c_center);
                if res.is_ok() {
                    // What should I do if the circumcenter is out of the polygon?
                    // For now just add the centroid of it...
                    // This one we KNOW will be inside the polygon... of itself, actually
                    let g_center = self.triangles[i].centroid();

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
        for (i,triangle) in t.triangles.iter().enumerate() {            

            // get area            
            triangulation_area += triangle.area();

            // Check proportions of triangle.
            if triangle.area() >= max_area {
                return Err(format!("Triangle {} in the triangulation has area ({}) >= max_area ({}) ",i, triangle.area(),max_area));                
            }

            if triangle.aspect_ratio() > max_aspect_ratio {
                return Err(format!("Triangle {} in the triangulation has aspect_ratio ({}) >= max_aspect_ratio ({}) ",i, triangle.aspect_ratio(),max_aspect_ratio));                
            }

            // Check if any of the three vertices is
            // an edge in the polygon... in such a case,
            // it should be constrained.
            for e in 0..3 {
                
                let edge = triangle.segment(e).unwrap();

                // Check neighbour in this edge
                let neighbour_i = triangle.neighbour(e).unwrap();
                if neighbour_i >= 0 {
                    let neigh = &t[neighbour_i as usize];
                    let neigh_edge = neigh.get_edge_index_by_points(edge.start, edge.end).unwrap();
                    let neighbours_neighbour = neigh.neighbour(neigh_edge).unwrap() as usize;
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
                        if !triangle.is_constrained(e).unwrap() {
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
                            if !triangle.is_constrained(e).unwrap() {
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
        t.triangles[0].constrain(0).unwrap();
        t.triangles[0].constrain(1).unwrap();
        t.triangles[0].constrain(2).unwrap();
        assert!(test_triangulation_results(&t, &poly, 1000., 1000.).is_ok());

        // Add another triangle... should not work.
        let d = Point3D::new(-1., 1., 0.);
        t.push(a, c, d, 0).unwrap();

        assert!(test_triangulation_results(&t, &poly, 1000., 1000.).is_err());
    }

    use std::fs::File;
    use std::io::prelude::*;
    use crate::vector3d::Vector3D;

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
            let tri = &t[i];
            if !tri.is_valid(){
                continue;
            }
            ret+=&"<polygon points='";
            for vertex_i in 0..3{
                let v = tri.vertex(vertex_i).unwrap();
                let svg_point = into_svg_coordinates(v);
                ret+=&format!("{},{} ",SCALE*svg_point.x, SCALE*svg_point.y);            
            }
            ret+="' style='fill:#b9b9b9;stroke:#b9b9b9;stroke-width:0;fill-rule:evenodd;' />\n\n";
        }

        // Edges and neighbours
        for i in 0..n_tri{
            let tri = &t[i];
            if !tri.is_valid(){
                continue;
            }                  

            for vertex_i in 0..3{                
                
                // Edges
                let v = tri.vertex(vertex_i).unwrap();
                let next_v = tri.vertex((vertex_i+1)%3).unwrap() ;
                let svg_point = into_svg_coordinates(v)*SCALE;
                let next_svg_point = into_svg_coordinates(next_v)*SCALE;

                let style = if tri.is_constrained(vertex_i).unwrap() {
                    "stroke:black;stroke-width:2"
                }else{
                    "stroke:black;stroke-width:1;stroke-dasharray:6,12"
                };
                let line_string = format!("<line x1='{}' y1='{}' x2='{}' y2='{}' style='{}' />\n\n",svg_point.x, svg_point.y,next_svg_point.x,next_svg_point.y, style);
                ret+=&line_string; 
                
                // Neighbour
                let neighbour_i = tri.neighbour(vertex_i).unwrap();
                if neighbour_i >= 0{
                    let neighbour = &t[neighbour_i as usize];
                    if !neighbour.is_valid(){
                        panic!("Triangle {} is neighbour of an invalid Triangle, {}", i, neighbour_i);
                    }
                    
                    let this_center = into_svg_coordinates(tri.centroid() )*SCALE;
                    let other_center = into_svg_coordinates(neighbour.centroid() )*SCALE;
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
        &t[0];        
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
        assert!(t.mark_as_neighbours(0, 2, 1).is_ok());
        assert_eq!(t.triangles[0].neighbour(2).unwrap(), 1);
        assert_eq!(t.triangles[1].neighbour(0).unwrap(), 0);

        // This should not work... the edge is incorrect
        assert!(t.mark_as_neighbours(0, 0, 1).is_err());

        // This should not work... edge is out of bounds
        assert!(t.mark_as_neighbours(0, 4, 1).is_err());

        // This should not work... they are not neighbours.
        assert!(t.mark_as_neighbours(0, 1, 2).is_err());
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
        let p = t.get_opposite_vertex(&t.triangles[0], s).unwrap();
        assert!(p.compare(c));

        let s = Segment3D::new(b, c);
        let p = t.get_opposite_vertex(&t.triangles[0], s).unwrap();
        assert!(p.compare(a));

        let s = Segment3D::new(a, c);
        let p = t.get_opposite_vertex(&t.triangles[0], s).unwrap();
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
        assert!(t.flip_diagonal(0, 0).is_err());

        t.mark_as_neighbours(0, 0, 1).unwrap();
        t.mark_as_neighbours(0, 1, 4).unwrap();
        t.mark_as_neighbours(0, 2, 5).unwrap();

        //t.mark_as_neighbours(1, 0, 0).unwrap();//reciprocity
        t.mark_as_neighbours(1, 1, 3).unwrap();
        t.mark_as_neighbours(1, 2, 2).unwrap();

        let original = get_triangulation_svg(&t, -l, l, -l, l);

        //there should still be 2 triangles.
        assert_eq!(t.n_triangles(), 6);
        // all valid
        assert_eq!(t.n_valid_triangles(), 6);

        // Should work now.
        assert!(t.flip_diagonal(0, 0).is_ok());

        let first_flip = get_triangulation_svg(&t, -l, l, -l, l);

        //there should still be 2 triangles.
        assert_eq!(t.n_triangles(), 6);
        // all valid
        assert_eq!(t.n_valid_triangles(), 6);

        // Test triangles.
        let a_opp_c = Triangle3D::new(a, opp, c, 0).unwrap();
        let c_opp_b = Triangle3D::new(c, opp, b, 1).unwrap();
        assert!(t.triangles[0].compare(&a_opp_c));
        assert!(t.triangles[1].compare(&c_opp_b));

        // Test neighbours
        assert_eq!(t.triangles[0].neighbour(0).unwrap(), 2);
        assert_eq!(t.triangles[0].neighbour(1).unwrap(), 1);
        assert_eq!(t.triangles[0].neighbour(2).unwrap(), 5);

        assert_eq!(t.triangles[1].neighbour(0).unwrap(), 0);
        assert_eq!(t.triangles[1].neighbour(1).unwrap(), 3);
        assert_eq!(t.triangles[1].neighbour(2).unwrap(), 4);

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
        let tri0 = Triangle3D::new(a, b, c, 0).unwrap();
        last_added = t.push(a, b, c, last_added).unwrap();
        t.push(c, a, d, last_added).unwrap();
        let tri1 = Triangle3D::new(c, a, d, 1).unwrap();

        // mark as neighbours
        assert!(t.mark_as_neighbours(0, 2, 1).is_ok());
        assert_eq!(t.triangles[0].neighbour(2).unwrap(), 1);
        assert_eq!(t.triangles[1].neighbour(0).unwrap(), 0);

        // These should return -1... no neighbours there.
        assert_eq!(-1., t.get_flipped_aspect_ratio(0, 0).unwrap());
        assert_eq!(-1., t.get_flipped_aspect_ratio(0, 1).unwrap());

        // constrain side 0 of triangle 0
        t.triangles[0].constrain(0).unwrap();
        assert_eq!(-1., t.get_flipped_aspect_ratio(0, 0).unwrap());

        // This should work.
        let tri0_ar = tri0.aspect_ratio();
        let tri1_ar = tri1.aspect_ratio();
        assert_eq!(tri0_ar, tri1_ar);
        assert_eq!(tri0_ar, t.get_flipped_aspect_ratio(0, 2).unwrap());
        assert_eq!(tri1_ar, t.get_flipped_aspect_ratio(1, 0).unwrap());
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

        t.mark_as_neighbours(1, 0, 0).unwrap();
        t.mark_as_neighbours(0, 2, 1).unwrap();
        t.mark_as_neighbours(0, 0, 2).unwrap();
        t.mark_as_neighbours(0, 1, 3).unwrap();

        // constrain AB
        t.triangles[1].constrain(2).unwrap();
        
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
        //         assert_eq!(tri.index(), i);

        //         assert_eq!(tri.neighbour(0).unwrap(), 0);
        //         assert_eq!(tri.neighbour(1).unwrap(), -1);
        //         assert_eq!(tri.neighbour(2).unwrap(), -1);

        //         assert!(!tri.is_constrained(0).unwrap());
        //         assert!(!tri.is_constrained(1).unwrap());
        //         assert!(!tri.is_constrained(2).unwrap());
        //     } else if tri.compare(&cpa) {
        //         assert_eq!(tri.index(), i);

        //         assert_eq!(tri.neighbour(0).unwrap(), 3);
        //         assert_eq!(tri.neighbour(1).unwrap(), 2);
        //         assert_eq!(tri.neighbour(2).unwrap(), 1);

        //         assert!(!tri.is_constrained(0).unwrap());
        //         assert!(!tri.is_constrained(1).unwrap());
        //         assert!(!tri.is_constrained(2).unwrap());
        //     } else if tri.compare(&bpa) {
        //         assert_eq!(tri.index(), i);

        //         assert_eq!(tri.neighbour(0).unwrap(), 3);
        //         assert_eq!(tri.neighbour(1).unwrap(), 0);
        //         assert_eq!(tri.neighbour(2).unwrap(), -1);

        //         // AB is constrained!
        //         assert!(!tri.is_constrained(0).unwrap());
        //         assert!(!tri.is_constrained(1).unwrap());
        //         assert!(tri.is_constrained(2).unwrap());
        //     } else if tri.compare(&cpb) {
        //         assert_eq!(tri.index(), i);

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

        assert!(t.mark_as_neighbours(0, 2, 1).is_ok());

        // Constrain BC and AB
        t.triangles[0].constrain(1).unwrap();
        t.triangles[0].constrain(0).unwrap();

        assert_eq!(2, t.n_triangles());
        assert_eq!(2, t.n_valid_triangles());

        let one_hemisphere_before = get_triangulation_svg(&t, -l, l, -l, l);

        // SPLIT
        t.split_edge(0, PointInTriangle::EdgeAB, p).unwrap();

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
        //         assert_eq!(i, tri.index());

        //         assert_eq!(tri.neighbour(0).unwrap(), -1);
        //         assert_eq!(tri.neighbour(1).unwrap(), pbc.index() as i32);
        //         assert_eq!(tri.neighbour(2).unwrap(), acd.index() as i32);

        //         assert!(tri.is_constrained(0).unwrap());
        //         assert!(!tri.is_constrained(1).unwrap());
        //         assert!(!tri.is_constrained(2).unwrap());
        //     } else if tri.compare(&acd) {
        //         assert_eq!(i, tri.index());

        //         assert_eq!(tri.neighbour(0).unwrap(), apc.index() as i32);
        //         assert_eq!(tri.neighbour(1).unwrap(), -1);
        //         assert_eq!(tri.neighbour(2).unwrap(), -1);

        //         assert!(!tri.is_constrained(0).unwrap());
        //         assert!(!tri.is_constrained(1).unwrap());
        //         assert!(!tri.is_constrained(2).unwrap());
        //     } else if tri.compare(&pbc) {
        //         assert_eq!(i, tri.index());

        //         assert_eq!(tri.neighbour(0).unwrap(), -1);
        //         assert_eq!(tri.neighbour(1).unwrap(), -1);
        //         assert_eq!(tri.neighbour(2).unwrap(), apc.index() as i32);

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

        assert!(t.mark_as_neighbours(0, 2, 1).is_ok());
        assert!(t.mark_as_neighbours(0, 0, 2).is_ok());
        assert!(t.mark_as_neighbours(2, 2, 3).is_ok());

        // Constrain BC and OPP-B
        t.triangles[0].constrain(1).unwrap();
        t.triangles[2].constrain(1).unwrap();

        assert_eq!(4, t.n_triangles());
        assert_eq!(4, t.n_valid_triangles());

        let two_hemispheres_before = get_triangulation_svg(&t, -l, l, -l, l);

        // SPLIT
        t.split_edge(0, PointInTriangle::EdgeAB, p).unwrap();


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
        //         assert_eq!(i, tri.index());

        //         assert_eq!(tri.neighbour(0).unwrap(), apo.index() as i32);
        //         assert_eq!(tri.neighbour(1).unwrap(), pbc.index() as i32);
        //         assert_eq!(tri.neighbour(2).unwrap(), acd.index() as i32);

        //         assert!(!tri.is_constrained(0).unwrap());
        //         assert!(!tri.is_constrained(1).unwrap());
        //         assert!(!tri.is_constrained(2).unwrap());
        //     } else if tri.compare(&acd) {
        //         assert_eq!(i, tri.index());

        //         assert_eq!(tri.neighbour(0).unwrap(), apc.index() as i32);
        //         assert_eq!(tri.neighbour(1).unwrap(), -1);
        //         assert_eq!(tri.neighbour(2).unwrap(), -1);

        //         assert!(!tri.is_constrained(0).unwrap());
        //         assert!(!tri.is_constrained(1).unwrap());
        //         assert!(!tri.is_constrained(2).unwrap());
        //     } else if tri.compare(&pbo) {
        //         // Constrain BC and OPP-B
        //         assert_eq!(i, tri.index());

        //         assert_eq!(tri.neighbour(0).unwrap(), pbc.index() as i32);
        //         assert_eq!(tri.neighbour(1).unwrap(), -1);
        //         assert_eq!(tri.neighbour(2).unwrap(), apo.index() as i32);

        //         assert!(!tri.is_constrained(0).unwrap());
        //         assert!(tri.is_constrained(1).unwrap());
        //         assert!(!tri.is_constrained(2).unwrap());
        //     } else if tri.compare(&aoe) {
        //         assert_eq!(i, tri.index());

        //         assert_eq!(tri.neighbour(0).unwrap(), apo.index() as i32);
        //         assert_eq!(tri.neighbour(1).unwrap(), -1);
        //         assert_eq!(tri.neighbour(2).unwrap(), -1);

        //         assert!(!tri.is_constrained(0).unwrap());
        //         assert!(!tri.is_constrained(1).unwrap());
        //         assert!(!tri.is_constrained(2).unwrap());
        //     } else if tri.compare(&pbc) {
        //         // Constrain BC and OPP-B
        //         assert_eq!(i, tri.index());

        //         assert_eq!(tri.neighbour(0).unwrap(), pbo.index() as i32);
        //         assert_eq!(tri.neighbour(1).unwrap(), -1);
        //         assert_eq!(tri.neighbour(2).unwrap(), apc.index() as i32);

        //         assert!(!tri.is_constrained(0).unwrap());
        //         assert!(tri.is_constrained(1).unwrap());
        //         assert!(!tri.is_constrained(2).unwrap());
        //     } else if tri.compare(&apo) {
        //         assert_eq!(i, tri.index());

        //         assert_eq!(tri.neighbour(0).unwrap(), apc.index() as i32);
        //         assert_eq!(tri.neighbour(1).unwrap(), pbo.index() as i32);
        //         assert_eq!(tri.neighbour(2).unwrap(), aoe.index() as i32);

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

        assert_eq!(t[0].neighbour(0).unwrap(),-1);
        assert_eq!(t[0].neighbour(1).unwrap(),2);
        assert_eq!(t[0].neighbour(2).unwrap(),-1);

        assert_eq!(t[1].neighbour(0).unwrap(),2);
        assert_eq!(t[1].neighbour(1).unwrap(),-1);
        assert_eq!(t[1].neighbour(2).unwrap(),-1);

        assert_eq!(t[2].neighbour(0).unwrap(),0);
        assert_eq!(t[2].neighbour(1).unwrap(),3);
        assert_eq!(t[2].neighbour(2).unwrap(),1);

        assert_eq!(t[3].neighbour(0).unwrap(),-1);
        assert_eq!(t[3].neighbour(1).unwrap(),-1);
        assert_eq!(t[3].neighbour(2).unwrap(),2);
    }

    #[test]
    fn test_restore_delaunay() {
        assert!(false)
    }

} // end of test module
