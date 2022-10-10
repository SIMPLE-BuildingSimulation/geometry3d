/*
MIT License

Copyright (c) 2021 Germán Molina

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

use crate::Float;
/*
    This section was heavily influenced by
    the following source:

    Ruppert, J. (1995). A Delaunay refinement algorithm for quality 2-dimensional mesh generation. J. Algorithms, 18(3), 548-585.

*/

use crate::{Point3D, PointInTriangle, Polygon3D, Segment3D, Triangle3D};

#[derive(Clone, Copy, Eq, PartialEq, Debug)]
#[repr(u8)]
enum Edge {
    Ab,
    Bc,
    Ca,
}

impl Edge {
    fn from_i(i: usize) -> Self {
        match i {
            0 => Self::Ab,
            1 => Self::Bc,
            2 => Self::Ca,
            _ => {
                panic!("Index given for Edge is out of bound (given was {})", i)
            }
        }
    }

    fn as_i(&self) -> usize {
        match self {
            Self::Ab => 0,
            Self::Bc => 1,
            Self::Ca => 2,
        }
    }
}

impl std::ops::Add<usize> for Edge {
    type Output = Self;

    fn add(self, other: usize) -> Self {
        let i = (self.as_i() + other) % 3;
        Self::from_i(i)
    }
}

#[derive(Debug, Clone, Copy)]
struct TriPiece {
    triangle: Triangle3D,

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

    // circumradius: Float,
    aspect_ratio: Float,
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
#[derive(Debug, Clone)]
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

impl TriPiece {
    pub fn new(
        vertex_a: Point3D,
        vertex_b: Point3D,
        vertex_c: Point3D,
        i: usize,
    ) -> Result<Self, String> {
        // Build the triangle
        let triangle = Triangle3D::new(vertex_a, vertex_b, vertex_c)?;

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
            Edge::Ca => self.n2 = Some(i),
        }
    }

    pub fn neighbour(&self, edge: Edge) -> Option<usize> {
        match edge {
            Edge::Ab => self.n0,
            Edge::Bc => self.n1,
            Edge::Ca => self.n2,
        }
    }

    pub fn constrain(&mut self, edge: Edge) {
        match edge {
            Edge::Ab => self.c0 = true,
            Edge::Bc => self.c1 = true,
            Edge::Ca => self.c2 = true,
        }
    }

    pub fn is_constrained(&self, edge: Edge) -> bool {
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
    pub fn with_capacity(i: usize) -> Triangulation3D {
        Triangulation3D {
            n_valid_triangles: 0,
            triangles: Vec::with_capacity(i),
        }
    }

    /// Transforms the `Triangulation3D` into a `Vec<Triangle3D>`.
    pub fn get_trilist(&self) -> Vec<Triangle3D> {
        self.triangles.iter().map(|t| t.triangle).collect()
    }

    /// Triangulates a [`Polygon3D`] using the simple ear
    /// clipping algorithm and then progresively refines the mesh in order to get
    /// a relatively healthy set of Delaunay triangles
    pub fn mesh_polygon(
        poly: &Polygon3D,
        max_area: Float,
        max_aspect_ratio: Float,
    ) -> Result<Triangulation3D, String> {
        let mut t = Self::from_polygon(poly)?;
        t.refine(max_area, max_aspect_ratio)?;
        Ok(t)
    }

    /// Loops over all [`Triangle3D`], setting the neighbours. THis is
    /// quite slow, but does the job.
    fn mark_neighbourhouds(&mut self) -> Result<(), String> {
        let n = self.triangles.len();
        for this_i in 0..n {
            for other_i in this_i + 1..n {
                for edge_i in 0..3 {
                    let edge = self.triangles[this_i].triangle.segment(edge_i).unwrap();

                    if self.triangles[other_i]
                        .triangle
                        .get_edge_index_from_segment(&edge)
                        .is_some()
                    {
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
        the_loop.close().unwrap();
        // This is a theorem, apparently.
        let mut t = Triangulation3D::with_capacity(the_loop.len() - 2);

        let mut anchor = 0;
        let mut count = 0;
        loop {
            assert!(count < 1000, "Excessive number of iteration when triangulating polygon");
            count += 1;
            let n = the_loop.len();
            let count = t.n_triangles();
            if n == 2 {
                // return
                t.mark_neighbourhouds()?;
                return Ok(t);
            }

            let v0 = the_loop[anchor % n];
            let v1 = the_loop[(anchor + 1) % n];
            let v2 = the_loop[(anchor + 2) % n];

            let potential_diag = Segment3D::new(v0, v2);
            let is_line = v0.is_collinear(v1, v2).unwrap();
            let is_diagonal = the_loop.is_diagonal(potential_diag);
            if !is_line && is_diagonal {
                // Do not add (i.e., just drop) ears that are too small.
                if potential_diag.length() > 0.0001 {
                    // Add triangle
                    t.push(v0, v1, v2, count).unwrap();

                    // Set contrain... there must be a smarter way of doing this
                    let s01 = Segment3D::new(v0, v1);
                    if poly.contains_segment(&s01) {
                        t.triangles[count].constrain(Edge::from_i(0));
                    }
                    let s12 = Segment3D::new(v1, v2);
                    if poly.contains_segment(&s12) {
                        t.triangles[count].constrain(Edge::from_i(1));
                    }
                    let s20 = Segment3D::new(v2, v0);
                    if poly.contains_segment(&s20) {
                        t.triangles[count].constrain(Edge::from_i(2));
                    }
                }

                //remove v1
                the_loop.remove((anchor + 1) % n);
            } else {
                anchor += 1;
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
        if !self.triangles[i1].valid {
            return Err("Accessing invalid triangle within mark_as_neighbours()".to_string());
        }
        // Get segment... we should panic here, as this is not a user error
        let seg1 = &self.triangles[i1].triangle.segment(edge_1.as_i()).unwrap();

        // Check neighbour
        if !self.triangles[i2].valid {
            return Err("Accessing invalid triangle within mark_as_neighbours()".to_string());
        }

        // Get edge segment from the other triangle... panic if does not work.
        let edge_2 = match self.triangles[i2].triangle.get_edge_index_from_segment(seg1){
            Some(e)=>e,
            None => panic!("The triangles that you are trying to mark as neighbours don't share a segment.\n Triangle 1 : {}\nTriangle 2: {}", self.triangles[i1].triangle, self.triangles[i2].triangle)
        };
        let edge_2 = Edge::from_i(edge_2);

        // We have already tested all the indexes.
        self.triangles[i1].set_neighbour(edge_1, i2);
        self.triangles[i2].set_neighbour(edge_2, i1);

        Ok(())
    }

    /// Creates a [`Triangle3D`] and pushes it into the [`Triangulation3D`]
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
    fn get_flipped_aspect_ratio(&self, index: usize, edge: Edge) -> Option<Float> {
        // do not modify constraints
        let tripiece = &self.triangles[index];
        if !tripiece.valid {
            panic!("Found an invalid triangle when getting flipped aspect ratio")
        }

        if tripiece.is_constrained(edge) {
            return None;
        }

        // get neighbor
        let neighbour_i = match tripiece.neighbour(edge) {
            None => return None,
            Some(i) => i,
        };

        let neighbour = &self.triangles[neighbour_i];
        if !neighbour.valid {
            panic!("Found an invalid neighbour when getting flipped aspect ratio");
        }

        if neighbour.index == tripiece.index {
            panic!("Triangle is its own neighbor!");
        }

        // get vertices
        let vertex_a = tripiece.triangle.vertex(edge.as_i() % 3).unwrap();
        let vertex_b = tripiece.triangle.vertex((edge.as_i() + 1) % 3).unwrap();
        let vertex_c = tripiece.triangle.vertex((edge.as_i() + 2) % 3).unwrap();

        // Get the oposite side
        let s = Segment3D::new(vertex_a, vertex_b);
        let opposite = self.get_opposite_vertex(&neighbour.triangle, s).unwrap();

        if !is_convex(vertex_a, opposite, vertex_b, vertex_c) {
            return None;
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
        // get neighbour index... There needs to be one, or panic
        let neighbour_index = match self.triangles[index].neighbour(edge) {
            None => panic!("Trying to flip diagonal of Triangle with no neighbour"),
            Some(i) => i,
        };

        // check if neighbour is valid
        if !self.triangles[neighbour_index].valid {
            panic!("Trying to flip diagonal with an invalid neighbour triangle");
        }

        // get vertices
        let vertex_a = self.triangles[index]
            .triangle
            .vertex(edge.as_i() % 3)
            .unwrap();
        let vertex_b = self.triangles[index]
            .triangle
            .vertex((edge.as_i() + 1) % 3)
            .unwrap();
        let vertex_c = self.triangles[index]
            .triangle
            .vertex((edge.as_i() + 2) % 3)
            .unwrap();
        // ... including the oposite side
        let s = Segment3D::new(vertex_a, vertex_b);

        let opposite = self
            .get_opposite_vertex(&self.triangles[neighbour_index].triangle, s)
            .unwrap();

        /* CHECK SURROUNDINGS */
        // AC segment
        let ac_i = match &self.triangles[index]
            .triangle
            .get_edge_index_from_points(vertex_a, vertex_c)
        {
            Some(i) => *i,
            None => {
                panic!("Segment AC not found on triangle when flipping diagonals.")
            }
        };
        let ac_edge = Edge::from_i(ac_i);
        let neighbour_ac_index = self.triangles[index].neighbour(ac_edge);
        let constrain_ac = self.triangles[index].is_constrained(ac_edge);

        // BC segment
        let cb_i = match &self.triangles[index]
            .triangle
            .get_edge_index_from_points(vertex_c, vertex_b)
        {
            Some(i) => *i,
            None => {
                panic!("Segment CB not found on triangle when flipping diagonals.")
            }
        };
        let cb_edge = Edge::from_i(cb_i);
        let neighbour_bc_i = self.triangles[index].neighbour(cb_edge);
        let constrain_bc = self.triangles[index].is_constrained(cb_edge);

        // B-Opposite
        let bopp_i = match self.triangles[neighbour_index]
            .triangle
            .get_edge_index_from_points(vertex_b, opposite)
        {
            Some(i) => i,
            None => {
                panic!("Segment B-Opposite not found on triangle when flipping diagonals.")
            }
        };
        let bopp_edge = Edge::from_i(bopp_i);
        let neighbour_bopp_i = self.triangles[neighbour_index].neighbour(bopp_edge);
        let constrain_bopp = self.triangles[neighbour_index].is_constrained(bopp_edge);

        // A-Opposite
        let aopp_i = match self.triangles[neighbour_index]
            .triangle
            .get_edge_index_from_points(vertex_a, opposite)
        {
            Some(i) => i,
            None => {
                panic!("Segment A-Opposite not found on triangle when flipping diagonals.")
            }
        };
        let aopp_edge = Edge::from_i(aopp_i);
        let neighbour_aopp_i = self.triangles[neighbour_index].neighbour(aopp_edge);
        let constrain_aopp = self.triangles[neighbour_index].is_constrained(aopp_edge);

        /* THE ORIGINAL TRIANGLES, AND PUSH THE NEW ONES */
        self.invalidate(index).unwrap();
        self.invalidate(neighbour_index).unwrap();
        let aoc_i = self.push(vertex_a, opposite, vertex_c, index).unwrap();
        let cob_i = self
            .push(vertex_c, opposite, vertex_b, neighbour_index)
            .unwrap();

        /* REORGANIZE NEIGHBOURHOOD */
        // Segment A-Opposite
        if let Some(ni) = neighbour_aopp_i {
            self.mark_as_neighbours(index, Edge::Ab, ni).unwrap();
        }
        if constrain_aopp {
            self.triangles[aoc_i].constrain(Edge::Ab);
        }

        // Segment C-Opposite
        self.mark_as_neighbours(aoc_i, Edge::Bc, cob_i).unwrap(); // this one cannot be constrained

        // Segment AC
        if let Some(ni) = neighbour_ac_index {
            self.mark_as_neighbours(index, Edge::Ca, ni).unwrap();
        }
        if constrain_ac {
            self.triangles[aoc_i].constrain(Edge::Ca);
        }

        // Segment C-Opposite
        // ... done by reciprocity; and cannot be constrained
        // self.mark_as_neighbours(cob_i, Edge::Ab, aoc_i);

        // Segment B-Opposite
        if let Some(ni) = neighbour_bopp_i {
            self.mark_as_neighbours(cob_i, Edge::Bc, ni).unwrap();
        }
        if constrain_bopp {
            self.triangles[cob_i].constrain(Edge::Bc);
        }

        // Segment BC
        if let Some(ni) = neighbour_bc_i {
            self.mark_as_neighbours(cob_i, Edge::Ca, ni).unwrap();
        }
        if constrain_bc {
            self.triangles[cob_i].constrain(Edge::Ca);
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

        let segment_to_split = self.triangles[triangle_index]
            .triangle
            .segment(edge_to_split.as_i())
            .unwrap();

        // Neighbour... this is not necessarily there
        let nei_i = self.triangles[triangle_index].neighbour(edge_to_split);

        // get points
        let mut process_hemisphere = |index: usize| -> (usize, usize) {
            let ab_index = self.triangles[index]
                .triangle
                .get_edge_index_from_segment(&segment_to_split)
                .unwrap();
            let ab = self.triangles[index].triangle.segment(ab_index).unwrap();
            let vertex_a = ab.start();
            let vertex_b = ab.end();

            let edge = self.triangles[index]
                .triangle
                .get_edge_index_from_segment(&ab)
                .unwrap();
            let edge = Edge::from_i(edge);

            // let s = self.triangles[index].segment(edge).unwrap();

            // Get points
            let vertex_c = self
                .get_opposite_vertex(&self.triangles[index].triangle, ab)
                .unwrap();

            // invalidate base triangle.
            self.invalidate(index).unwrap();

            // Get neighbours in AC and BC and Constrains
            let bc_n = self.triangles[index].neighbour(edge + 1);
            let ac_n = self.triangles[index].neighbour(edge + 2);

            let ab_c = self.triangles[index].is_constrained(edge);
            let bc_c = self.triangles[index].is_constrained(edge + 1);
            let ac_c = self.triangles[index].is_constrained(edge + 2);

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
            if let Some(ni) = ac_n {
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
            if let Some(ni) = bc_n {
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
        if let Some(nei_i) = nei_i {
            let (bottom_right_i, bottom_left_i) = process_hemisphere(nei_i);

            // Mark the upper and lower hemispheres as neighbours.
            self.mark_as_neighbours(top_left_i, Edge::Ab, bottom_left_i)
                .unwrap();
            self.mark_as_neighbours(top_right_i, Edge::Ab, bottom_right_i)
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

        if !self.triangles[i].valid {
            let msg = format!("Trying to split an invalid triangle (index {})", i);
            return Err(msg);
        }

        // get vertices
        let vertex_a = self.triangles[i].triangle.a();
        let vertex_b = self.triangles[i].triangle.b();
        let vertex_c = self.triangles[i].triangle.c();

        // Get neighbours and constraints
        let edge = self.triangles[i]
            .triangle
            .get_edge_index_from_points(vertex_a, vertex_b)
            .unwrap();
        let edge = Edge::from_i(edge);
        let neighbour_ab_i = self.triangles[i].neighbour(edge);
        let constrain_ab = self.triangles[i].is_constrained(edge);

        let edge = self.triangles[i]
            .triangle
            .get_edge_index_from_points(vertex_b, vertex_c)
            .unwrap();
        let edge = Edge::from_i(edge);
        let neighbour_bc_i = self.triangles[i].neighbour(edge);
        let constrain_bc = self.triangles[i].is_constrained(edge);

        let edge = self.triangles[i]
            .triangle
            .get_edge_index_from_points(vertex_c, vertex_a)
            .unwrap();
        let edge = Edge::from_i(edge);
        let neighbour_ca_i = self.triangles[i].neighbour(edge);
        let constrain_ca = self.triangles[i].is_constrained(edge);

        // Invalidate this triangle.
        self.invalidate(i).unwrap();

        // Add new triangles, noting their indices
        // Indices of the new triangles
        let cap_i = self.push(vertex_c, vertex_a, point, 0).unwrap();
        let abp_i = self.push(vertex_a, vertex_b, point, 0).unwrap();
        let bcp_i = self.push(vertex_b, vertex_c, point, 0).unwrap();

        // Connect them to each other
        self.mark_as_neighbours(cap_i, Edge::Bc, abp_i).unwrap();
        self.mark_as_neighbours(abp_i, Edge::Bc, bcp_i).unwrap();
        self.mark_as_neighbours(bcp_i, Edge::Bc, cap_i).unwrap();

        // Connect to outside
        if constrain_ca {
            self.triangles[cap_i].constrain(Edge::Ab);
        }
        if let Some(ni) = neighbour_ca_i {
            self.mark_as_neighbours(cap_i, Edge::Ab, ni).unwrap();
        }

        if constrain_ab {
            self.triangles[abp_i].constrain(Edge::Ab);
        }
        if let Some(ni) = neighbour_ab_i {
            self.mark_as_neighbours(abp_i, Edge::Ab, ni).unwrap();
        }

        if constrain_bc {
            self.triangles[bcp_i].constrain(Edge::Ab);
        }
        if let Some(ni) = neighbour_bc_i {
            self.mark_as_neighbours(bcp_i, Edge::Ab, ni).unwrap();
        }

        // return
        Ok(())
    }

    /// Runs through the [`Triangulation3D`] checking if it is worth
    /// flipping diagonals. I.e., if flipping the diagonal would make
    /// the aspect ratios become better
    fn restore_delaunay(&mut self, max_aspect_ratio: Float) {
        // Flipping diagonals does not change the number of triangles.
        let n = self.triangles.len();

        let mut any_changes = true;
        const MAX_LOOPS: usize = 30;
        let mut n_loops: usize = 0;

        while any_changes && n_loops < MAX_LOOPS {
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
                let mut best_aspect_ratio = Float::MAX;

                // Search in three directions (i.e. three edges)
                for j in 0..3 {
                    let this_edge = Edge::from_i(j);
                    // calculate possible aspect ratio...
                    if let Some(ar) = self.get_flipped_aspect_ratio(i, this_edge) {
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

    /// Adds a point on a specific [`Triangle3D`]. Returns a `bool`, indicating
    /// whether any change was carried out
    fn add_point_to_triangle(
        &mut self,
        index: usize,
        point: Point3D,
        p_location: PointInTriangle,
    ) -> bool {
        if !self.triangles[index].valid {
            panic!("{}", "Trying to add point into an obsolete triangle");
        }

        if p_location.is_vertex() {
            // Point is a vertex... ignore, but pretend we did something
            false
        } else if p_location.is_edge() {
            let edge = match p_location {
                PointInTriangle::EdgeAB => Edge::Ab,
                PointInTriangle::EdgeBC => Edge::Bc,
                PointInTriangle::EdgeAC => Edge::Ca,
                _ => {
                    panic!("Trying to split edge with a point that does not fall on an edge")
                }
            };
            self.split_edge(index, edge, point).unwrap();
            true
        } else if p_location == PointInTriangle::Inside {
            self.split_triangle(index, point).unwrap();
            true
        } else {
            unreachable!();
        }
    }

    /// Finds the location of a [`Point3D`] within the triangulation
    /// and inserts it. Returns a `bool` indicating if the addition of the new [`Point3D`]
    /// produced any change
    fn add_point(&mut self, point: Point3D) -> Result<bool, String> {
        // Iterate through triangles to check

        for (i, tripiece) in self.triangles.iter().enumerate() {
            // skip triangle if it has been deleted
            if !tripiece.valid {
                continue;
            }

            let p_location = tripiece.triangle.test_point(point);
            if p_location != PointInTriangle::Outside {
                return Ok(self.add_point_to_triangle(i, point, p_location));
            }
        }

        Err(
            "No Triangle3D in Triangulation3D contained the point that you wanted to add."
                .to_string(),
        )
    }

    /// Refines a [`Triangulation3D`]. This only works if all then [`Triangle3D`] have their
    /// vertices in the same order, meaning that their `normal` points in the same direction.
    fn refine(&mut self, max_area: Float, max_aspect_ratio: Float) -> Result<(), String> {
        // let n_triangles = self.n_triangles();

        let mut any_changes = false;
        for i in 0..self.n_triangles() {
            if !self.triangles[i].valid {
                // I think validity of triangles is obsolete and we
                // should remove it
                unreachable!();
            }

            let area = self.triangles[i].triangle.area();

            // if area is already too small, just ignore
            if area < 1e-3 {
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
                any_changes = true;
            } else if area > max_area {
                // try to add the circumcenter
                let c_center = self.triangles[i].circumcenter;

                // Since the circumcenter may be in another triangle, we need
                // to search for it.
                match self.add_point(c_center) {
                    Ok(did_something) => {
                        if did_something {
                            any_changes = true;
                            self.restore_delaunay(max_aspect_ratio);
                        }
                    }
                    Err(_) => {
                        // What should I do if the circumcenter is out of the polygon?
                        // For now just add the centroid of it...
                        let centroid = self.triangles[i].centroid;

                        // The Centroid of a triangle is always inside of it... right?
                        if self.add_point_to_triangle(i, centroid, PointInTriangle::Inside) {
                            any_changes = true;
                            self.restore_delaunay(max_aspect_ratio);
                        }
                    }
                }
            }
        }
        if any_changes {
            self.refine(max_area, max_aspect_ratio)
        } else {
            Ok(())
        }
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
    use crate::Loop3D;

    fn test_triangulation_results(
        t: &Triangulation3D,
        p: &Polygon3D,
        max_area: Float,
        max_aspect_ratio: Float,
    ) -> Result<(), String> {
        // Test area.
        let mut triangulation_area = 0.;
        let polygon_normal = p.normal();
        // Checks triangles one by one
        for (i, tripiece) in t.triangles.iter().enumerate() {
            // get area
            triangulation_area += tripiece.triangle.area();
            let triangle_normal = tripiece.triangle.normal();
            if !polygon_normal.is_same_direction(triangle_normal) {
                return Err(format!("Triangle and Polygon do not have the same Normal (they are {} and {}, respectively)", triangle_normal, polygon_normal));
            }

            // Check proportions of triangle.
            if tripiece.triangle.area() >= max_area {
                return Err(format!(
                    "Triangle {} in the triangulation has area ({}) >= max_area ({}) ",
                    i,
                    tripiece.triangle.area(),
                    max_area
                ));
            }

            if tripiece.aspect_ratio > max_aspect_ratio {
                return Err(format!("Triangle {} in the triangulation has aspect_ratio ({}) >= max_aspect_ratio ({}) ",i, tripiece.aspect_ratio,max_aspect_ratio));
            }

            // Check if any of the three edges is
            // an edge in the polygon... in such a case,
            // it should be constrained.
            for e in 0..3 {
                let edge = tripiece.triangle.segment(e).unwrap();

                // Check neighbour in this edge
                let neighbour_i = tripiece.neighbour(Edge::from_i(e));
                if let Some(neighbour_i) = neighbour_i {
                    let neigh = &t.triangles[neighbour_i];
                    let neigh_edge = neigh
                        .triangle
                        .get_edge_index_from_points(edge.start, edge.end)
                        .unwrap();
                    let neighbours_neighbour =
                        neigh.neighbour(Edge::from_i(neigh_edge)).unwrap() as usize;
                    if neighbours_neighbour != i {
                        panic!(
                            " ... This({}).neighbour({})={} != Neighbour({}).neighbour({})={}",
                            i, e, neighbour_i, neighbour_i, neigh_edge, neighbours_neighbour
                        );
                    }
                }

                // Go through outer loop.
                let outer = p.outer();
                let n_out = outer.len();
                for j in 0..n_out {
                    let a = outer[j];
                    let b = outer[(j + 1) % n_out];
                    let _outer_segment = Segment3D::new(a, b);

                    // if outer_segment.contains(&edge).unwrap() {
                    //     if !tripiece.is_constrained(Edge::from_i(e)) {
                    //         return Err(format!(
                    //             "Edge {} of Triangle {} in triangulation should be constrained",
                    //             e, i
                    //         ));
                    //     }
                    // }
                }

                // Go through inner loops.
                for l_i in 0..p.n_inner_loops() {
                    let inner = p.inner(l_i).unwrap();
                    let n_in = inner.len();

                    for j in 0..n_in {
                        let a = inner[j];
                        let b = inner[(j + 1) % n_in];
                        let inner_segment = Segment3D::new(a, b);

                        if inner_segment.contains(&edge).unwrap() {
                            if !tripiece.is_constrained(Edge::from_i(e)) {
                                return Err(format!("Edge {} of Triangle3D {} in triangulation should be constrained",e,i));
                            }
                        }
                    }
                } // end of iterating inner loops
            }
        }

        // Must have the same area.
        if (triangulation_area - p.area()).abs() > 1e-4 {
            return Err(format!(
                "Areas of orginal polygon (A = {}) and triangulation (A = {}) do not match",
                triangulation_area,
                p.area()
            ));
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
        // assert!(test_triangulation_results(&t, &poly, 1000., 1000.).is_err());

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

    fn draw_triangulation(filename: &str, cases: Vec<(&str, String)>) {
        let mut file = File::create(format!("./test_data/{}", filename)).unwrap();
        file.write(b"<html><head></head><body>").unwrap();

        for (title, svg) in cases.iter() {
            file.write(b"<div>").unwrap();

            file.write(format!("<h1>{}</h1>", title).as_bytes())
                .unwrap();
            file.write(svg.as_bytes()).unwrap();
            file.write(b"</div>").unwrap();
        }

        file.write(b"</body></html>").unwrap();
    }

    fn get_triangulation_svg(
        t: &Triangulation3D,
        min_x: Float,
        max_x: Float,
        min_y: Float,
        max_y: Float,
    ) -> String {
        // Arrange a transformation
        const SCALE: Float = 50.;
        const PADDING: Float = 5.; // PErcentage of BBOX
        let width = max_x - min_x;
        let height = max_y - min_y;
        let padding_x = PADDING * width / 100.;
        let padding_y = PADDING * height / 100.;

        let into_svg_coordinates = |pt: Point3D| -> Point3D {
            Point3D::new(pt.x - min_x + padding_x, padding_y + max_y - pt.y, 0.0)
        };

        /* DRAW TRIANGULATION */
        let mut ret = format!(
            "<svg width='{}' height='{}'>\n\n",
            SCALE * (2. * padding_x + width),
            SCALE * (2. * padding_y + height)
        );

        let n_tri = t.n_triangles();
        // Triangles
        for i in 0..n_tri {
            let tri = &t.triangles[i];
            if !tri.valid {
                continue;
            }
            ret += &"<polygon points='";
            for vertex_i in 0..3 {
                let v = tri.triangle.vertex(vertex_i).unwrap();
                let svg_point = into_svg_coordinates(v);
                ret += &format!("{},{} ", SCALE * svg_point.x, SCALE * svg_point.y);
            }
            ret += "' style='fill:#b9b9b9;stroke:#b9b9b9;stroke-width:0;fill-rule:evenodd;' />\n\n";
        }

        // Edges and neighbours
        for i in 0..n_tri {
            let tri = &t.triangles[i];
            if !tri.valid {
                continue;
            }

            for vertex_i in 0..3 {
                let this_edge = Edge::from_i(vertex_i);
                // Edges
                let v = tri.triangle.vertex(vertex_i).unwrap();
                let next_v = tri.triangle.vertex((vertex_i + 1) % 3).unwrap();
                let svg_point = into_svg_coordinates(v) * SCALE;
                let next_svg_point = into_svg_coordinates(next_v) * SCALE;

                let style = if tri.is_constrained(this_edge) {
                    "stroke:black;stroke-width:3"
                } else {
                    "stroke:#282828;stroke-width:.7" //;stroke-dasharray:6,12
                };
                let line_string = format!(
                    "<line x1='{}' y1='{}' x2='{}' y2='{}' style='{}' />\n\n",
                    svg_point.x, svg_point.y, next_svg_point.x, next_svg_point.y, style
                );
                ret += &line_string;

                // Neighbour
                let neighbour_i = tri.neighbour(this_edge);
                if let Some(neighbour_i) = neighbour_i {
                    let neighbour = &t.triangles[neighbour_i];
                    if !neighbour.valid {
                        panic!(
                            "Triangle {} is neighbour of an invalid Triangle, {}",
                            i, neighbour_i
                        );
                    }

                    let this_center = into_svg_coordinates(tri.centroid) * SCALE;
                    let other_center = into_svg_coordinates(neighbour.centroid) * SCALE;
                    let style = "stroke:red;stroke-width:0;fill:red";
                    let line_string = format!(
                        "<line x1='{}' y1='{}' x2='{}' y2='{}' style='{}' />\n\n",
                        this_center.x, this_center.y, other_center.x, other_center.y, style
                    );
                    ret += &line_string;

                    let circle_string = format!(
                        "<circle cx='{}' cy='{}' r='0' style='{}' />",
                        this_center.x, this_center.y, style
                    );
                    ret += &circle_string;
                }
            }
        }
        ret += "</svg>";
        ret
    }

    fn get_svg(t: &Triangulation3D, p: &Polygon3D) -> String {
        // Get polygon bounding box
        let mut min_x = Float::MAX;
        let mut max_x = Float::MIN;
        let mut min_y = Float::MAX;
        let mut max_y = Float::MIN;
        for point in p.outer().vertices().iter() {
            if point.x > max_x {
                max_x = point.x;
            }
            if point.x < min_x {
                min_x = point.x;
            }
            if point.y > max_y {
                max_y = point.y;
            }
            if point.y < min_y {
                min_y = point.y;
            }
        }
        let height = max_y - min_y;
        let width = max_x - min_x;

        // Arrange a transformation
        const SCALE: Float = 50.;
        const PADDING: Float = 5.; // PErcentage of BBOX
        let padding_x = PADDING * width / 100.;
        let padding_y = PADDING * height / 100.;

        let into_svg_coordinates = |pt: Point3D| -> Point3D {
            Point3D::new(pt.x - min_x + padding_x, padding_y + max_y - pt.y, 0.0)
        };

        let mut ret = format!(
            "<svg width='{}' height='{}'>\n\n",
            SCALE * (2. * padding_x + width),
            SCALE * (2. * padding_y + height)
        );

        /* DRAW POLYGON */
        ret += "<polygon points='";

        for v in p.outer().vertices().iter() {
            let svg_point = into_svg_coordinates(*v);
            ret += &format!("{},{} ", SCALE * svg_point.x, SCALE * svg_point.y).to_string();
        }
        ret += &"' style='fill:#b9b9b9;stroke:black;stroke-width:2;fill-rule:evenodd;' />\n\n";

        let n_inner = p.n_inner_loops();

        for i in 0..n_inner {
            let inner = p.inner(i).unwrap();
            ret += &"<polygon points='";
            for v in inner.vertices().iter() {
                let svg_point = into_svg_coordinates(*v);
                ret += &format!("{},{} ", SCALE * svg_point.x, SCALE * svg_point.y);
            }
            ret += &"' style='fill:white;stroke:black;stroke-width:2;fill-rule:evenodd;' />\n\n";
        }

        ret += "</svg>";

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
        t.add_point_to_triangle(i, Point3D::new(0.0, 0.1, 0.), PointInTriangle::Inside);
        assert_eq!(3, t.triangles.len());

        // Point on AB edge (calls split_edge())
        let mut t = Triangulation3D::new();
        let i = t.push(a, b, c, 0).unwrap();
        t.add_point_to_triangle(i, Point3D::new(0.0, 0.0, 0.), PointInTriangle::EdgeAB);
        assert_eq!(2, t.triangles.len());

        // Point on Vertex (does nothing)
        let mut t = Triangulation3D::new();
        let i = t.push(a, b, c, 0).unwrap();
        t.add_point_to_triangle(i, a, PointInTriangle::VertexA);
        assert_eq!(1, t.triangles.len());
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
        assert_eq!(3, t.triangles.len());

        // Point on AB edge (calls split_edge())
        let mut t = Triangulation3D::new();
        t.push(a, b, c, 0).unwrap();
        t.add_point(Point3D::new(0.0, 0.0, 0.)).unwrap();
        assert_eq!(2, t.triangles.len());

        // Point on Vertex (does nothing)
        let mut t = Triangulation3D::new();
        t.push(a, b, c, 0).unwrap();
        t.add_point(a).unwrap();
        assert_eq!(1, t.triangles.len());
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
        let _a = &t.triangles[0];
    }

    #[test]
    #[should_panic]
    fn test_mark_as_neighbours_panic() {
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

        // This should not work... the edge is incorrect
        assert!(t.mark_as_neighbours(0, Edge::Ab, 1).is_err());

        // This should not work... they are not neighbours.
        assert!(t.mark_as_neighbours(0, Edge::Bc, 2).is_err());
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
    #[should_panic]
    fn test_flip_diagonal_panic() {
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
        t.flip_diagonal(0, Edge::Ab);
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

        t.mark_as_neighbours(0, Edge::Ab, 1).unwrap();
        t.mark_as_neighbours(0, Edge::Bc, 4).unwrap();
        t.mark_as_neighbours(0, Edge::Ca, 5).unwrap();

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

        draw_triangulation(
            "flip_diagonal.html",
            vec![("original", original), ("after", first_flip)],
        );
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
        let a = Point3D::new(-l / 2., -l / 2., 0.);
        let b = Point3D::new(l / 2., -l / 2., 0.);
        let c = Point3D::new(l / 2., l / 2., 0.);
        let d = Point3D::new(-l / 2., l / 2., 0.);
        let e = Point3D::new(-l / 2., -l, 0.);
        let f = Point3D::new(l, -l / 2., 0.);

        let mut t = Triangulation3D::new();
        t.push(a, b, c, 0).unwrap(); // 0
        t.push(a, c, d, 0).unwrap(); // 1
        t.push(e, b, a, 0).unwrap(); // 2
        t.push(b, f, c, 0).unwrap(); // 3

        t.mark_as_neighbours(1, Edge::Ab, 0).unwrap();
        t.mark_as_neighbours(0, Edge::Ab, 2).unwrap();
        t.mark_as_neighbours(0, Edge::Bc, 3).unwrap();

        // constrain AB
        t.triangles[1].constrain(Edge::Ca);

        let original = get_triangulation_svg(&t, -l, l, -l, l);

        // SPLIT!
        let p = Point3D::new(l / 6., -l / 6., 0.);

        // Should work.
        assert!(t.split_triangle(0, p).is_ok());

        let after = get_triangulation_svg(&t, -l, l, -l, l);

        // There should be a total of 4 valid triangles now.
        // Also, 4 triangles in total (the invalid one should
        // have been replaced by a new one.)
        assert_eq!(t.n_valid_triangles(), 6);
        assert_eq!(t.n_triangles(), 6);

        // Check neighbours and constraints.
        let abc = Triangle3D::new(a, b, c).unwrap();

        let acd = Triangle3D::new(a, c, d).unwrap();
        let cpa = Triangle3D::new(c, p, a).unwrap();
        let bpa = Triangle3D::new(b, p, a).unwrap();
        let cpb = Triangle3D::new(c, p, b).unwrap();
        let abe = Triangle3D::new(a, b, e).unwrap();
        let bfc = Triangle3D::new(b, f, c).unwrap();

        assert!(cpa.compare(&t.triangles[0].triangle));
        assert!(acd.compare(&t.triangles[1].triangle));
        assert!(bpa.compare(&t.triangles[4].triangle));
        assert!(cpb.compare(&t.triangles[5].triangle));

        for i in 0..t.n_triangles() {
            let tri = &t.triangles[i];

            if tri.triangle.compare(&abc) {
                panic!("Triangle ABC should not be in Triangulation")
            } else if tri.triangle.compare(&cpa) {
                assert_eq!(tri.index, 0);

                assert_eq!(tri.neighbour(Edge::Ab), Some(1));
                assert_eq!(tri.neighbour(Edge::Bc), Some(4));
                assert_eq!(tri.neighbour(Edge::Ca), Some(5));

                assert!(!tri.is_constrained(Edge::Ab));
                assert!(!tri.is_constrained(Edge::Bc));
                assert!(!tri.is_constrained(Edge::Ca));
            } else if tri.triangle.compare(&acd) {
                assert_eq!(tri.index, 1);

                assert_eq!(tri.neighbour(Edge::Ab), Some(0));
                assert_eq!(tri.neighbour(Edge::Bc), None);
                assert_eq!(tri.neighbour(Edge::Ca), None);

                assert!(!tri.is_constrained(Edge::Ab));
                assert!(!tri.is_constrained(Edge::Bc));
                assert!(tri.is_constrained(Edge::Ca));
            } else if tri.triangle.compare(&abe) {
                assert_eq!(tri.index, 2);

                assert_eq!(tri.neighbour(Edge::Ab), None);
                assert_eq!(tri.neighbour(Edge::Bc), Some(4));
                assert_eq!(tri.neighbour(Edge::Ca), None);

                assert!(!tri.is_constrained(Edge::Ab));
                assert!(!tri.is_constrained(Edge::Bc));
                assert!(!tri.is_constrained(Edge::Ca));
            } else if tri.triangle.compare(&bfc) {
                assert_eq!(tri.index, 3);

                assert_eq!(tri.neighbour(Edge::Ab), None);
                assert_eq!(tri.neighbour(Edge::Bc), None);
                assert_eq!(tri.neighbour(Edge::Ca), Some(5));

                assert!(!tri.is_constrained(Edge::Ab));
                assert!(!tri.is_constrained(Edge::Bc));
                assert!(!tri.is_constrained(Edge::Ca));
            } else if tri.triangle.compare(&bpa) {
                assert_eq!(tri.index, 4);

                assert_eq!(tri.neighbour(Edge::Ab), Some(2));
                assert_eq!(tri.neighbour(Edge::Bc), Some(5));
                assert_eq!(tri.neighbour(Edge::Ca), Some(0));

                assert!(!tri.is_constrained(Edge::Ab));
                assert!(!tri.is_constrained(Edge::Bc));
                assert!(!tri.is_constrained(Edge::Ca));
            } else if tri.triangle.compare(&cpb) {
                assert_eq!(tri.index, 5);

                assert_eq!(tri.neighbour(Edge::Ab), Some(3));
                assert_eq!(tri.neighbour(Edge::Bc), Some(0));
                assert_eq!(tri.neighbour(Edge::Ca), Some(4));

                assert!(!tri.is_constrained(Edge::Ab));
                assert!(!tri.is_constrained(Edge::Bc));
                assert!(!tri.is_constrained(Edge::Ca));
            }
        }

        t.restore_delaunay(1.6);
        let after_delaunay = get_triangulation_svg(&t, -l, l, -l, l);

        draw_triangulation(
            "split_triangle.html",
            vec![
                ("original", original),
                ("after", after),
                ("after restoring", after_delaunay),
            ],
        );
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

        // Check triangles.                          // INDEX:
        let abc = Triangle3D::new(a, b, c).unwrap(); // Should not be there
        let apc = Triangle3D::new(a, p, c).unwrap(); // 0
        let acd = Triangle3D::new(a, c, d).unwrap(); // 1
        let pbc = Triangle3D::new(p, b, c).unwrap(); // 2

        // Check indexes (assigned above)... makes it
        // easier to check the neighbourhood.
        assert!(apc.compare(&t.triangles[0].triangle));
        assert!(acd.compare(&t.triangles[1].triangle));
        assert!(pbc.compare(&t.triangles[2].triangle));

        for i in 0..t.n_triangles() {
            let tri = &t.triangles[i];

            if tri.triangle.compare(&abc) {
                // This should no be aywhere.
                panic!("Triangle ABC should not be in the Triangulation");
            } else if tri.triangle.compare(&apc) {
                assert_eq!(i, 0);

                assert_eq!(tri.neighbour(Edge::Ab), None);
                assert_eq!(tri.neighbour(Edge::Bc), Some(2));
                assert_eq!(tri.neighbour(Edge::Ca), Some(1));

                assert!(tri.is_constrained(Edge::Ab));
                assert!(!tri.is_constrained(Edge::Bc));
                assert!(!tri.is_constrained(Edge::Ca));
            } else if tri.triangle.compare(&acd) {
                assert_eq!(i, 1);

                assert_eq!(tri.neighbour(Edge::Ab), Some(0));
                assert_eq!(tri.neighbour(Edge::Bc), None);
                assert_eq!(tri.neighbour(Edge::Ca), None);

                assert!(!tri.is_constrained(Edge::Ab));
                assert!(!tri.is_constrained(Edge::Bc));
                assert!(!tri.is_constrained(Edge::Ca));
            } else if tri.triangle.compare(&pbc) {
                assert_eq!(i, 2);

                assert_eq!(tri.neighbour(Edge::Ab), None);
                assert_eq!(tri.neighbour(Edge::Bc), None);
                assert_eq!(tri.neighbour(Edge::Ca), Some(0));

                assert!(tri.is_constrained(Edge::Ab));
                assert!(tri.is_constrained(Edge::Bc));
                assert!(!tri.is_constrained(Edge::Ca));
            }
        }

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
        t.push(b, a, opp, 0).unwrap(); //2
        t.push(a, e, opp, 0).unwrap(); //3

        assert!(t.mark_as_neighbours(0, Edge::Ca, 1).is_ok());
        assert!(t.mark_as_neighbours(0, Edge::Ab, 2).is_ok());
        assert!(t.mark_as_neighbours(2, Edge::Bc, 3).is_ok());

        // Constrain BC and OPP-B
        t.triangles[0].constrain(Edge::Bc);
        t.triangles[2].constrain(Edge::Ca);

        assert_eq!(4, t.n_triangles());
        assert_eq!(4, t.n_valid_triangles());

        let two_hemispheres_before = get_triangulation_svg(&t, -l, l, -l, l);

        // SPLIT
        t.split_edge(0, Edge::Ab, p).unwrap();

        assert_eq!(6, t.n_triangles());
        assert_eq!(6, t.n_valid_triangles());

        let two_hemispheres_after = get_triangulation_svg(&t, -l, l, -l, l);

        // Check triangles.                          // INDEX:
        let abc = Triangle3D::new(a, b, c).unwrap(); // Should not be there... replaced by APC
        let bao = Triangle3D::new(a, b, opp).unwrap(); // Should not be there... replaced by BPO

        let apc = Triangle3D::new(a, p, c).unwrap(); // 0
        let apc_index = 0;

        let acd = Triangle3D::new(a, c, d).unwrap(); // 1
        let acd_index = 1;

        let bpo = Triangle3D::new(p, b, opp).unwrap(); // 2
        let bpo_index = 2;

        let aeo = Triangle3D::new(a, opp, e).unwrap(); // 3
        let aeo_index = 3;

        let pbc = Triangle3D::new(p, b, c).unwrap(); // 4
        let pbc_index = 4;

        let pao = Triangle3D::new(p, a, opp).unwrap(); // 2
        let pao_index = 5;

        // Check indexes (assigned above)... makes it
        // easier to check the neighbourhood.
        assert!(apc.compare(&t.triangles[apc_index].triangle));
        assert!(acd.compare(&t.triangles[acd_index].triangle));
        assert!(pao.compare(&t.triangles[pao_index].triangle));
        assert!(aeo.compare(&t.triangles[aeo_index].triangle));
        assert!(pbc.compare(&t.triangles[pbc_index].triangle));
        assert!(bpo.compare(&t.triangles[bpo_index].triangle));

        for i in 0..t.n_triangles() {
            let tri = &t.triangles[i];

            if tri.triangle.compare(&abc) {
                // This should no be aywhere.
                panic!("Triangle ABC should not exist")
            } else if tri.triangle.compare(&bao) {
                // This should no be aywhere.
                panic!("Triangle BAO should not exist")
            } else if tri.triangle.compare(&apc) {
                assert_eq!(i, tri.index);

                assert_eq!(tri.neighbour(Edge::Ab), Some(pao_index));
                assert_eq!(tri.neighbour(Edge::Bc), Some(pbc_index));
                assert_eq!(tri.neighbour(Edge::Ca), Some(acd_index));

                assert!(!tri.is_constrained(Edge::Ab));
                assert!(!tri.is_constrained(Edge::Bc));
                assert!(!tri.is_constrained(Edge::Ca));
            } else if tri.triangle.compare(&acd) {
                assert_eq!(i, tri.index);

                assert_eq!(tri.neighbour(Edge::Ab), Some(apc_index));
                assert_eq!(tri.neighbour(Edge::Bc), None);
                assert_eq!(tri.neighbour(Edge::Ca), None);

                assert!(!tri.is_constrained(Edge::Ab));
                assert!(!tri.is_constrained(Edge::Bc));
                assert!(!tri.is_constrained(Edge::Ca));
            } else if tri.triangle.compare(&bpo) {
                // Constrain BC and OPP-B
                assert_eq!(i, tri.index);

                assert_eq!(tri.neighbour(Edge::Ab), Some(pbc_index));
                assert_eq!(tri.neighbour(Edge::Bc), Some(pao_index));
                assert_eq!(tri.neighbour(Edge::Ca), None);

                assert!(!tri.is_constrained(Edge::Ab));
                assert!(!tri.is_constrained(Edge::Bc));
                assert!(tri.is_constrained(Edge::Ca));
            } else if tri.triangle.compare(&aeo) {
                assert_eq!(i, tri.index);

                assert_eq!(tri.neighbour(Edge::Ab), None);
                assert_eq!(tri.neighbour(Edge::Bc), None);
                assert_eq!(tri.neighbour(Edge::Ca), Some(pao_index));

                assert!(!tri.is_constrained(Edge::Ab));
                assert!(!tri.is_constrained(Edge::Bc));
                assert!(!tri.is_constrained(Edge::Ca));
            } else if tri.triangle.compare(&pbc) {
                // Constrain BC and OPP-B
                assert_eq!(i, tri.index);

                assert_eq!(tri.neighbour(Edge::Ab), Some(bpo_index));
                assert_eq!(tri.neighbour(Edge::Bc), None);
                assert_eq!(tri.neighbour(Edge::Ca), Some(apc_index));

                assert!(!tri.is_constrained(Edge::Ab));
                assert!(tri.is_constrained(Edge::Bc));
                assert!(!tri.is_constrained(Edge::Ca));
            } else if tri.triangle.compare(&pao) {
                assert_eq!(i, tri.index);

                assert_eq!(tri.neighbour(Edge::Ab), Some(apc_index));
                assert_eq!(tri.neighbour(Edge::Bc), Some(aeo_index));
                assert_eq!(tri.neighbour(Edge::Ca), Some(bpo_index));

                assert!(!tri.is_constrained(Edge::Ab));
                assert!(!tri.is_constrained(Edge::Bc));
                assert!(!tri.is_constrained(Edge::Ca));
            }
        }

        draw_triangulation(
            "split_edge.html",
            vec![
                ("One hemisphere - original", one_hemisphere_before),
                ("One hemisphere - after", one_hemisphere_after),
                ("Two hemispheres - original", two_hemispheres_before),
                ("Two hemispheres - after", two_hemispheres_after),
            ],
        );
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
        // let min_x = 0.;
        // let max_x = 5.;
        // let min_y = 0.;
        // let max_y = 6.;
        let p0 = Point3D::new(0., 0., 0.);
        let p1 = Point3D::new(3., 0., 0.);
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
        assert_eq!(6, outer.len());

        let poly = Polygon3D::new(outer).unwrap();
        let max_area = 0.1;
        let max_aspect_ratio = 1.6;

        let t = Triangulation3D::mesh_polygon(&poly, max_area, max_aspect_ratio).unwrap();
        let case1 = get_svg(&t, &poly);
        // test_triangulation_results(&t, &poly, max_area, max_aspect_ratio).unwrap();

        /* SOMEHOW MORE COMPLICATED CASE ... 6 vertices + hole */
        // let min_x = 0.;
        // let max_x = 5.;
        // let min_y = 0.;
        // let max_y = 6.;
        let p0 = Point3D::new(0., 0., 0.);
        let p1 = Point3D::new(3., 0., 0.);
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
        let mut poly = Polygon3D::new(outer).unwrap();

        let i0 = Point3D::new(0.2, 0.2, 0.);
        let i1 = Point3D::new(0.8, 0.2, 0.);
        let i2 = Point3D::new(0.8, 1.2, 0.);
        let i3 = Point3D::new(0.2, 1.2, 0.);
        let mut inner = Loop3D::new();
        inner.push(i0).unwrap();
        inner.push(i1).unwrap();
        inner.push(i2).unwrap();
        inner.push(i3).unwrap();
        inner.close().unwrap();
        poly.cut_hole(inner).unwrap();

        let i0 = Point3D::new(1.2, 0.6, 0.);
        let i1 = Point3D::new(2., 0.6, 0.);
        let i2 = Point3D::new(2., 3.2, 0.);
        let i3 = Point3D::new(1.2, 3.2, 0.);
        let mut inner = Loop3D::new();
        inner.push(i0).unwrap();
        inner.push(i1).unwrap();
        inner.push(i2).unwrap();
        inner.push(i3).unwrap();
        inner.close().unwrap();
        poly.cut_hole(inner).unwrap();

        let max_area = 0.4;
        let max_aspect_ratio = 1.6;

        let t = Triangulation3D::mesh_polygon(&poly, max_area, max_aspect_ratio).unwrap();
        let case2 = get_svg(&t, &poly);
        // test_triangulation_results(&t, &poly, max_area, max_aspect_ratio).unwrap();

        draw_triangulation(
            "test_mesh_polygon.html",
            vec![("Case 0", case0), ("Case 1", case1), ("Case 2", case2)],
        );
    }

    #[test]
    fn test_from_polygon() {
        /* SIMPLE CASE ... three vertices */
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

        /* SOMEHOW MORE COMPLICATED CASE ... 6 vertices */
        let p0 = Point3D::new(0., 0., 0.);
        let p1 = Point3D::new(3., 3., 0.); // Collinear... disapears
        let p2 = Point3D::new(5., 5., 0.);
        let p3 = Point3D::new(3., 6., 0.);
        let p4 = Point3D::new(0., 5., 0.);
        let p5 = Point3D::new(0., 3., 0.); // Collinear, disappear

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
        let p1 = Point3D::new(3., 0., 0.);
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

        /* SOMEHOW MORE COMPLICATED CASE ... 6 vertices + hole */
        let p0 = Point3D::new(0., 0., 0.);
        let p1 = Point3D::new(3., 0., 0.);
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
        let mut poly = Polygon3D::new(outer).unwrap();

        let i0 = Point3D::new(0.2, 0.2, 0.);
        let i1 = Point3D::new(0.8, 0.2, 0.);
        let i2 = Point3D::new(0.8, 1.2, 0.);
        let i3 = Point3D::new(0.2, 1.2, 0.);
        let mut inner = Loop3D::new();
        inner.push(i0).unwrap();
        inner.push(i1).unwrap();
        inner.push(i2).unwrap();
        inner.push(i3).unwrap();
        inner.close().unwrap();
        poly.cut_hole(inner).unwrap();

        let i0 = Point3D::new(1.2, 0.6, 0.);
        let i1 = Point3D::new(2., 0.6, 0.);
        let i2 = Point3D::new(2., 3.2, 0.);
        let i3 = Point3D::new(1.2, 3.2, 0.);
        let mut inner = Loop3D::new();
        inner.push(i0).unwrap();
        inner.push(i1).unwrap();
        inner.push(i2).unwrap();
        inner.push(i3).unwrap();
        inner.close().unwrap();
        poly.cut_hole(inner).unwrap();

        let t = Triangulation3D::from_polygon(&poly).unwrap();
        test_triangulation_results(&t, &poly, 1000., 1000.).unwrap();

        let mut closed = poly.get_closed_loop();
        closed.close().unwrap();
        let poly = Polygon3D::new(closed).unwrap();

        let case3 = get_svg(&t, &poly);

        draw_triangulation(
            "test_from_polygon.html",
            vec![
                ("Case 0", case0),
                ("Case 1", case1),
                ("Case 2", case2),
                ("Case 3", case3),
            ],
        )
    }

    #[test]
    fn test_mark_neighbourhoods() {
        let p0 = Point3D::new(0., 0., 0.);
        let p1 = Point3D::new(3., 0., 0.);
        let p2 = Point3D::new(3., 3., 0.);
        let p3 = Point3D::new(5., 5., 0.);
        let p4 = Point3D::new(3., 6., 0.);
        let p5 = Point3D::new(0., 5., 0.);

        let mut t = Triangulation3D::new();
        t.push(p0, p1, p2, 0).unwrap();
        t.push(p2, p4, p3, 0).unwrap();
        t.push(p2, p5, p4, 0).unwrap();
        t.push(p0, p2, p5, 0).unwrap();

        t.mark_neighbourhouds().unwrap();

        assert_eq!(t.triangles[0].neighbour(Edge::Ab), None);
        assert_eq!(t.triangles[0].neighbour(Edge::Bc), None);
        assert_eq!(t.triangles[0].neighbour(Edge::Ca), Some(3));

        assert_eq!(t.triangles[1].neighbour(Edge::Ab), Some(2));
        assert_eq!(t.triangles[1].neighbour(Edge::Bc), None);
        assert_eq!(t.triangles[1].neighbour(Edge::Ca), None);

        assert_eq!(t.triangles[2].neighbour(Edge::Ab), Some(3));
        assert_eq!(t.triangles[2].neighbour(Edge::Bc), None);
        assert_eq!(t.triangles[2].neighbour(Edge::Ca), Some(1));

        assert_eq!(t.triangles[3].neighbour(Edge::Ab), Some(0));
        assert_eq!(t.triangles[3].neighbour(Edge::Bc), Some(2));
        assert_eq!(t.triangles[3].neighbour(Edge::Ca), None);
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
    fn test_add_edge() {
        let edge = Edge::Ab;
        assert_eq!(Edge::Bc, edge + 1);
        assert_eq!(Edge::Ca, edge + 2);
        assert_eq!(Edge::Ab, edge + 3);
    }
} // end of test module
