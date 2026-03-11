#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Point {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[derive(Hash, Eq, PartialEq, Copy, Clone, Debug)]
struct QPoint {
    x: i64,
    y: i64,
    z: i64,
}

fn snap(point: Point) -> QPoint {
    QPoint {
        x: (point.x * 10000.0) as i64,
        y: (point.y * 10000.0) as i64,
        z: (point.z * 10000.0) as i64,
    }
}

impl Point {
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }
}
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Node {
    index: usize,
    min: Point,
    max: Point,
    left_child: usize,
    is_leaf: bool,
    right_child: usize,
    tri_count: usize,
    first_tri: usize,
}

use std::fs::File;
use std::io::{BufRead, BufReader};

fn read_nodes(path: String) -> Vec<Node> {
    let file = File::open(path).expect("failed to open node file");
    let reader = BufReader::new(file);

    let mut nodes = Vec::new();

    for line in reader.lines() {
        let line = line.expect("failed to read line");
        let line = line.trim();

        // skip empty lines and comments / header
        if line.is_empty() || line.starts_with('#') {
            continue;
        }

        let parts: Vec<&str> = line.split_whitespace().collect();
        assert!(
            parts.len() == 9,
            "expected 9 fields per line, got {}: {}",
            parts.len(),
            line
        );

        let index: usize = parts[0].parse().unwrap();

        let min = Point::new(
            parts[1].parse().unwrap(),
            parts[2].parse().unwrap(),
            parts[3].parse().unwrap(),
        );

        let max = Point::new(
            parts[4].parse().unwrap(),
            parts[5].parse().unwrap(),
            parts[6].parse().unwrap(),
        );

        let left_first: usize = parts[7].parse().unwrap();
        let tri_count: usize = parts[8].parse().unwrap();

        let (left_child, right_child) = if tri_count == 0 {
            (left_first, left_first + 1)
        } else {
            (0, 0)
        };

        nodes.push(Node {
            index,
            min,
            max,
            is_leaf: tri_count != 0,
            left_child,
            right_child,
            tri_count,
            first_tri: 0,
        });
    }

    nodes
}


#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Indices {
    pub node_index: usize,
    pub first_triangle_index: usize,
    pub num_triangles: usize,
}

fn read_indices(path: String) -> Vec<Indices> {
    let file = File::open(path).expect("failed to open indices file");
    let reader = BufReader::new(file);

    let mut indices = Vec::new();

    for line in reader.lines() {
        let line = line.expect("failed to read line");
        let line = line.trim();

        // skip header / comments / empty lines
        if line.is_empty() || line.starts_with('#') {
            continue;
        }

        let parts: Vec<&str> = line.split_whitespace().collect();
        assert!(
            parts.len() == 3,
            "expected 3 fields per line, got {}: {}",
            parts.len(),
            line
        );

        indices.push(Indices {
            node_index: parts[0].parse().unwrap(),
            first_triangle_index: parts[1].parse().unwrap(),
            num_triangles: parts[2].parse().unwrap(),
        });
    }

    indices
}

#[derive(Debug, Copy, Clone)]
pub struct Triangle {
    pub index: usize,
    pub v0: Point,
    pub v1: Point,
    pub v2: Point,
}


fn read_triangles(path: String) -> Vec<Triangle> {
    let file = File::open(path).expect("failed to open triangle file");
    let reader = BufReader::new(file);

    let mut tris = Vec::new();

    for line in reader.lines() {
        let line = line.expect("failed to read line");
        let line = line.trim();

        // skip header / comments / empty lines
        if line.is_empty() || line.starts_with('#') {
            continue;
        }

        let parts: Vec<&str> = line.split_whitespace().collect();
        assert!(
            parts.len() == 10,
            "expected 10 fields per line, got {}: {}",
            parts.len(),
            line
        );

        let index: usize = parts[0].parse().unwrap();

        let v0 = Point::new(
            parts[1].parse().unwrap(),
            parts[2].parse().unwrap(),
            parts[3].parse().unwrap(),
        );

        let v1 = Point::new(
            parts[4].parse().unwrap(),
            parts[5].parse().unwrap(),
            parts[6].parse().unwrap(),
        );

        let v2 = Point::new(
            parts[7].parse().unwrap(),
            parts[8].parse().unwrap(),
            parts[9].parse().unwrap(),
        );

        tris.push(Triangle { index, v0, v1, v2 });
    }

    tris
}

/*
function dfs(node):
    size = X + leaf_tri_count(node) * Z
    subtree_count = 0

    for child in node.children:
        (child_size, child_subtrees) = dfs(child)
        subtree_count += child_subtrees

        if size + child_size <= A:
            size += child_size
        else:
            // cannot include child; must cut here
            subtree_count += 1
            // child becomes a separate subtree

    return (size, subtree_count)
    
*/

use hashbrown::HashMap;
const INDEX_SIZE: usize = 2;
const POINT_SIZE: usize = 4 * 3;
const NODE_SIZE: usize = 2 * POINT_SIZE + 2 + 2; // min, max, left_child, num_indices
const MATERIAL_POINTER_SIZE: usize = 4;
fn traverse_tree(index: usize, nodes: &Vec<Node>, triangles: &Vec<Triangle>, max_alloc: usize) -> (usize, Option<HashMap<QPoint, usize>>, Vec<usize>) {
    let node_in_focus = &nodes[index];
    if node_in_focus.is_leaf {
        // println!("I Assume this sometimes happens");
        let mut size = NODE_SIZE;
        let mut point_list =  HashMap::new();
        for i in node_in_focus.first_tri..node_in_focus.first_tri + node_in_focus.tri_count {
            size += INDEX_SIZE * 3;
            size += MATERIAL_POINTER_SIZE;
            point_list.insert(snap(triangles[i].v0), 1);
            point_list.insert(snap(triangles[i].v1), 1);
            point_list.insert(snap(triangles[i].v2), 1);
        }
        assert!(size + point_list.len() * POINT_SIZE <= max_alloc, "SIZE OF A SINGLE LEAF DOES NOT FIT INTO A CORE");
        return (size, Some(point_list), vec![]);
    }
    let (left_size, left_used_points, mut left_subtrees) = traverse_tree(node_in_focus.left_child, nodes, triangles, max_alloc);
    let (right_size, right_used_points, mut right_subtrees) = traverse_tree(node_in_focus.right_child, nodes, triangles, max_alloc);
    if left_subtrees.is_empty() && right_subtrees.is_empty() {
        let mut conjoined_map = left_used_points.unwrap().clone();
        conjoined_map.extend(right_used_points.unwrap());
        if left_size + right_size + NODE_SIZE + conjoined_map.len() * POINT_SIZE <= max_alloc{
            (left_size + right_size + NODE_SIZE, Some(conjoined_map), vec![])
        }
        else {
            // println!("Joined");
            (0, None, vec![node_in_focus.left_child, node_in_focus.right_child])
        }
    }
    else if left_subtrees.is_empty() && !right_subtrees.is_empty() {
        right_subtrees.push(node_in_focus.left_child);
        (0, None, right_subtrees)
    }
    else if !left_subtrees.is_empty() && right_subtrees.is_empty() {
        left_subtrees.push(node_in_focus.right_child);
        (0, None, left_subtrees)
    }
    else {
        left_subtrees.append(&mut right_subtrees);
        (0, None, left_subtrees)
    }
}



const LEAF_INDEX_SIZE: usize = 2;
fn traverse_branches(index: usize, nodes: &Vec<Node>, max_alloc: usize) -> (usize, Vec<usize>) {
    let node_in_focus = &nodes[index];
    if node_in_focus.is_leaf {
        return (LEAF_INDEX_SIZE, vec![]);
    }
    let (left_size, mut left_subtrees) = traverse_branches(node_in_focus.left_child, nodes, max_alloc);
    let (right_size, mut right_subtrees) = traverse_branches(node_in_focus.right_child, nodes, max_alloc);
    if left_subtrees.is_empty() && right_subtrees.is_empty() {
        if left_size + right_size + NODE_SIZE <= max_alloc{
            (left_size + right_size + NODE_SIZE, vec![])
        }
        else {
            (0, vec![node_in_focus.left_child, node_in_focus.right_child])
        }
    }
    else if left_subtrees.is_empty() && !right_subtrees.is_empty() {
        right_subtrees.push(node_in_focus.left_child);
        (0, right_subtrees)
    }
    else if !left_subtrees.is_empty() && right_subtrees.is_empty() {
        left_subtrees.push(node_in_focus.right_child);
        (0, left_subtrees)
    }
    else {
        left_subtrees.append(&mut right_subtrees);
        (0, left_subtrees)
    }
}


fn count_subtree_nodes(index: usize, nodes: &Vec<Node>) -> (usize, usize) {
    let node = &nodes[index];

    if node.is_leaf {
        (node.tri_count, 1)
    } else {
        let (a, b) = count_subtree_nodes(node.left_child, nodes);
        let (c, d) = count_subtree_nodes(node.right_child, nodes);
        (a + c, 1 + b + d)
    }
}

pub fn assemble_tree(subfolder: String) {
    let mut bvh_leaves_path = subfolder.clone();
    bvh_leaves_path.push_str("\\bvh_leaves.txt");
    let mut bvh_nodes_path = subfolder.clone();
    bvh_nodes_path.push_str("\\bvh_nodes.txt");
    let mut bvh_triangles_path = subfolder.clone();
    bvh_triangles_path.push_str("\\bvh_triangles.txt");

    let triangles = read_triangles(bvh_triangles_path);
    let indices = read_indices(bvh_leaves_path);
    let mut nodes = read_nodes(bvh_nodes_path);
    let mut expanded_indices: Vec<Indices> = vec![Indices{node_index: 0, first_triangle_index: 0, num_triangles:0}; 2_000_000];
    for index in indices {
        expanded_indices[index.node_index] = index;
    }

    for node in nodes.iter_mut() {
        node.first_tri = expanded_indices[node.index].first_triangle_index;
    }

    let (_, _, subtree_list) = traverse_tree(0, &nodes, &triangles, 32*1024);
    println!("NUMBER OF CORES REQUIRED IN TOTAL WHICH WORK ON LEAVES: {}", subtree_list.len());

    let mut branch_nodes = nodes.clone();

    for node in subtree_list {
        branch_nodes[node].is_leaf = true;
    }
    let (_, first_branches) = traverse_branches(0, &branch_nodes, 32*1024);

    println!("NUMBER OF CORES REQUIRED IN TOTAL WHICH WORK ON BRANCHES: {}", first_branches.len());

    for branch in first_branches {
        let (triangles, nodes) = count_subtree_nodes(branch, &nodes);
        println!("NUM NODES OWNED: {}, TRIS OWNED: {}", nodes, triangles);
        println!("Branch: {}", branch);
    }
}