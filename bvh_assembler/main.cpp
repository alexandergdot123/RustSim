// main.cpp

#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <cassert>
#include <cstdint>

// IMPORTANT: define these in exactly ONE .cpp (this one).
#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

#define TINYBVH_IMPLEMENTATION
#include "tiny_bvh.h"

// =========================
// OBJ LOADER
// =========================

static bool LoadOBJ(
    const std::string& path,
    std::vector<float>& out_vertices,      // xyzxyz...
    std::vector<uint32_t>& out_indices     // vertex indices (triangulated)
) {
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string warn, err;

    // triangulate = true
    bool ok = tinyobj::LoadObj(
        &attrib,
        &shapes,
        &materials,
        &warn,
        &err,
        path.c_str(),
        nullptr,
        true
    );

    if (!warn.empty()) std::cerr << "tinyobj warn: " << warn << "\n";
    if (!err.empty())  std::cerr << "tinyobj err: "  << err  << "\n";
    if (!ok) return false;

    out_vertices = attrib.vertices;

    // Collect triangle indices (vertex_index only)
    out_indices.clear();
    for (const auto& shape : shapes) {
        // With triangulate=true, faces become triangles, but tinyobj still stores a flat index list.
        // Each index corresponds to one vertex of a triangle.
        for (const auto& idx : shape.mesh.indices) {
            if (idx.vertex_index < 0) {
                std::cerr << "ERROR: negative vertex_index in OBJ.\n";
                return false;
            }
            out_indices.push_back(static_cast<uint32_t>(idx.vertex_index));
        }
    }

    if (out_indices.size() % 3 != 0) {
        std::cerr << "ERROR: index count not multiple of 3 after triangulation.\n";
        return false;
    }
    return true;
}

// =========================
// DUMP FUNCTIONS
// =========================

static void DumpNodes(const tinybvh::BVH& bvh, const char* filename = "bvh_nodes.txt")
{
    std::ofstream out(filename);
    out << "# node_id  min(x y z)  max(x y z)  leftFirst  triCount\n";

    for (uint32_t i = 0; i < bvh.usedNodes; i++) {
        const auto& n = bvh.bvhNode[i];
        out << i << "  "
            << n.aabbMin.x << " " << n.aabbMin.y << " " << n.aabbMin.z << "   "
            << n.aabbMax.x << " " << n.aabbMax.y << " " << n.aabbMax.z << "   "
            << n.leftFirst << "  "
            << n.triCount << "\n";
    }
}

static void DumpLeafRanges(const tinybvh::BVH& bvh, const char* filename = "bvh_leaves.txt")
{
    std::ofstream out(filename);
    out << "# node_id  firstTri  triCount\n";

    for (uint32_t i = 0; i < bvh.usedNodes; i++) {
        const auto& n = bvh.bvhNode[i];
        if (n.triCount > 0) { // leaf
            out << i << " " << n.leftFirst << " " << n.triCount << "\n";
        }
    }
}

static void DumpTrianglesSoup(
    const std::vector<tinybvh::bvhvec4>& tris, // 3 verts per tri
    const char* filename = "bvh_triangles.txt"
) {
    std::ofstream out(filename);
    out << "# tri_id  v0(x y z)  v1(x y z)  v2(x y z)\n";

    uint32_t triCount = static_cast<uint32_t>(tris.size() / 3);
    for (uint32_t t = 0; t < triCount; t++) {
        const auto& v0 = tris[t * 3 + 0];
        const auto& v1 = tris[t * 3 + 1];
        const auto& v2 = tris[t * 3 + 2];

        out << t << "  "
            << v0.x << " " << v0.y << " " << v0.z << "   "
            << v1.x << " " << v1.y << " " << v1.z << "   "
            << v2.x << " " << v2.y << " " << v2.z << "\n";
    }
}

// This is the “node -> vertices owned by node” view.
// For each leaf node, it writes all triangles in that leaf, including their vertex positions.
static void DumpLeafTriangles(
    const tinybvh::BVH& bvh,
    const std::vector<tinybvh::bvhvec4>& tris,
    const char* filename = "bvh_leaf_triangles.txt"
) {
    std::ofstream out(filename);
    out << "# leaf_node_id  tri_id  v0(x y z)  v1(x y z)  v2(x y z)\n";

    const uint32_t triCountTotal = static_cast<uint32_t>(tris.size() / 3);

    for (uint32_t nodeId = 0; nodeId < bvh.usedNodes; nodeId++) {
        const auto& n = bvh.bvhNode[nodeId];
        if (n.triCount == 0) continue; // not a leaf

        const uint32_t first = n.leftFirst;
        const uint32_t count = n.triCount;

        // sanity check against our triangle array
        if (first + count > triCountTotal) {
            out << "# ERROR: leaf range out of bounds for node " << nodeId
                << " first=" << first << " count=" << count
                << " total=" << triCountTotal << "\n";
            continue;
        }

        for (uint32_t t = 0; t < count; t++) {
            const uint32_t triId = first + t;

            const auto& v0 = tris[triId * 3 + 0];
            const auto& v1 = tris[triId * 3 + 1];
            const auto& v2 = tris[triId * 3 + 2];

            out << nodeId << "  " << triId << "  "
                << v0.x << " " << v0.y << " " << v0.z << "   "
                << v1.x << " " << v1.y << " " << v1.z << "   "
                << v2.x << " " << v2.y << " " << v2.z << "\n";
        }
    }
}

// =========================
// MAIN
// =========================

int main()
{
    std::vector<float> raw_vertices;
    std::vector<uint32_t> indices;

    if (!LoadOBJ("room.obj", raw_vertices, indices)) {
        std::cerr << "Failed to load OBJ\n";
        return 1;
    }

    const uint32_t triCount = static_cast<uint32_t>(indices.size() / 3);
    std::cout << "OBJ triangles: " << triCount << "\n";

    // =========================
    // BUILD TRIANGLE SOUP
    // =========================

    // triangle soup: 3 bvhvec4 per triangle (v0,v1,v2)
    std::vector<tinybvh::bvhvec4> tris;
    tris.reserve(triCount * 3);

    const size_t vertCount = raw_vertices.size() / 3;

    for (uint32_t i = 0; i < indices.size(); i++) {
        uint32_t idx = indices[i];
        if (idx >= vertCount) {
            std::cerr << "ERROR: OBJ index out of range: " << idx
                      << " (vertCount=" << vertCount << ")\n";
            return 1;
        }
        tris.emplace_back(
            raw_vertices[idx * 3 + 0],
            raw_vertices[idx * 3 + 1],
            raw_vertices[idx * 3 + 2],
            1.0f
        );
    }

    assert(tris.size() % 3 == 0);

    // =========================
    // BUILD BVH FROM SOUP
    // =========================

    tinybvh::BVH bvh;
    bvh.Build(tris.data(), triCount);

    std::cout << "BVH triangles: " << bvh.triCount << "\n";
    std::cout << "BVH usedNodes: " << bvh.usedNodes << "\n";

    // =========================
    // DUMP EVERYTHING
    // =========================

    DumpNodes(bvh);
    DumpLeafRanges(bvh);

    // IMPORTANT NOTE:
    // tinybvh may reorder tris during Build to make leaf triangle ranges contiguous.
    // So dumping tris AFTER Build is what you want if you’re using leaf ranges.
    DumpTrianglesSoup(tris);
    DumpLeafTriangles(bvh, tris);

    std::cout << "Dump complete\n";
    return 0;
}
