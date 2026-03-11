#include "obj_loader.h"

#include <iostream>
#include "tiny_obj_loader.h"

bool LoadOBJ(
    const std::string& path,
    std::vector<float>& out_vertices,
    std::vector<uint32_t>& out_indices
) {
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;

    std::string warn, err;

    std::string base_dir = path.substr(0, path.find_last_of("/\\") + 1);

    bool ok = tinyobj::LoadObj(
        &attrib,
        &shapes,
        &materials,
        &warn,
        &err,
        path.c_str(),
        base_dir.c_str(),
        true
    );

    if (!warn.empty()) std::cerr << warn << "\n";
    if (!err.empty())  std::cerr << err << "\n";
    if (!ok) return false;

    out_vertices = attrib.vertices;
    out_indices.clear();

    for (const auto& shape : shapes) {
        for (const auto& idx : shape.mesh.indices) {
            out_indices.push_back(
                static_cast<uint32_t>(idx.vertex_index)
            );
        }
    }

    return true;
}
