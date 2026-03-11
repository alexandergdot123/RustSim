#include "bvh_builder.h"

using namespace tinybvh;

void BuildBVH(
    const std::vector<float>& vertices,
    const std::vector<uint32_t>& indices,
    BVH& bvh
) {
    const uint32_t tri_count = static_cast<uint32_t>(indices.size() / 3);

    std::vector<bvhvec4> tris;
    tris.reserve(tri_count * 3);

    for (uint32_t i = 0; i < tri_count; ++i) {
        for (int v = 0; v < 3; ++v) {
            uint32_t idx = indices[i * 3 + v];
            tris.emplace_back(
                vertices[idx * 3 + 0],
                vertices[idx * 3 + 1],
                vertices[idx * 3 + 2],
                0.0f
            );
        }
    }

    bvh.Build(tris.data(), tri_count);
}
