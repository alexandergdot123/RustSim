#pragma once

#include <vector>
#include <cstdint>

#include "tiny_bvh.h"   // ← THIS WAS MISSING

void BuildBVH(
    const std::vector<float>& vertices,
    const std::vector<uint32_t>& indices,
    tinybvh::BVH& out_bvh
);
