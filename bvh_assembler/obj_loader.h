#pragma once

#include <string>
#include <vector>
#include <cstdint>

bool LoadOBJ(
    const std::string& path,
    std::vector<float>& out_vertices,
    std::vector<uint32_t>& out_indices
);
