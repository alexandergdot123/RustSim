// =============================================================================
// Wavefront Path Tracer - CUDA Skeleton
// Demonstrates: wavefront decomposition, persistent threads, SoA rays,
//               material sorting, shadow ray batching, coherence recovery
// =============================================================================
//ALEX NOTE - AI GENERATED EXAMPLE
#include <cuda_runtime.h>
#include <cstdint>
#include <cfloat>

// ---------------------------------------------------------------------------
// Config
// ---------------------------------------------------------------------------
#define WARP_SIZE       32
#define BLOCK_SIZE      256
#define MAX_RAYS        (1 << 22)  // 4M rays
#define MAX_TRIS        (1 << 20)
#define MAX_BVH_NODES   (1 << 20)
#define BVH_STACK_DEPTH 64
#define MAX_BOUNCES     5

// ---------------------------------------------------------------------------
// BVH Node - 64 bytes, cache-line aligned
// Using BVH2 for clarity; real impl would use BVH4/BVH8
// ---------------------------------------------------------------------------
struct __align__(64) BVHNode {
    float min_x, min_y, min_z;
    float max_x, max_y, max_z;
    int left;           // child index or ~tri_start if leaf (negative = leaf)
    int right;          // child index or tri_count if leaf
};

struct Triangle {
    float v0x, v0y, v0z;
    float e1x, e1y, e1z;   // v1 - v0
    float e2x, e2y, e2z;   // v2 - v0
    uint32_t material_id;
    float nx, ny, nz;       // face normal (precomputed)
};

// ---------------------------------------------------------------------------
// SoA Ray Pool - coalesced memory access pattern
// Each field is a contiguous array so warp reads are 128-byte transactions
// ---------------------------------------------------------------------------
struct RayPoolSoA {
    // Origin
    float* ox;
    float* oy;
    float* oz;
    // Direction
    float* dx;
    float* dy;
    float* dz;
    // Precomputed inverse direction (avoids repeated division in AABB test)
    float* inv_dx;
    float* inv_dy;
    float* inv_dz;
    // Hit info
    float* t_max;
    int*   tri_index;       // -1 if no hit
    float* hit_u;           // barycentric
    float* hit_v;
    // Pixel and path metadata
    uint32_t* pixel_index;  // linear pixel index
    uint32_t* material_id;  // filled after intersection, used for sorting
    uint8_t*  bounce;
    uint8_t*  active;       // 0 = dead, 1 = active

    // Throughput (accumulated color weight along path)
    float* throughput_r;
    float* throughput_g;
    float* throughput_b;
};

// ---------------------------------------------------------------------------
// Atomic work queue for persistent threads
// ---------------------------------------------------------------------------
struct WorkQueue {
    int* ray_indices;       // indices into RayPoolSoA
    int  count;             // number of work items
    int  _pad[15];          // pad to avoid false sharing on counter
};

__device__ int g_work_counter;  // global atomic counter for persistent threads

// ---------------------------------------------------------------------------
// DEVICE: Moller-Trumbore ray-triangle intersection
// ---------------------------------------------------------------------------
__device__ __forceinline__
bool intersect_triangle(
    float ox, float oy, float oz,
    float dx, float dy, float dz,
    const Triangle& tri,
    float t_max,
    float& out_t, float& out_u, float& out_v)
{
    float pvec_x = dy * tri.e2z - dz * tri.e2y;
    float pvec_y = dz * tri.e2x - dx * tri.e2z;
    float pvec_z = dx * tri.e2y - dy * tri.e2x;

    float det = tri.e1x * pvec_x + tri.e1y * pvec_y + tri.e1z * pvec_z;
    if (fabsf(det) < 1e-8f) return false;

    float inv_det = 1.0f / det;

    float tvec_x = ox - tri.v0x;
    float tvec_y = oy - tri.v0y;
    float tvec_z = oz - tri.v0z;

    float u = (tvec_x * pvec_x + tvec_y * pvec_y + tvec_z * pvec_z) * inv_det;
    if (u < 0.0f || u > 1.0f) return false;

    float qvec_x = tvec_y * tri.e1z - tvec_z * tri.e1y;
    float qvec_y = tvec_z * tri.e1x - tvec_x * tri.e1z;
    float qvec_z = tvec_x * tri.e1y - tvec_y * tri.e1x;

    float v = (dx * qvec_x + dy * qvec_y + dz * qvec_z) * inv_det;
    if (v < 0.0f || u + v > 1.0f) return false;

    float t = (tri.e2x * qvec_x + tri.e2y * qvec_y + tri.e2z * qvec_z) * inv_det;
    if (t < 1e-4f || t >= t_max) return false;

    out_t = t;
    out_u = u;
    out_v = v;
    return true;
}

// ---------------------------------------------------------------------------
// DEVICE: AABB slab test using precomputed inverse direction
// ---------------------------------------------------------------------------
__device__ __forceinline__
bool intersect_aabb(
    float ox, float oy, float oz,
    float inv_dx, float inv_dy, float inv_dz,
    const BVHNode& node,
    float t_max)
{
    float tx1 = (node.min_x - ox) * inv_dx;
    float tx2 = (node.max_x - ox) * inv_dx;
    float tmin = fminf(tx1, tx2);
    float tmax = fmaxf(tx1, tx2);

    float ty1 = (node.min_y - oy) * inv_dy;
    float ty2 = (node.max_y - oy) * inv_dy;
    tmin = fmaxf(tmin, fminf(ty1, ty2));
    tmax = fminf(tmax, fmaxf(ty1, ty2));

    float tz1 = (node.min_z - oz) * inv_dz;
    float tz2 = (node.max_z - oz) * inv_dz;
    tmin = fmaxf(tmin, fminf(tz1, tz2));
    tmax = fminf(tmax, fmaxf(tz1, tz2));

    return tmax >= fmaxf(tmin, 0.0f) && tmin < t_max;
}

// ===========================================================================
// KERNEL 1: Ray Generation
// One thread per pixel, generates primary camera rays
// ===========================================================================
__global__ void kernel_generate_primary_rays(
    RayPoolSoA rays,
    int width, int height,
    float3 cam_origin, float3 cam_forward, float3 cam_right, float3 cam_up,
    float fov_scale,
    int* ray_count)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int total_pixels = width * height;
    if (idx >= total_pixels) return;

    int px = idx % width;
    int py = idx / width;

    // Jittered sampling could go here (pass in per-frame RNG state)
    float u = (2.0f * ((float)px + 0.5f) / (float)width  - 1.0f) * fov_scale;
    float v = (2.0f * ((float)py + 0.5f) / (float)height - 1.0f) * fov_scale * ((float)height / (float)width);

    float dir_x = cam_forward.x + u * cam_right.x + v * cam_up.x;
    float dir_y = cam_forward.y + u * cam_right.y + v * cam_up.y;
    float dir_z = cam_forward.z + u * cam_right.z + v * cam_up.z;

    // Normalize
    float inv_len = rsqrtf(dir_x * dir_x + dir_y * dir_y + dir_z * dir_z);
    dir_x *= inv_len;
    dir_y *= inv_len;
    dir_z *= inv_len;

    // Write SoA - every field write is coalesced across the warp
    rays.ox[idx] = cam_origin.x;
    rays.oy[idx] = cam_origin.y;
    rays.oz[idx] = cam_origin.z;
    rays.dx[idx] = dir_x;
    rays.dy[idx] = dir_y;
    rays.dz[idx] = dir_z;
    rays.inv_dx[idx] = 1.0f / dir_x;
    rays.inv_dy[idx] = 1.0f / dir_y;
    rays.inv_dz[idx] = 1.0f / dir_z;
    rays.t_max[idx]      = FLT_MAX;
    rays.tri_index[idx]  = -1;
    rays.pixel_index[idx] = idx;
    rays.material_id[idx] = 0;
    rays.bounce[idx]      = 0;
    rays.active[idx]      = 1;
    rays.throughput_r[idx] = 1.0f;
    rays.throughput_g[idx] = 1.0f;
    rays.throughput_b[idx] = 1.0f;

    // First thread sets the count
    if (idx == 0) *ray_count = total_pixels;
}

// ===========================================================================
// KERNEL 2: BVH Traversal + Intersection (Persistent Threads)
//
// Key optimization: instead of 1 thread per ray (where some warps finish
// early and SMs go idle), we launch exactly enough threads to fill the GPU
// and each thread grabs rays from a global queue in a loop.
// ===========================================================================
__global__ void kernel_traverse_persistent(
    RayPoolSoA rays,
    const int* __restrict__ active_indices,  // compacted list of active ray indices
    int num_active,
    const BVHNode* __restrict__ bvh_nodes,
    const Triangle* __restrict__ triangles)
{
    // --- Persistent thread loop ---
    // Each thread atomically grabs the next ray to process
    while (true) {
        int work_idx = atomicAdd(&g_work_counter, 1);
        if (work_idx >= num_active) return;

        int ray_idx = active_indices[work_idx];

        // Load ray data - coalesced because nearby threads got nearby indices
        // __ldg routes through read-only cache (texture cache path)
        float ox = rays.ox[ray_idx];
        float oy = rays.oy[ray_idx];
        float oz = rays.oz[ray_idx];
        float dx = rays.dx[ray_idx];
        float dy = rays.dy[ray_idx];
        float dz = rays.dz[ray_idx];
        float inv_dx = rays.inv_dx[ray_idx];
        float inv_dy = rays.inv_dy[ray_idx];
        float inv_dz = rays.inv_dz[ray_idx];
        float t_best = rays.t_max[ray_idx];
        int   best_tri = -1;
        float best_u = 0, best_v = 0;

        // BVH traversal stack (register/local memory)
        int stack[BVH_STACK_DEPTH];
        int stack_ptr = 0;
        stack[stack_ptr++] = 0;  // root node

        while (stack_ptr > 0) {
            int node_idx = stack[--stack_ptr];
            const BVHNode& node = __ldg(&bvh_nodes[node_idx]);

            if (node.left < 0) {
                // Leaf node: test triangles
                int tri_start = ~node.left;  // bitwise NOT to decode
                int tri_count = node.right;
                for (int i = 0; i < tri_count; i++) {
                    const Triangle& tri = __ldg(&triangles[tri_start + i]);
                    float t, u, v;
                    if (intersect_triangle(ox, oy, oz, dx, dy, dz,
                                           tri, t_best, t, u, v)) {
                        t_best = t;
                        best_tri = tri_start + i;
                        best_u = u;
                        best_v = v;
                    }
                }
            } else {
                // Internal node: test both children, push far child first
                // (so near child is processed next = depth-first)
                const BVHNode& left  = __ldg(&bvh_nodes[node.left]);
                const BVHNode& right = __ldg(&bvh_nodes[node.right]);

                bool hit_left  = intersect_aabb(ox, oy, oz, inv_dx, inv_dy, inv_dz,
                                                left, t_best);
                bool hit_right = intersect_aabb(ox, oy, oz, inv_dx, inv_dy, inv_dz,
                                                right, t_best);

                // Ordered traversal: push far child first so near is on top
                if (hit_left && hit_right) {
                    // Simple heuristic: compare AABB midpoint distance along ray
                    float mid_left  = (left.min_x + left.max_x)   * 0.5f * dx +
                                      (left.min_y + left.max_y)   * 0.5f * dy +
                                      (left.min_z + left.max_z)   * 0.5f * dz;
                    float mid_right = (right.min_x + right.max_x) * 0.5f * dx +
                                      (right.min_y + right.max_y) * 0.5f * dy +
                                      (right.min_z + right.max_z) * 0.5f * dz;
                    if (mid_left < mid_right) {
                        stack[stack_ptr++] = node.right;  // far
                        stack[stack_ptr++] = node.left;   // near (on top)
                    } else {
                        stack[stack_ptr++] = node.left;
                        stack[stack_ptr++] = node.right;
                    }
                } else if (hit_left) {
                    stack[stack_ptr++] = node.left;
                } else if (hit_right) {
                    stack[stack_ptr++] = node.right;
                }
            }
        }

        // Write results
        rays.t_max[ray_idx]     = t_best;
        rays.tri_index[ray_idx] = best_tri;
        rays.hit_u[ray_idx]     = best_u;
        rays.hit_v[ray_idx]     = best_v;

        // Tag with material ID so we can sort before shading
        if (best_tri >= 0) {
            rays.material_id[ray_idx] = __ldg(&triangles[best_tri].material_id);
        } else {
            rays.material_id[ray_idx] = 0xFFFFFFFF;  // miss sentinel
        }
    }
}

// ===========================================================================
// KERNEL 3: Compact active rays
// Stream compaction: remove dead rays, produce dense index list
// Uses warp-level ballot for efficiency
// ===========================================================================
__global__ void kernel_compact_active(
    const RayPoolSoA rays,
    int num_rays,
    int* __restrict__ active_indices,
    int* __restrict__ active_count)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_rays) return;

    bool is_active = (rays.active[idx] == 1) && (rays.tri_index[idx] >= 0);

    // Warp-level ballot: each thread knows which lanes are active
    unsigned int ballot = __ballot_sync(0xFFFFFFFF, is_active);
    int warp_count = __popc(ballot);

    // One thread per warp atomically reserves slots
    int warp_base = 0;
    int lane = threadIdx.x & 31;
    if (lane == 0 && warp_count > 0) {
        warp_base = atomicAdd(active_count, warp_count);
    }
    warp_base = __shfl_sync(0xFFFFFFFF, warp_base, 0);

    // Each active thread writes to its compacted position
    if (is_active) {
        // Count set bits below my lane in the ballot
        unsigned int mask_below = (1u << lane) - 1u;
        int my_offset = __popc(ballot & mask_below);
        active_indices[warp_base + my_offset] = idx;
    }
}

// ===========================================================================
// KERNEL 4: Sort rays by material ID
// After intersection, rays that hit the same material are grouped together
// so shading warps take uniform branches through the material shader.
//
// In practice: use CUB DeviceRadixSort on (material_id, ray_index) pairs.
// Shown here conceptually.
// ===========================================================================
// void sort_by_material(int* active_indices, uint32_t* material_keys, int count) {
//     cub::DeviceRadixSort::SortPairs(
//         d_temp, temp_bytes,
//         material_keys, material_keys_sorted,
//         active_indices, active_indices_sorted,
//         count);
// }

// ===========================================================================
// KERNEL 5: Material Shading
// Because rays are sorted by material, entire warps execute the same
// material branch - eliminating divergence in the shader.
// ===========================================================================
__global__ void kernel_shade_and_bounce(
    RayPoolSoA rays,
    const int* __restrict__ sorted_active_indices,
    int num_active,
    const Triangle* __restrict__ triangles,
    // Material data, textures, lights would go here
    RayPoolSoA shadow_rays,     // output: shadow rays to test
    int* shadow_ray_count,
    int max_bounces)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_active) return;

    int ray_idx = sorted_active_indices[idx];
    uint32_t mat_id = rays.material_id[ray_idx];

    if (mat_id == 0xFFFFFFFF) {
        // Miss - accumulate sky color, kill ray
        // (In practice: sample environment map)
        rays.active[ray_idx] = 0;
        return;
    }

    // Compute hit point: P = O + t * D
    float t = rays.t_max[ray_idx];
    float hit_x = rays.ox[ray_idx] + rays.dx[ray_idx] * t;
    float hit_y = rays.oy[ray_idx] + rays.dy[ray_idx] * t;
    float hit_z = rays.oz[ray_idx] + rays.dz[ray_idx] * t;

    int tri_idx = rays.tri_index[ray_idx];
    const Triangle& tri = __ldg(&triangles[tri_idx]);

    // -----------------------------------------------------------------------
    // Material dispatch - all threads in a warp have the same mat_id,
    // so this switch is NOT divergent within a warp
    // -----------------------------------------------------------------------
    float new_dx, new_dy, new_dz;
    float atten_r = 1.0f, atten_g = 1.0f, atten_b = 1.0f;

    switch (mat_id) {
        case 0: // Diffuse Lambertian
        {
            // Cosine-weighted hemisphere sample (simplified, needs RNG)
            // In practice: use curand or a low-discrepancy sequence
            new_dx = tri.nx;  // placeholder - would be random hemisphere dir
            new_dy = tri.ny;
            new_dz = tri.nz;
            atten_r = 0.8f;
            atten_g = 0.2f;
            atten_b = 0.2f;
            break;
        }
        case 1: // Perfect mirror
        {
            // R = D - 2(D·N)N
            float dn = rays.dx[ray_idx] * tri.nx +
                       rays.dy[ray_idx] * tri.ny +
                       rays.dz[ray_idx] * tri.nz;
            new_dx = rays.dx[ray_idx] - 2.0f * dn * tri.nx;
            new_dy = rays.dy[ray_idx] - 2.0f * dn * tri.ny;
            new_dz = rays.dz[ray_idx] - 2.0f * dn * tri.nz;
            atten_r = 0.95f;
            atten_g = 0.95f;
            atten_b = 0.95f;
            break;
        }
        default:
        {
            new_dx = tri.nx;
            new_dy = tri.ny;
            new_dz = tri.nz;
            atten_r = 0.5f;
            atten_g = 0.5f;
            atten_b = 0.5f;
            break;
        }
    }

    // Update throughput
    rays.throughput_r[ray_idx] *= atten_r;
    rays.throughput_g[ray_idx] *= atten_g;
    rays.throughput_b[ray_idx] *= atten_b;

    // -----------------------------------------------------------------------
    // Generate shadow ray (for direct lighting)
    // Shadow rays are batched separately and use an any-hit traversal
    // -----------------------------------------------------------------------
    int shadow_idx = atomicAdd(shadow_ray_count, 1);
    // Point light position (hardcoded for skeleton)
    float lx = 0.0f, ly = 10.0f, lz = 0.0f;
    float to_light_x = lx - hit_x;
    float to_light_y = ly - hit_y;
    float to_light_z = lz - hit_z;
    float light_dist = sqrtf(to_light_x * to_light_x +
                             to_light_y * to_light_y +
                             to_light_z * to_light_z);
    float inv_dist = 1.0f / light_dist;
    to_light_x *= inv_dist;
    to_light_y *= inv_dist;
    to_light_z *= inv_dist;

    shadow_rays.ox[shadow_idx] = hit_x + tri.nx * 1e-4f;  // bias off surface
    shadow_rays.oy[shadow_idx] = hit_y + tri.ny * 1e-4f;
    shadow_rays.oz[shadow_idx] = hit_z + tri.nz * 1e-4f;
    shadow_rays.dx[shadow_idx] = to_light_x;
    shadow_rays.dy[shadow_idx] = to_light_y;
    shadow_rays.dz[shadow_idx] = to_light_z;
    shadow_rays.inv_dx[shadow_idx] = 1.0f / to_light_x;
    shadow_rays.inv_dy[shadow_idx] = 1.0f / to_light_y;
    shadow_rays.inv_dz[shadow_idx] = 1.0f / to_light_z;
    shadow_rays.t_max[shadow_idx]      = light_dist - 1e-3f;
    shadow_rays.pixel_index[shadow_idx] = rays.pixel_index[ray_idx];

    // -----------------------------------------------------------------------
    // Set up bounce ray (overwrites current ray in-place)
    // -----------------------------------------------------------------------
    uint8_t bounce = rays.bounce[ray_idx];
    if (bounce >= max_bounces) {
        rays.active[ray_idx] = 0;
        return;
    }

    rays.ox[ray_idx] = hit_x + tri.nx * 1e-4f;
    rays.oy[ray_idx] = hit_y + tri.ny * 1e-4f;
    rays.oz[ray_idx] = hit_z + tri.nz * 1e-4f;
    rays.dx[ray_idx] = new_dx;
    rays.dy[ray_idx] = new_dy;
    rays.dz[ray_idx] = new_dz;
    rays.inv_dx[ray_idx] = 1.0f / new_dx;
    rays.inv_dy[ray_idx] = 1.0f / new_dy;
    rays.inv_dz[ray_idx] = 1.0f / new_dz;
    rays.t_max[ray_idx]     = FLT_MAX;
    rays.tri_index[ray_idx] = -1;
    rays.bounce[ray_idx]    = bounce + 1;
}

// ===========================================================================
// KERNEL 6: Shadow Ray Traversal (Any-Hit, Early Termination)
// Simpler than closest-hit: we only need to know if ANYTHING blocks the light
// ===========================================================================
__global__ void kernel_shadow_traverse(
    RayPoolSoA shadow_rays,
    int num_shadow,
    const BVHNode* __restrict__ bvh_nodes,
    const Triangle* __restrict__ triangles,
    uint8_t* __restrict__ shadow_results)  // 0 = lit, 1 = shadowed
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_shadow) return;

    float ox = shadow_rays.ox[idx];
    float oy = shadow_rays.oy[idx];
    float oz = shadow_rays.oz[idx];
    float dx = shadow_rays.dx[idx];
    float dy = shadow_rays.dy[idx];
    float dz = shadow_rays.dz[idx];
    float inv_dx = shadow_rays.inv_dx[idx];
    float inv_dy = shadow_rays.inv_dy[idx];
    float inv_dz = shadow_rays.inv_dz[idx];
    float t_max  = shadow_rays.t_max[idx];

    int stack[BVH_STACK_DEPTH];
    int stack_ptr = 0;
    stack[stack_ptr++] = 0;

    while (stack_ptr > 0) {
        int node_idx = stack[--stack_ptr];
        const BVHNode& node = __ldg(&bvh_nodes[node_idx]);

        if (node.left < 0) {
            int tri_start = ~node.left;
            int tri_count = node.right;
            for (int i = 0; i < tri_count; i++) {
                const Triangle& tri = __ldg(&triangles[tri_start + i]);
                float t, u, v;
                if (intersect_triangle(ox, oy, oz, dx, dy, dz,
                                       tri, t_max, t, u, v)) {
                    // ANY hit = shadowed, early exit
                    shadow_results[idx] = 1;
                    return;
                }
            }
        } else {
            if (intersect_aabb(ox, oy, oz, inv_dx, inv_dy, inv_dz,
                               __ldg(&bvh_nodes[node.left]), t_max))
                stack[stack_ptr++] = node.left;
            if (intersect_aabb(ox, oy, oz, inv_dx, inv_dy, inv_dz,
                               __ldg(&bvh_nodes[node.right]), t_max))
                stack[stack_ptr++] = node.right;
        }
    }

    shadow_results[idx] = 0;  // no occlusion found
}

// ===========================================================================
// KERNEL 7: Accumulate results to framebuffer
// ===========================================================================
__global__ void kernel_accumulate(
    const RayPoolSoA rays,
    const uint8_t* __restrict__ shadow_results,
    const RayPoolSoA shadow_rays,
    int num_shadow,
    float4* __restrict__ framebuffer,
    int sample_index)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_shadow) return;

    uint32_t pixel = shadow_rays.pixel_index[idx];
    float lit = shadow_results[idx] ? 0.0f : 1.0f;

    // Simple direct lighting contribution
    // (In practice: multiply by BRDF, NdotL, light intensity, etc.)
    float r = lit * 0.8f;
    float g = lit * 0.8f;
    float b = lit * 0.8f;

    // Progressive accumulation (running average)
    float4 prev = framebuffer[pixel];
    float weight = 1.0f / (float)(sample_index + 1);
    framebuffer[pixel] = make_float4(
        prev.x * (1.0f - weight) + r * weight,
        prev.y * (1.0f - weight) + g * weight,
        prev.z * (1.0f - weight) + b * weight,
        1.0f
    );
}

// ===========================================================================
// HOST: Main render loop - wavefront pipeline
// ===========================================================================
// void render_frame(/* scene, camera, framebuffer, ... */) {
//
//     // 1. Generate primary rays
//     kernel_generate_primary_rays<<<grid, BLOCK_SIZE>>>(
//         ray_pool, width, height, cam_origin, cam_fwd, cam_right, cam_up,
//         fov_scale, d_ray_count);
//
//     for (int bounce = 0; bounce < MAX_BOUNCES; bounce++) {
//
//         // 2. Compact active rays (stream compaction)
//         cudaMemset(d_active_count, 0, sizeof(int));
//         kernel_compact_active<<<grid, BLOCK_SIZE>>>(
//             ray_pool, num_rays, d_active_indices, d_active_count);
//
//         // 3. Reset persistent thread counter
//         cudaMemset(&g_work_counter, 0, sizeof(int));
//
//         // 4. BVH traversal with persistent threads
//         //    Launch exactly (num_SMs * blocks_per_SM) blocks
//         int num_blocks = num_SMs * 2;  // ~2 blocks per SM for occupancy
//         kernel_traverse_persistent<<<num_blocks, BLOCK_SIZE>>>(
//             ray_pool, d_active_indices, active_count,
//             d_bvh_nodes, d_triangles);
//
//         // 5. Sort active rays by material ID (CUB radix sort)
//         // sort_by_material(d_active_indices, d_material_keys, active_count);
//
//         // 6. Shade hits, generate bounce + shadow rays
//         cudaMemset(d_shadow_count, 0, sizeof(int));
//         kernel_shade_and_bounce<<<grid, BLOCK_SIZE>>>(
//             ray_pool, d_sorted_indices, active_count,
//             d_triangles,
//             shadow_pool, d_shadow_count, MAX_BOUNCES);
//
//         // 7. Shadow ray traversal (any-hit)
//         kernel_shadow_traverse<<<grid, BLOCK_SIZE>>>(
//             shadow_pool, shadow_count,
//             d_bvh_nodes, d_triangles, d_shadow_results);
//
//         // 8. Accumulate lighting
//         kernel_accumulate<<<grid, BLOCK_SIZE>>>(
//             ray_pool, d_shadow_results, shadow_pool, shadow_count,
//             d_framebuffer, sample_index);
//     }
// }
