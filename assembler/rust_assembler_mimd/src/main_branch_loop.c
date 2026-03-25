// always do pre-order traversal
typedef struct { //48 Bytes
    float x_min;
    float x_max;
    float y_min;
    float y_max;
    float z_min;
    float z_max;
    uint16_t* left_child;          // 2 bytes - 0 if leaf
    uint16_t* right_child;         // 2 bytes - 0 if leaf
    uint16_t* parent;              // 2 bytes
    uint8_t  is_right;             // 1 byte
    uint8_t  pad[3];
    uint16_t core_owner;        // 2 bytes - the core that is currently responsible for this node (0xFFFF if no owner)
    uint32_t queue_low_bit_addr; // 4 bytes - the address of the low bits of the ray queue for this node, used for sending rays to the owning core
    uint32_t queue_high_bit_addr; // 4 bytes - the address of the high bits of the ray queue for this node, used for sending rays to the owning core
    uint32_t node_id;
} AABB_Node;

typedef struct { //64 Bytes, 16 packets
    float ox, oy, oz;      // 12 bytes - origin
    float dx, dy, dz;      // 12 bytes - direction
    float inv_dx, inv_dy, inv_dz; // 12 bytes - precomputed 1/direction
    float t_min, t_max;    // 8 bytes  - valid interval
    uint32_t check_left;   // used for backtracking
    uint32_t check_right;  // used for backtracking
    uint16_t pix_x;
    uint16_t pix_y;
    uint32_t tri_index;     // index of the triangle hit, 0xFFFF_FFFF if no hit
    uint8_t bounce_count;
    uint8_t ray_depth;    
    uint8_t light_id; //0 for not a shadow, 1, 2, 3 for lights
    uint8_t active_ray;
} Ray;                    //64 Bytes, 16 packets



typedef struct {
    uint32_t data_mailbox[16]; //deep
    uint32_t must_respond[16];
    uint32_t may_respond[16];
    uint32_t interrupt_mailbox[16];
} mailbox_system;


typedef struct { //16924 Bytes
    uint32_t head_relative; //relative to the start of the queue in DRAM
    uint32_t tail_relative; //relative to the start of the queue in DRAM
    uint32_t count;
    uint32_t next_ticket;//atomically incremented
    uint32_t now_serving; //spin on when this value equals your ticket, then increment when done
    uint32_t lock;
    uint32_t core_owner_count;
    uint16_t core_slots[256];
    struct Ray[256] rays; //256 * 64 bytes for the rays
} ray_queue_dram;

typedef struct { //32 Bytes, 8 packets
    float ox, oy, oz;      // 12 bytes - origin
    float dx, dy, dz;      // 12 bytes - direction
    uint16_t pix_x; //2 bytes
    uint16_t pix_y; //2 bytes
    uint8_t bounce_count;
    uint8_t light_id; //msb = is_shadow
    uint8_t padding;
    uint8_t open_slot;
} RaySpawn;                    //32 Bytes, 8 packets

typedef struct {
    uint32_t head; //bytes relative to start of RaySpawns
    uint32_t tail; //bytes relative to start of RaySpawns
    uint32_t count;
    struct RaySpawn newRays[262144]; //num_cores (8192) * num_threads (16) * max_rays_per_pix (16) / num_stacks (1 per stack) (8)
} SpawnedRayPool; //67MB across DRAM total for each stack

typedef struct { //64*32 Bytes = 2052 total bytes
    uint16_t head;
    uint16_t tail;
    struct RaySpawn rays[64];
} RaySpawnArray;

// Single ray result after traversal + shading
// Stored in fp16, accumulated in fp32 during final pass
typedef struct {  // 16 bytes
    float r, g, b;
    union {
        float len_sq;
        uint32_t tri_index;
    };
} RayResult;

typedef struct {  // 256 bytes
    // results[depth * 4 + 0] = bounce
    // results[depth * 4 + 1] = shadow light 0
    // results[depth * 4 + 2] = shadow light 1
    // results[depth * 4 + 3] = shadow light 2
    RayResult results[16];
} PixelResults;

typedef struct {
    float red, green, blue;
    float roughness, metallic;
    float x_norm, y_norm, z_norm;
} Triangle;                       
// Full framebuffer of ray results for 4K
// Address: base + (y * 3840 + x) * 256
// Total: 3840 * 2160 * 256 = ~2.03 GB
typedef struct {
    PixelResults pixels[1920 * 1080];  // 2,073,600 pixels
} FrameResults;

// Final pixel color output
typedef struct {  // 4 bytes
    uint8_t r, g, b, a;
} Pixel;

// Final framebuffer
typedef struct {
    Pixel pixels[1920 * 1080];  // ~8 MB
} Framebuffer;

typedef struct {
    uint32_t index;
    uint16_t count;
    uint16_t is_valid;
} tile_slot;

typedef struct {
    uint32_t head;
    uint32_t tail;
    struct tile_slot slots[1 << 15]; // slightly larger than 1920*1080/64 + 1. Also initializes to full.
} tile_queue;

typedef struct {
    float r, g, b; 
    float x, y, z;
} light;

typedef struct {
    light[3];
} light_array;

const core_ray_forward = 5;
const branch_trade_rays = 6;
const reject_ray = 7;
const wrong_core = 8;






//start_ray_traversal
node = self.root->left_child; // Start with the left child
//start_searching
yield();
if (ray->check_left & 1 != 0 && ray->check_right & 1 != 0){
    jmp complete_ray;
}
uint32_t left_bitfield_check = ray->check_left & (1 << ray->ray_depth) | node->left_child == 0;
uint32_t right_bitfield_check = ray->check_right & (1 << ray->ray_depth) | node->right_child == 0;
if(left_bitfield_check != 0 && right_bitfield_check != 0){
    // We've already visited both subtrees at this depth, so we can backtrack
    if(ray->ray_depth == 0){
        //we have completed tracing the ray. now we need to calculate any new spawns
        //say we are creating two additional rays
        struct RayArray* rayqueue;
        int head_pos = rayqueue->head;
        int old_tail = atomic_add(&rayqueue->head, 64);
        int count = old_tail - head_pos;
        if(count > 63){
            atomic_add(&rayqueue->head, -64);
            goto core_dram_queue;
        }
        //ray math goes here, transfering from ray -> old_tail, making sure to mark last byte as 1

        //then complete
        goto ray_done;
        //core_dram_queue
        int queue_address_low = self.stack_ray_pool_address_low;
        int queue_address_high = self.stack_ray_pool_address_high;
        set_address_bits(queue_address_high);
        int cur_ray_count = load_dram_word(queue_address_low + 8); //check if there are any rays in the queue
        //queue_overfill
        if(cur_ray_count < 8330000){ //real number is 8338608
            goto queue_overfill;
        }
        int cur_ray_count_check = atomic_add_dram(queue_address_low + 8, 1); //increment the count of rays in the queue
        int tail = atomic_add_dram(queue_address_low, 64); // advance tail atomically
        queue_address_low = queue_address_low + 12; //skip the head, tail, and count
        tail = tail & 0x1FFFFFFF;//wrap tail, can be smaller if potential ray count is smaller
        queue_address_low = queue_address_low + tail;
        //wait_for_write
        int ready = load_dram_byte(queue_address_low + 63); //the last 4 bytes of the ray slot are used to indicate if the ray is ready to be read
        if(ready == 1){
            goto wait_for_write;
        }
        //do ray calculations, the push onto dram one by one
        write_dram_byte(queue_address_low - 1, 1); //mark the ray as live
        goto ray_done;


    }
    uint32_t bitfield = *(ray.check_left + node->is_right * 4);
    uint32_t or_value = 1 << (ray->ray_depth - 1);
    bitfield |= or_value;
    *(ray.check_left + node->is_right * 4) = bitfield;
    ray->ray_depth--;
    if(node->parent == 0){
        goto finish;    
    }
    node = node->parent;
}
else if(left_bitfield_check == 0 && right_bitfield_check == 0){
    int hit = AABB_Intersect(node, ray);
    //First, mark that we've performed an intersection on the node
    //every time we do an intersection, we mark the current depth with a 1 on the correct side
    //every time we move up the tree, we mark all nodes below that level with 0
    if(hit){
        if(node->tri_count == 0){
            ray->ray_depth++;
            if(node->core_owner != 0xFFFF){
                //finish
                ray_send_pending[self.thread_id] = 1;
                send_packet(self.thread_id, node->core_owner, 48);
                send_packet(node->node_id, node->core_owner, 48);
                //forward_branch_ray_loop
                int got_response = non_blocking_receive(16 + self.thread_id);
                if(got_response == 1){
                    int branch_core_response = blocking_receive(16 + self.thread_id);
                    if(branch_core_response >> 24 == reject_ray || branch_core_response >> 24 == wrong_core){
                        int queue_address_low = node->queue_low_bit_addr;
                        int queue_address_high = node->queue_high_bit_addr;
                        set_address_bits(queue_address_high);
                        node->core_owner = load_dram_word(queue_address_low + 12);

                        //ensure_space_in_queue:
                        int cur_ray_count = load_dram_word(queue_address_low + 8);
                        if(cur_ray_count >= 200){ //if the queue is full, wait until there is space
                            goto ensure_space_in_queue;
                        }

                        int tail = atomic_add_dram(queue_address_low + 4, 64); //add a ray to the queue
                        atomic_add_dram(queue_address_low + 8, 1); //increment the count of rays in the queue
                        tail = tail & 0x00003FFF; //64 bytes, 256 slots in queue
                        queue_address_low = queue_address_low + 16; //skip the head and count, which are the first 8 bytes of the queue structure
                        queue_address_low = queue_address_low + tail;
                        int ray_index = ray;
                        for(int i = 0; i < 16; i += 1){
                            store_dram_word(queue_address_low, ray_index);
                            int queue_address_low = queue_address_low + 4;
                            ray_index = ray_index + 4;
                        }
                        ray_send_pending[self.thread_id] = 0;
                        atomic_add(&total_rays_traced, 1);
                        ray->active_ray = 0; //the ray has been accepted, so we can mark it as inactive
                        goto ray_done;
                    }
                    for(int i = 0; i < 16; i++){
                        send_packet(((uint32_t*)ray)[i], node->core_owner, self.thread_id);
                    }
                    if(branch_core_response >> 24 == trade_rays){
                        for(int i = 0; i < 16; i++){
                            ((uint32_t*)ray)[i] = blocking_receive(self.thread_id);
                        }
                    }
                    else{
                        ray->active_ray = 0; //the ray has been accepted, so we can mark it as inactive
                    }
                    ray_send_pending[self.thread_id] = 0;
                    atomic_add(&total_rays_traced, 1);
                    goto ray_done;
                }
                yield(); // I am going to need to be extremely careful about designing my interrupts
                goto forward_branch_ray_loop;
            }
            else{
                node = node->left_child; // Traverse left child first
            }
        }
        else{
            // We've hit a leaf node, so we need to check for intersections with the triangles
            uint32_t bitfield = *(ray.check_left + node->is_right * 4);
            uint32_t or_value = 1 << (ray->ray_depth - 1);
            bitfield |= or_value;
            *(ray.check_left + node->is_right * 4) = bitfield;
            uint16_t tri_index = node->tri_start;
            for(int i = 0; i < node->tri_count; i++){
                Triangle_Intersect(tri_index, ray);
                tri_index = tri_index + 12;
            }
            // After checking all triangles, we can backtrack
            ray->ray_depth--;
            node = node->parent;
        }
    }
    else{
        // No intersection with the AABB, so we can backtrack
        if(ray->ray_depth == 0){
            ray->check_right = 0xFFFFFFFF;
            ray->check_left = 0xFFFFFFFF;
            goto start_searching;
        }
        uint32_t right_bitfield = *(ray.check_left + node->is_right * 4);
        uint32_t or_value = 1 << (ray->ray_depth - 1);
        right_bitfield |= or_value;
        *(ray.check_left + node->is_right * 4) = right_bitfield;
        ray->ray_depth--;
        node = node->parent;
    }
}
else{
    // We haven't visited the left subtree yet, so we should traverse it
    uint32_t zero_out_subtree = ~(0xFFFFFFFF << ray->ray_depth + 1);
    ray->check_left &= zero_out_subtree; // Clear all bits below the current depth
    ray->check_right &= zero_out_subtree; // Clear all bits below the current depth
    node = *(node.left_child + ((left_bitfield_check != 0) * 2));
    ray->ray_depth++;
}
goto start_searching;
// ray_done
if(ray->active_ray == 1){
    goto start_ray_traversal;
}
while(1){
    yield();
    int queue_address_low = self.ray_queue_address_low;
    int queue_address_high = self.ray_queue_address_high;
    set_address_bits(queue_address_high);
    int cur_ray_count = load_dram_word(queue_address_low + 8); //check if there are any rays in the queue
    if(cur_ray_count > 0){
        int cur_ray_count_check = atomic_add_dram(queue_address_low + 8, -1); //decrement the count of rays in the queue
        if(cur_ray_count_check == 0){ //if another core took the last ray, we should check again
            atomic_add_dram(queue_address_low + 8, 1); //undo the decrement
            goto ray_done;
        }
        int head = atomic_add_dram(queue_address_low, 64); // advance head atomically first
        queue_address_low = queue_address_low + 16; //skip the head and count, which are the first 8 bytes of the queue structure
        head = head & 0x00003FFF;
        queue_address_low = queue_address_low + head;
        //wait_for_write
        int ready = load_dram_byte(queue_address_low + 63); //the last 4 bytes of the ray slot are used to indicate if the ray is ready to be read
        if(ready == 0){
            goto wait_for_write;
        }
        int ray_index = ray;
        for(int i = 0; i < 16; i++){
            *(ray_index) = load_dram_word(queue_address_low);
            queue_address_low = queue_address_low + 4;
            ray_index = ray_index + 4;
        }
        write_dram_byte(queue_address_low - 1, 0); //mark the ray as consumed
        queue_address_low = self.ray_queue_address_low;
        ray->active_ray = 1; //mark the ray as active
        goto start_ray_traversal;
    }
    yield();
    yield();
    yield();
    yield();
}

int AABB_Intersect(AABB_Node* node, Ray* ray) {
    float tx1 = (node->x_min - ray->ox) * ray->inv_dx;
    float tx2 = (node->x_max - ray->ox) * ray->inv_dx;

    float tmin = min(tx1, tx2);
    float tmax = max(tx1, tx2);

    tmin = max(tmin, ray->t_min);
    tmax = min(tmax, ray->t_max);

    if (tmin > tmax || tmax <= 0.0) return 0;

    float ty1 = (node->y_min - ray->oy) * ray->inv_dy;
    float ty2 = (node->y_max - ray->oy) * ray->inv_dy;

    tmin = max(tmin, min(ty1, ty2));
    tmax = min(tmax, max(ty1, ty2));

    if (tmin > tmax || tmax <= 0.0) return 0;

    float tz1 = (node->z_min - ray->oz) * ray->inv_dz;
    float tz2 = (node->z_max - ray->oz) * ray->inv_dz;

    tmin = max(tmin, min(tz1, tz2));
    tmax = min(tmax, max(tz1, tz2));

    return (tmin <= tmax) & (0.0 < tmax);
}

//complete_ray
uint32_t result_addr_high = self.ray_result_addr_high;
uint32_t result_addr_low = self.ray_result_addr_low;
uint32_t pix_index = ray->pix_y;
pix_index *= 1440;
pix_index += ray->pix_x;
pix_index <<= 8;
result_addr_low += pix_index;
uint32_t bounce = ray->bounce_count;
bounce <<= 6;
result_addr_low += bounce;


if(ray->light_id != 0) {
    goto shadow_ray;
}

uint32_t tri_addr_high = self.triangle_address_high;
set_address_bits(tri_addr_high);
uint32_t tri_addr_low = self.triangle_address_low;
uint32_t tri_offset = ray->tri_index << 5;  // * 32 bytes
tri_addr_low += tri_offset;
float tri_red = load_dram_word(tri_addr_low);
float tri_green = load_dram_word(tri_addr_low + 4);
float tri_blue = load_dram_word(tri_addr_low + 8);
float norm_x = load_dram_word(tri_addr_low + 20);
float norm_y = load_dram_word(tri_addr_low + 24);
float norm_z = load_dram_word(tri_addr_low + 28);
set_address_bits(result_addr_high);
store_dram_word(result_addr_low,      tri_red);
store_dram_word(result_addr_low + 4,  tri_green);
store_dram_word(result_addr_low + 8,  tri_blue);
//tri_red, _green, _blue unnecessary here now
float hit_x = ray->ox + ray->t_max * ray->dx;
float hit_y = ray->oy + ray->t_max * ray->dy;
float hit_z = ray->oz + ray->t_max * ray->dz;


// light 0 → slot 1
uint16_t light = self.light_array;
float lx = *(light) - hit_x;
float ly = *(light + 4) - hit_y;
float lz = *(light + 8) - hit_z;
float ndotl = norm_x * lx + norm_y * ly + norm_z * lz;
ndotl = max(ndotl, 0.0);
store_dram_word(result_addr_low + 28, ndotl);

// light 1 → slot 2
lx = self.lights[1].x - hit_x;
ly = self.lights[1].y - hit_y;
lz = self.lights[1].z - hit_z;
ndotl = norm_x * lx + norm_y * ly + norm_z * lz;
ndotl = max(ndotl, 0.0);
store_dram_word(result_addr_low + 44, ndotl);

// light 2 → slot 3
lx = self.lights[2].x - hit_x;
ly = self.lights[2].y - hit_y;
lz = self.lights[2].z - hit_z;
ndotl = norm_x * lx + norm_y * ly + norm_z * lz;
ndotl = max(ndotl, 0.0);
store_dram_word(result_addr_low + 60, ndotl);


//need to spawn bounce rays here:







goto ray_done;



// shadow_ray
set_address_bits(result_addr_high);
uint32_t shadow = ray->light_id;
result_addr_low += shadow << 4;

if(ray->tri_index != 0xFFFFFFFF){
    uint32_t one = 0x3F800000;
    store_dram_word(one, result_addr_low + 12);
    ray->active_ray = 0;
    goto ray_done;
}

shadow -= 1;
uint16_t light = self.light_array;
shadow *= 24;
light += shadow;
float light_r = *(light);
float light_g = *(light + 4);
float light_b = *(light + 8);
float ndotl = load_dram_word(result_addr_low + 12);
light_r *= ndotl;
light_g *= ndotl;
light_b *= ndotl;
store_dram_word(light_r, result_addr_low);
store_dram_word(light_g, result_addr_low + 4);
store_dram_word(light_b, result_addr_low + 8);
float light_x = *(light + 12);
float light_y = *(light + 16);
float light_z = *(light + 20);
light_x -= ray->ox;
light_y -= ray->oy;
light_z -= ray->oz;
light_x *= light_x;
light_y *= light_y;
light_z *= light_z;
light_x += light_y;
light_x += light_z;
store_dram_word(light_x, result_addr_low + 12);
ray->active_ray = 0;
goto ray_done;


