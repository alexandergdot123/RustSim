
typedef struct {
    uint16_t v0[3]; // 6 bytes per vertex, 3 vertices per triangle
    uint16_t pad;
    uint32_t tri_index;
} Triangle;                       

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
    uint16_t* tri_start;           // 2 bytes - pointer to Triangle[tri_count]
    uint8_t  tri_count;            // 1 byte
    uint8_t  is_right;             // 1 byte
    uint16_t core_owner;        // 2 bytes - the core that is currently responsible for this node (0xFFFF if no owner)
    uint32_t queue_low_bit_addr; // 4 bytes - the address of the low bits of the ray queue for this node, used for sending rays to the owning core
    uint32_t queue_high_bit_addr; // 4 bytes - the address of the high bits of the ray queue for this node, used for sending rays to the owning core
    uint32_t node_id;
} AABB_Node;

typedef struct {
    float x, y, z;        // 12 bytes - position
    float r, g, b;        // 12 bytes - color
    float luminance;      // 4 bytes  - emissive
} Vertex;                 // 28 bytes total

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
    uint8_t light_id; //msb = is_shadow
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

//ideally, there are 22 levels of nodes. 
//branch cores own 9 local levels of nodes = 512 nodes = 24.5KB.
//Additionally, they cache another 6 levels of nodes above themselves to allow leaf-leaf forwarding.
//6 levels = 64 nodes = 3KB
//Branch cores own 7 levels of nodes above their owned leaf cores's roots. 
//7 levels = 128 nodes = 6KB
//Additionally, they will own the final tree above their root so that they can get 9 + 7 + 6 = 22 fully live nodes
//All of these top 6 levels will be shared between all branch cores
//6 levels = 64 nodes = 3KB

const core_ray_forward = 5;
const branch_trade_rays = 6;
const reject_ray = 7;
const wrong_core = 8;


//start_ray_traversal
node = self.root->left_child; // Start with the left child
//start_searching
yield();
uint32_t left_bitfield_check = ray->check_left & (1 << ray->ray_depth) | node->left_child == 0;
uint32_t right_bitfield_check = ray->check_right & (1 << ray->ray_depth) | node->right_child == 0;
if(left_bitfield_check != 0 && right_bitfield_check != 0){
    // We've already visited both subtrees at this depth, so we can backtrack
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
                    queue_address_low = queue_address_low + 536; //skip the head and count, which are the first 8 bytes of the queue structure
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
                    queue_address_low = node->queue_low_bit_addr;
                    queue_address_low += 20;
                    atomic_add_dram(queue_address_low, 1);
                    //lock_loop
                    int lock_positive = load_dram_word(queue_address_low);
                    if(lock_positive < 0) {
                        goto lock_loop;
                    }
                    int old_queue_address = queue_address_low;
                    queue_address_low += 4;
                    int num_cores = load_dram_word(queue_address_low);
                    int hash = (self.coreid >> 6) ^ self.coreid;
                    int core_hash = hash % num_cores;
                    core_hash = core_hash << 2;
                    queue_address_low += 4;
                    queue_address_low += core_hash;
                    int cur_core = load_dram_half(queue_address_low);
                    node->core_owner = cur_core;
                    atomic_add(old_queue_address, -1);
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
        // No intersection with the AABB, so we can backtrack
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
goto start_searching
// ray_done
if(ray->active_ray == 1){
    goto start_ray_traversal;
}
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
//there needs to be a way to create new rays if there are none to be pulled

goto ray_done //this should only be important once all computation has basically finished

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

float fp_div(float a, float b) {
    // extract top 8 bits of b's mantissa
    uint32_t b_bits = (uint32_t)b;
    uint32_t index = (b_bits >> 15) & 0xFF;
    
    // load seed from SRAM table
    float x = load_byte_unsigned(reciprocal_table + index);
    // scale x appropriately based on b's exponent
    // (need to reconstruct proper float from seed + exponent)
    
    // Newton-Raphson iteration 1: x = x * (2.0 - b * x)
    set_accumulator(2.0);
    float bx = b * x;
    float correction = 2.0 - bx;
    x = x * correction;
    
    // Newton-Raphson iteration 2: x = x * (2.0 - b * x)
    bx = b * x;
    correction = 2.0 - bx;
    x = x * correction;
    
    // x ≈ 1/b, so a/b ≈ a * x
    return a * x;
}
void Triangle_Intersect(Triangle* tri, Ray* ray, Vertex* vertices) {
    Vertex* v0 = &vertices[tri->v0[0]];
    Vertex* v1 = &vertices[tri->v0[1]];
    Vertex* v2 = &vertices[tri->v0[2]];

    float e1x = v1->x - v0->x;
    float e1y = v1->y - v0->y;
    float e1z = v1->z - v0->z;

    float e2x = v2->x - v0->x;
    float e2y = v2->y - v0->y;
    float e2z = v2->z - v0->z;

    float px = ray->dy * e2z - ray->dz * e2y;
    float py = ray->dz * e2x - ray->dx * e2z;
    float pz = ray->dx * e2y - ray->dy * e2x;

    set_accumulator(0.0);
    fmac(e1x, px);
    fmac(e1y, py);
    fmac(e1z, pz);
    float det = store_accumulator();

    if (det == 0.0) return;

    float tx = ray->ox - v0->x;
    float ty = ray->oy - v0->y;
    float tz = ray->oz - v0->z;

    set_accumulator(0.0);
    fmac(tx, px);
    fmac(ty, py);
    fmac(tz, pz);
    float u_unscaled = store_accumulator();

    if (det > 0.0) {
        if (u_unscaled < 0.0 || u_unscaled > det) return;
    } else {
        if (u_unscaled > 0.0 || u_unscaled < det) return;
    }

    float qx = ty * e1z - tz * e1y;
    float qy = tz * e1x - tx * e1z;
    float qz = tx * e1y - ty * e1x;

    set_accumulator(0.0);
    fmac(ray->dx, qx);
    fmac(ray->dy, qy);
    fmac(ray->dz, qz);
    float v_unscaled = store_accumulator();

    float uv_sum = u_unscaled + v_unscaled;
    if (det > 0.0) {
        if (v_unscaled < 0.0 || uv_sum > det) return;
    } else {
        if (v_unscaled > 0.0 || uv_sum < det) return;
    }

    set_accumulator(0.0);
    fmac(e2x, qx);
    fmac(e2y, qy);
    fmac(e2z, qz);
    float t_unscaled = store_accumulator();

    float tmin_scaled = ray->t_min * det;
    float tmax_scaled = ray->t_max * det;
    if (det > 0.0) {
        if (t_unscaled < tmin_scaled || t_unscaled > tmax_scaled) return;
    } else {
        if (t_unscaled > tmin_scaled || t_unscaled < tmax_scaled) return;
    }

    float t = t_unscaled * fp_div(1.0, det);
    ray->t_max = t;
    ray->tri_index = tri->tri_index;
}
//need a changed_parent interrupt
//need a new_ray interrupt
//need a "convert to a branch core" interrupt
//need a "convert to a different leaf core" interrupt