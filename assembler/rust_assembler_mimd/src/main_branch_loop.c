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
    float t_max;    // 4 bytes  - valid interval
    uint32_t leaf_node_starting_point;
    uint32_t check_left;   // used for backtracking
    uint32_t check_right;  // used for backtracking
    uint16_t pix_x;
    uint16_t pix_y;
    uint32_t tri_index;     // index of the triangle hit, 0xFFFF_FFFF if no hit
    uint8_t bounce_count;
    uint8_t light_id; //0 for not a shadow, 1, 2, 3 for lights
    uint8_t ray_depth;    
    uint8_t active_ray;
} Ray;                    //64 Bytes, 16 packets

typedef struct {
    uint16_t leaf_core_ptrs[128]; //indexed by node_id, gives the starting address of the leaf core's data for that node. 0 if not a leaf or not owned
    uint16_t root_node_ptr; //the starting address of the root node's data, used for traversal
} leaf_core_lookup_table;


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

typedef struct { //16924 Bytes
    uint32_t head_relative; //relative to the start of the queue
    uint32_t tail_relative; //relative to the start of the queue
    uint32_t count;
    struct Ray[16] rays; //16 * 64 bytes for the rays
} ray_queue_sram;



typedef struct { //32 Bytes, 8 packets
    float ox, oy, oz;      // 12 bytes - origin
    float dx, dy, dz;      // 12 bytes - direction
    uint16_t pix_x; //2 bytes
    uint16_t pix_y; //2 bytes
    uint8_t bounce_count;
    uint8_t light_id; 
    uint8_t padding;
    uint8_t open_slot;
} RaySpawn;                    //32 Bytes, 8 packets

typedef struct {
    uint32_t head; //bytes relative to start of RaySpawns
    uint32_t tail; //bytes relative to start of RaySpawns
    uint32_t count;
    struct RaySpawn newRays[262144]; //num_cores (8192) * num_threads (16) * max_rays_per_pix (16) / num_stacks (1 per stack) (8)
} SpawnedRayPool; //67MB across DRAM total for each stack

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
    PixelResults pixels[2560 * 1440];
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
    uint16_t index;
    uint8_t count;
    uint8_t is_valid;
} tile_slot;

typedef struct {
    uint32_t head;
    uint32_t tail;
    uint32_t count;
    struct tile_slot slots[1 << 14]; // slightly larger than 1920*1080/64 + 1. Also initializes to full.
} tile_queue;

typedef struct {
    float r, g, b; 
    float x, y, z;
} light;

typedef struct {
    light[3];
} light_array;

typedef struct {
    uint32_t relative_byte_cnt;
    uint32_t random_vals[65536];
} random_table;

typedef struct {
    uint32_t count;
    uint16_t is_active;
    uint8_t tile_x_index;
    uint8_t tile_y_index;
    uint8_t cur_ray_spawned_from_tile[16];
    uint32_t rays_spawned_from_tile;
    uint32_t rays_forwarded_out_from_tile;
} tile_data_sram;



const ray_ack = 5;
const reject_ray = 7;
const wrong_core = 8;
const correct_core = 9;





//start_ray_traversal:
yield();
if (ray->check_left & 1 != 0 && ray->check_right & 1 != 0){
    goto complete_ray;
}
uint32_t left_bitfield_check = ray->check_left & (1 << ray->ray_depth) | node->left_child == 0;
uint32_t right_bitfield_check = ray->check_right & (1 << ray->ray_depth) | node->right_child == 0;
if(left_bitfield_check != 0 && right_bitfield_check != 0){
    // We've already visited both subtrees at this depth, so we can backtrack
    if(ray->ray_depth == 0){
        //we have completed tracing the ray. now we need to calculate any new spawns
        //say we are creating two additional rays
        goto complete_ray;
    }
    uint32_t bitfield = *(ray.check_left + node->is_right * 4);
    uint32_t or_value = 1 << (ray->ray_depth - 1);
    bitfield |= or_value;
    *(ray.check_left + node->is_right * 4) = bitfield;
    ray->ray_depth--;
    if(node->parent == 0){
        goto complete_ray;    
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
                uint16_t ray_send_pending_addr = self.ray_send_pending_addr;
                atomic_add(ray_send_pending_addr, 1);
                /*
                send_ray_to_core(ray, dest):
                    rays_incoming = 0
                    send request to dest's interrupt mailbox
                    sent = false
                    loop:
                        if nb_recv(data_mailbox) >= 1:
                            for i in 0..16:
                                blocking_recv(data_mailbox)
                            enqueue(new_ray) //this enqueue MUST allow for 
                            rays_incoming--
                        if nb_recv(shallow_mailbox) >= 1:
                            msg = recv(shallow_mailbox)
                            sent = true
                            if msg == ACK:
                                for i in 0..16:
                                    send(dest, ray[i])
                            if msg == REJECT:
                                push to dram queue
                        if nb_recv(interrupt_mailbox) >= 1:
                            req = recv(interrupt_mailbox)
                            if space_in_queue - rays_incoming > 0:
                                send_ack(req.src)
                                rays_incoming++
                            else:
                                send_reject(req.src)
                        if sent and rays_incoming == 0:
                            break
                */

                uint32_t is_ray_spawned_from_tile = *(self.tile_data_sram->cur_ray_spawned_from_tile + self.thread_id);
                if(is_ray_spawned_from_tile != 0){
                    uint32_t depth = ray->ray_depth;
                    if(depth < 12){
                        uint32_t tile_sram_address = &(self.tile_data_sram->rays_forwarded_out_from_tile);
                        atomic_add(tile_sram_address, 1);
                    }
                }
                uint32_t slot = 0xFFFFFFFF;
                uint32_t sent = 0;
                uint32_t request_word = (node->node_id << 17) | self.thread_id;
                send_packet(request_word, node->core_owner, INTERRUPT_MAILBOX);

                //send_ray_loop:
                uint32_t msg_available = nb_recv(self.thread_id + 16);
                if(msg_available == 1){
                    uint32_t msg = blocking_receive(self.thread_id + 16);
                    uint32_t header = msg >> 24;

                    if(header == ack_ray){
                        for(int i = 0; i < 16; i++){
                            send_packet(((uint32_t*)ray)[i], node->core_owner, msg & 0xF);
                        }
                        ray->active_ray = 0;
                        sent = 1;
                    }
                    else{
                        int queue_address_high = node->queue_high_bit_addr;
                        set_address_bits(queue_address_high);

                        // typedef struct { //16924 Bytes
                        //     uint32_t head_relative; //relative to the start of the queue in DRAM
                        //     uint32_t tail_relative; //relative to the start of the queue in DRAM
                        //     uint32_t count;
                        //     uint32_t next_ticket;//atomically incremented
                        //     uint32_t now_serving; //spin on when this value equals your ticket, then increment when done
                        //     uint32_t lock;
                        //     uint32_t core_owner_count;
                        //     uint16_t core_slots[256];
                        //     struct Ray[256] rays; //256 * 64 bytes for the rays
                        // } ray_queue_dram;
                        int queue_address_low = node->queue_low_bit_addr;
                        queue_address_low += 20;
                        //ensure_no_writers:
                        int is_there_a_writer = atomic_add_dram(queue_address_low, 1); 
                        if(is_there_a_writer < 0){
                            atomic_add_dram(queue_address_low, -1);
                            goto ensure_no_writers;
                        }
                        //ensure_space_in_queue:
                        int cur_ray_count = load_dram_word(queue_address_low - 12);
                        if(cur_ray_count > 255){
                            goto ensure_space_in_queue;
                        }
                        int cur_num_cores_serving_queue = load_dram_word(queue_address_low + 4);
                        if(cur_num_cores_serving_queue > 200){
                            //need to throw the node_id into a queue for someone to pick up the geometry.
                            //TODO
                        }
                        queue_address_low -= 16;
                        int tail = atomic_add_dram(queue_address_low, 64);
                        tail &= 0x00003FFF;
                        int write_addr = queue_address_low + 536;
                        write_addr += tail;
                        //wait_for_slot_to_open:
                        int cur_ray_count = load_dram_byte(write_addr + 63);
                        if(cur_ray_count != 0){
                            goto wait_for_slot_to_open;
                        }
                        uint32_t ray_index = ray;
                        for(int i = 0; i < 16; i++){
                            uint32_t ray_word = load_word(ray_index);
                            store_dram_word(write_addr, ray_word);
                            write_addr = write_addr + 4;
                            ray_index = ray_index + 4;
                        }
                        queue_address_low = node->queue_low_bit_addr;
                        if(header == wrong_core){
                            uint32_t core_owner_count = load_dram_word(queue_address_low + 24);
                            if(core_owner_count == 0){
                                node->core_owner = 0xFFFFFFFE;
                            }
                            else{
                                uint16_t idx = ((self.core_id >> 7) ^ (self.core_id & 0x7F)) % core_owner_count;
                                idx <<= 1;
                                queue_address_low += idx;
                                uint32_t core_to_cache = load_dram_word(queue_address_low + 28);
                                node->core_owner = core_to_cache;
                            }
                        }
                        queue_address_low += 20;
                        atomic_add_dram(queue_address_low, -1);
                        ray->active_ray = 0;
                        sent = 1;
                    }
                }
                uint32_t data_available = nb_recv(self.thread_id);
                if(data_available == 1){
                    for(int i = 0; i < 16; i++){ 
                        uint32_t ray_word = blocking_receive(self.thread_id);
                        *slot = ray_word;
                        slot = slot + 4;
                    }
                    uint16_t leaf_node_index = ray->leaf_node_starting_point;
                    leaf_node_index <<= 1;
                    uint32_t leaf_core_data_addr = self.leaf_core_lookup_table->leaf_core_ptrs[leaf_node_index];
                    *(slot - 16) = leaf_core_data_addr;
                    slot = 0xFFFFFFFF;
                }
                uint32_t is_odd_thread = self.thread_id & 1;
                uint32_t mailbox_index = is_odd_thread += 32;
                uint32_t interrupt_available = nb_recv(mailbox_index);
                if(interrupt_available != 0){
                    // typedef struct { //16924 Bytes
                    //     uint32_t head_relative; //relative to the start of the queue
                    //     uint32_t tail_relative; //relative to the start of the queue
                    //     uint32_t count;
                    //     struct Ray[16] rays; //16 * 64 bytes for the rays
                    // } ray_queue_sram;
                    //the below should occur on an interrupt which is accepted.

                    uint32_t message = blocking_receive(mailbox_index);
                    uint32_t my_node_id = *self.root_node_id;
                    uint32_t supposed_node_id = (message >> 17);
                    if(supposed_node_id != my_node_id){
                        uint32_t wrong_core_msg = wrong_core << 24;
                        send_packet(wrong_core_msg, (message >> 4) * 0x1FFF, message & 0xF + 16);
                        goto done_with_interrupt;
                    }

                    uint32_t local_queue = self.local_queue + 8; //skip head and tail
                    is_odd_thread = self.thread_id & 1;
                    is_odd_thread *= 1036;
                    local_queue += is_odd_thread; //odd threads write to the receiver queue rather than sender queue
                    uint32_t old_count = atomic_add(&local_queue.count, 1);
                    if(old_count > 16){
                        atomic_add(&local_queue.count, -1);
                        uint32_t reject_ray_msg = reject_ray << 24;
                        send_packet(reject_ray_msg, (message >> 4) * 0x1FFF, message & 0xF + 16);
                        goto done_with_interrupt;
                    }
                    local_queue -= 4;
                    uint32_t tail_relative = atomic_add(&local_queue, 64);
                    tail_relative = tail_relative & 0x000003FF;
                    local_queue += 8;
                    local_queue += tail_relative;
                    slot = local_queue;
                    uint32_t ray_ack_msg = ray_ack << 24 | self.thread_id;
                    send_packet(ray_ack_msg, (message >> 4) * 0x1FFF, message & 0xF + 16);
                }
                //done_with_interrupt:
                if(sent == 1 && slot == 0xFFFFFFFF){
                    goto ray_done;
                }
                goto send_ray_loop;
                
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
            goto start_ray_traversal;
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
goto start_ray_traversal;
// ray_done
if(ray->active_ray == 1){
    goto start_ray_traversal;
}
yield();
//check local ray queue
uint32_t local_ray_count = *(self.local_ray_queue + 8);
if(local_ray_count == 0){
    goto no_rays_available;
}
uint16_t local_ray_queue_head = self.local_ray_queue_head;
uint32_t slot = atomic_add(local_ray_queue_head, 64);
local_ray_queue_head += 8;
atomic_add(local_ray_queue_head, -1);
slot &= 0x7FF;
local_ray_queue_head += slot;
local_ray_queue_head += 4;
uint32_t ray_val = *(local_ray_queue_head + 0);
*(ray + 0) = ray_val;
ray_val = *(local_ray_queue_head + 4);
*(ray + 4) = ray_val;
ray_val = *(local_ray_queue_head + 8);
*(ray + 8) = ray_val;
ray_val = *(local_ray_queue_head + 12);
*(ray + 12) = ray_val;
ray_val = *(local_ray_queue_head + 16);
*(ray + 16) = ray_val;
ray_val = *(local_ray_queue_head + 20);
*(ray + 20) = ray_val;
ray_val = *(local_ray_queue_head + 24);
*(ray + 24) = ray_val;
ray_val = *(local_ray_queue_head + 28);
*(ray + 28) = ray_val;
ray_val = *(local_ray_queue_head + 32);
*(ray + 32) = ray_val;
ray_val = *(local_ray_queue_head + 36);
*(ray + 36) = ray_val;
ray_val = 128;
*(ray + 40) = ray_val;
ray_val = *(local_ray_queue_head + 44);
*(ray + 44) = ray_val;
ray_val = *(local_ray_queue_head + 48);
*(ray + 48) = ray_val;
ray_val = *(local_ray_queue_head + 52);
*(ray + 52) = ray_val;
ray_val = *(local_ray_queue_head + 56);
*(ray + 56) = ray_val;
ray_val = *(local_ray_queue_head + 60);
*(ray + 60) = ray_val;
ray_val &= 0;
*(local_ray_queue_head + 63) = ray_val;
*(self.tile_data_sram->cur_ray_spawned_from_tile + self.thread_id) = 0;
node = ray->leaf_node_starting_point;
goto start_ray_traversal;
//no_rays_available:
yield();
//check dram ray queue
int queue_address_low = self.ray_queue_address_low;
int queue_address_high = self.ray_queue_address_high;
set_address_bits(queue_address_high);
int cur_ray_count = load_dram_word(queue_address_low + 8); //check if there are any rays in the queue
if(cur_ray_count > 0){
    int cur_ray_count_check = atomic_add_dram(queue_address_low + 8, -1); //decrement the count of rays in the queue
    if(cur_ray_count_check <= 0){ //if another core took the last ray, we should check again
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
//check spawned_ray_pool
uint32_t spawned_ray_pool_high = self.spawned_ray_pool_high;
set_address_bits(spawned_ray_pool_high);
uint32_t spawned_ray_pool_low = self.spawned_ray_pool_low;
uint32_t count = load_dram_word(spawned_ray_pool_low + 8);
if(count <= 0){
    goto grab_from_tile;
}
spawned_ray_pool_low += 8;
uint32_t old_cnt = atomic_add(spawned_ray_pool_low, -1);
if (old_cnt <= 0) {
    atomic_add(spawned_ray_pool_low, 1);
    goto grab_from_tile;
}
spawned_ray_pool_low -= 8;
uint32_t head = atomic_add(spawned_ray_pool_low, 32);
uint32_t head_mask = 0x007FFFFF;
head &= head_mask;
spawned_ray_pool_low += head;

//ensure_slot_ready:
uint8_t slot_ready = load_dram_byte(spawned_ray_pool_low + 43);
if(slot_ready == 0){
    goto ensure_slot_ready;
}

uint32_t value_one   = load_dram_word(spawned_ray_pool_low + 12); // ox
uint32_t value_two   = load_dram_word(spawned_ray_pool_low + 16); // oy
uint32_t value_three = load_dram_word(spawned_ray_pool_low + 20); // oz

ray->ox = value_one;
ray->oy = value_two;
ray->oz = value_three;

uint32_t value_four  = load_dram_word(spawned_ray_pool_low + 24); // dx
uint32_t value_five  = load_dram_word(spawned_ray_pool_low + 28); // dy
uint32_t value_six   = load_dram_word(spawned_ray_pool_low + 32); // dz

ray->dx = value_four;
ray->dy = value_five;
ray->dz = value_six;

uint32_t pix_xy = load_dram_word(spawned_ray_pool_low + 36);
uint32_t meta   = load_dram_word(spawned_ray_pool_low + 40);
ray->pix_x = pix_xy;
ray->bounce_count = meta & 0xFF;
ray->light_id = (meta >> 8) & 0xFF;

uint32_t is_shadow = ray->light_id;
float len_sq = ray->dx * ray->dx;
float tmp = ray->dy * ray->dy;
len_sq += tmp;
tmp = ray->dz * ray->dz;
len_sq += tmp;
float inv_len = fast_inv_sqrt(len_sq);
ray->dx = ray->dx * inv_len;
ray->dy = ray->dy * inv_len;
ray->dz = ray->dz * inv_len;
ray->inv_dx = reciprocal(ray->dx);
ray->inv_dy = reciprocal(ray->dy);
ray->inv_dz = reciprocal(ray->dz);
if(is_shadow != 0){
    ray->t_max = reciprocal(inv_len);
}
else{
    ray->t_max = 0x7F800000;
}

ray->check_left  = 0;
ray->check_right = 0;
ray->tri_index = 0xFFFFFFFF;
ray->leaf_node_starting_point = 128;
ray->active_ray = 1;
ray->ray_depth = 0;

store_dram_byte(spawned_ray_pool_low + 43, 0);

goto start_ray_traversal;


//grab_from_tile:
uint16_t is_active = *(self.tile_data_sram->is_active);
if(!is_active){
    goto get_new_tile;
}
uint32_t tile_total_count = *(self.tile_data_sram->count);
if(tile_total_count > 255){
    goto skip_returning_tile;
}
uint32_t rays_spawned_from_tile = *(self.tile_data_sram->rays_spawned_from_tile);
if(rays_spawned_from_tile < 7){
    goto spawn_from_tile;
}
uint32_t rays_forwarded_out_from_tile = *(self.tile_data_sram->rays_forwarded_out_from_tile);
rays_forwarded_out_from_tile <<= 1;
if(rays_forwarded_out_from_tile < rays_spawned_from_tile){
    goto spawn_from_tile;
}

//get_new_tile:
get_ownership();
uint32_t tile_pool_high = self.tile_pool_high;
set_address_bits(tile_pool_high);
uint32_t tile_pool_low = self.tile_pool_low;
//NEED TO THROW CURRENT TILE BACK CONDITIONALLY ON THE QUEUE
uint8_t have_tile = *(self.tile_data_sram->is_active);
if(have_tile == 0){
    goto skip_returning_tile;
}
uint8_t tile_rays_spawned = *(self.tile_data_sram->count);
if(tile_rays_spawned > 255){
    goto skip_returning_tile;
}
tile_pool_low += 4;
uint32_t bytes_relative_tail = atomic_add_dram(tile_pool_low, 4);
tile_pool_low += 4;
atomic_add_dram(tile_pool_low, 1);
bytes_relative_tail &= 0x0000FFFF;
tile_pool_low += bytes_relative_tail;
//loop_on_putting_tile_back:
tile_pool_low += 4;
uint8_t is_valid = load_dram_byte(tile_pool_low + 3);
if(is_valid != 0){
    goto loop_on_putting_tile_back;
}
uint32_t tile_y_index = *(self.tile_data_sram->tile_y_index);
tile_y_index *= 160; // 2560 / 16
uint32_t tile_x_index = *(self.tile_data_sram->tile_x_index);
uint16_t tile_index = tile_y_index + tile_x_index;
store_dram_half(tile_pool_low, tile_index);
uint8_t tile_rays_spawned = *(self.tile_data_sram->count);
tile_pool_low += 2;
store_dram_byte(tile_pool_low, tile_rays_spawned);
tile_pool_low += 1;
uint8_t one_small = 1;
store_dram_byte(tile_pool_low, one_small);

//skip_returning_tile:
uint32_t tile_pool_low = self.tile_pool_low;
uint32_t count = load_dram_word(tile_pool_low + 8);
if(count <= 0){
    goto skip_grabbing_tile_rays;
}
tile_pool_low += 8;
uint32_t old_cnt = atomic_add(tile_pool_low, -1);
if (old_cnt <= 0) {
    atomic_add(tile_pool_low, 1);
    goto skip_grabbing_tile_rays;
}
tile_pool_low -= 8;
uint32_t head = atomic_add(tile_pool_low, 4);
uint32_t head_mask = 0x0000FFFF;
head &= head_mask;
tile_pool_low += head;

//ensure_tile_slot_ready:
uint8_t slot_ready = load_dram_byte(tile_pool_low + 15);
if(slot_ready == 0){
    goto ensure_tile_slot_ready;
}
uint16_t tile_index = load_dram_half(tile_pool_low + 12);
uint16_t tile_cnt = load_dram_byte(tile_pool_low + 14);
tile_pool_low += 15;
store_dram_byte(tile_pool_low, 0);
self.tile_data_sram->count = tile_cnt;
uint32_t tile_y_index = tile_index / 160;
uint32_t tile_x_index = tile_y_index * 160;
tile_x_index = tile_index - tile_x_index;
self.tile_data_sram->tile_x_index = tile_x_index;
self.tile_data_sram->tile_y_index = tile_y_index;
uint32_t zero = 0;
*(self.tile_data_sram->cur_ray_spawned_from_tile + 0) = zero;
*(self.tile_data_sram->cur_ray_spawned_from_tile + 4) = zero;
*(self.tile_data_sram->cur_ray_spawned_from_tile + 8) = zero;
*(self.tile_data_sram->cur_ray_spawned_from_tile + 12) = zero;
*(self.tile_data_sram->cur_ray_spawned_from_tile + self.thread_id) = 1;
self.tile_data_sram->rays_forwarded_out_from_tile = zero;
self.tile_data_sram->rays_spawned_from_tile = zero;
relinquish_ownership();
//spawn_from_tile:
uint16_t tile_data_sram_address = &(self.tile_data_sram->count);
uint32_t ray_num_from_tile = atomic_add(tile_data_sram_address, 1);
if(ray_num_from_tile > 255) {
    goto get_new_tile;
}
tile_data_sram_address += 24;
atomic_add(tile_data_sram_address, 1);
uint32_t intra_tile_x = ray_num_from_tile & 0xF;
uint32_t intra_tile_y = ray_num_from_tile >> 4;
uint32_t inter_tile_x = *(self.tile_data_sram->tile_x_index);
uint32_t inter_tile_y = *(self.tile_data_sram->tile_y_index);
inter_tile_x <<= 4;
inter_tile_y <<= 4;
uint32_t pix_x = inter_tile_x + intra_tile_x;
uint32_t pix_y = inter_tile_y + intra_tile_y;

// === Generate camera ray from (pix_x, pix_y) ===
ray->pix_x = pix_x;
ray->pix_y = pix_y;

uint32_t itof_table_high = self.itof_table_high;
set_address_bits(itof_table_high);
uint32_t itof_table_low = self.itof_table_low;

uint32_t x_offset = pix_x << 2;
uint32_t y_offset = pix_y << 2;
float fpix_x = load_dram_word(itof_table_low + x_offset);
float fpix_y = load_dram_word(itof_table_low + y_offset);

float cam_cx = *(self.cam_cx);
float cam_cy = *(self.cam_cy);
float cam_inv_f = *(self.cam_inv_f);

float dx = fpix_x - cam_cx;
float dy = fpix_y - cam_cy;
dx = dx * cam_inv_f;
dy = dy * cam_inv_f;
float dz = 0xBF800000; // -1.0f

// Normalize
float len_sq = dx * dx;
float tmp = dy * dy;
len_sq += tmp;
tmp = dz * dz;
len_sq += tmp;
float inv_len = fast_inv_sqrt(len_sq);

dx = dx * inv_len;
dy = dy * inv_len;
dz = dz * inv_len;

// Reciprocals
float inv_dx = reciprocal(dx);
float inv_dy = reciprocal(dy);
float inv_dz = reciprocal(dz);
ray->dx = dx;
ray->dy = dy;
ray->dz = dz;
ray->inv_dx = inv_dx;
ray->inv_dy = inv_dy;
ray->inv_dz = inv_dz;
// === Store ray ===
float ox = *(self.cam_x);
float oy = *(self.cam_y);
float oz = *(self.cam_z);
ray->ox = ox;
ray->oy = oy;
ray->oz = oz;

uint32_t epsilon = 0x38D1B717; // ~1e-5
uint32_t pos_inf = 0x7F800000;
ray->t_max = pos_inf;

uint32_t zero = 0;
ray->check_left = zero;
ray->check_right = zero;
ray->bounce_count = zero;
uint8_t one_byte = 1;
ray->active_ray = one_byte;
uint32_t no_hit = 0xFFFFFFFF;
ray->tri_index = no_hit;
goto start_ray_traversal;

//skip_grabbing_tile_rays
yield();
//check to see if we're done
uint32_t finished_ray_high = self.ray_result_addr_high;
set_address_bits(finished_ray_high);
uint32_t finished_ray_low = self.ray_result_addr_low;
uint32_t rays_finished = load_dram_word(finished_ray_low);
uint32_t max_rays = 1440 * 2560 * 4;
yield();
if(rays_finished != max_rays){
    goto ray_done;
}

// pixel_color subroutine
// NUM_BOUNCES is a compile-time constant
get_thread_ownership();
set_ctx(15);
relinquish_ownership();
yield();
uint32_t pixel_addr_high = self.ray_result_addr_high;
set_address_bits(pixel_addr_high);
uint32_t pixel_addr_low = self.ray_result_addr_low;
uint32_t pix_index = self.core_id >> 4;
uint32_t thread_index = self.core_id & 0xF;
pix_index *= 15;
pix_index += 15;
pix_index <<= 8;
uint32_t pix_increment = pix_index;
//loop_pixel:
uint32_t pixel_addr_low = self.ray_result_addr_low;
pixel_addr_low += pix_increment;

float carried_r = 0.0f;
float carried_g = 0.0f;
float carried_b = 0.0f;

uint32_t bounce = NUM_BOUNCES - 1;
// bounce_loop:
    uint32_t bounce_addr = bounce;
    bounce_addr <<= 6;
    bounce_addr += pixel_addr_low;

    float sr = load_dram_word(bounce_addr);
    float sg = load_dram_word(bounce_addr + 4);
    float sb = load_dram_word(bounce_addr + 8);
    float metallic = load_dram_word(bounce_addr + 12);

    float acc_r = 0.0f;
    float acc_g = 0.0f;
    float acc_b = 0.0f;

    uint32_t shadow_addr = bounce_addr + 16;
    uint32_t light = 0;
    // shadow_loop:
        uint32_t len_sq = load_dram_word(shadow_addr + 12);
        if(len_sq != 0xFFFFFFFF) goto shadow_skip;
            float lr = load_dram_word(shadow_addr);
            float lg = load_dram_word(shadow_addr + 4);
            float lb = load_dram_word(shadow_addr + 8);
            float atten = reciprocal(len_sq);
            lr *= atten;
            lg *= atten;
            lb *= atten;
            acc_r += lr;
            acc_g += lg;
            acc_b += lb;
        shadow_skip:
        shadow_addr += 16;
        light += 1;
        if(light < NUM_LIGHTS) goto shadow_loop;

    float diffuse_r = acc_r * sr;
    float diffuse_g = acc_g * sg;
    float diffuse_b = acc_b * sb;

    carried_r *= sr;
    carried_g *= sg;
    carried_b *= sb;

    float one = 1.0f;
    float inv_metallic = one - metallic;
    diffuse_r *= inv_metallic;
    diffuse_g *= inv_metallic;
    diffuse_b *= inv_metallic;
    carried_r *= metallic;
    carried_g *= metallic;
    carried_b *= metallic;
    carried_r += diffuse_r;
    carried_g += diffuse_g;
    carried_b += diffuse_b;

    if(bounce == 0) goto bounce_done;
    bounce -= 1;
    goto bounce_loop;

// bounce_done:
// carried_r/g/b is final pixel color
float one = 1.0f;
carried_r += one;
carried_g += one;
carried_b += one;
carried_r >>= 14;//using 9 bits to reduce aliasing
carried_g >>= 14;
carried_b >>= 14;
carried_r &= 0x1FF;
carried_g &= 0x1FF;
carried_b &= 0x1FF;
red_byte = *(self.table_mappings + carried_r);
green_byte = *(self.table_mappings + carried_g);
blue_byte = *(self.table_mappings + carried_b);

uint32_t pixel_addr_high = self.frame_buffer_high;
set_address_bits(pixel_addr_high);
uint32_t pixel_addr_low = self.frame_buffer_low;
uint32_t pix_offset = pix_increment >> 6; // each pixel is 4 bytes
pixel_addr_low += pix_offset;
store_dram_byte(red_byte, pixel_addr_low);
pixel_addr_low += 1;
store_dram_byte(green_byte, pixel_addr_low);
pixel_addr_low += 1;
store_dram_byte(blue_byte, pixel_addr_low);
pix_increment += pix_index;
uint32_t max_rez = 2560 * 1440;
max_rez <<= 8;
//need some atomic to increment up till 2560 * 1440;  
uint32_t finished_pixel_high = self.finished_pixel_high;
set_address_bits(finished_pixel_high)
uint32_t finished_pixel_low = self.finished_pixel_low;
atomic_add_dram(finished_pixel_low, 1);
if(max_rez > pix_increment){
    goto loop_pixel;
}
//inf_loop:
goto inf_loop;

int AABB_Intersect(AABB_Node* node, Ray* ray) {
    float tx1 = (node->x_min - ray->ox) * ray->inv_dx;
    float tx2 = (node->x_max - ray->ox) * ray->inv_dx;

    float tmin = min(tx1, tx2);
    float tmax = max(tx1, tx2);

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

uint32_t finished_ray_high = self.ray_result_addr_high;
set_address_bits(finished_ray_high);
uint32_t finished_ray_low = self.ray_result_addr_low;
atomic_add(finished_ray_low, 1);
uint32_t pix_index = ray->pix_y;
pix_index *= 2560;
pix_index += ray->pix_x;
pix_index <<= 8;
result_addr_low += pix_index;
uint32_t bounce = ray->bounce_count;
bounce <<= 6;
result_addr_low += bounce;


if(ray->light_id != 0) {
    goto shadow_ray;
}
if(ray->tri_index == 0xFFFFFFFF){
    uint32_t zero = 0;
    ray->active_ray = zero;
    goto ray_done
}

uint32_t tri_addr_high = self.triangle_address_high;
set_address_bits(tri_addr_high);
uint32_t tri_addr_low = self.triangle_address_low;
uint32_t tri_offset = ray->tri_index << 5;  // * 32 bytes
tri_addr_low += tri_offset;
float tri_red = load_dram_word(tri_addr_low);
float tri_green = load_dram_word(tri_addr_low + 4);
float tri_blue = load_dram_word(tri_addr_low + 8);
float tri_metallic = load_dram_word(tri_addr_low + 16);
float norm_x = load_dram_word(tri_addr_low + 20);
float norm_y = load_dram_word(tri_addr_low + 24);
float norm_z = load_dram_word(tri_addr_low + 28);
set_address_bits(result_addr_high);
store_dram_word(result_addr_low,      tri_red);
store_dram_word(result_addr_low + 4,  tri_green);
store_dram_word(result_addr_low + 8,  tri_blue);
store_dram_word(result_addr_low + 12,  tri_metallic);
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
//need to spawn shadow rays here:
uint32_t new_ray_pool_high = self.new_ray_pool_high;
set_address_bits(new_ray_pool_high);
uint32_t new_ray_pool_low = self.new_ray_pool_low;
//ensure_space_ray_pool
uint32_t cur_num_new_rays = load_dram_word(new_ray_pool_low + 8);
if(cur_num_new_rays > 260000){
    goto ensure_space_ray_pool;
}
new_ray_pool_low += 8;
atomic_add_dram(new_ray_pool_low, 3)
new_ray_pool_low -= 4;
uint32_t tail_slot = atomic_add_dram(new_ray_pool_low, 32);//need to break this into 3 transactions, since it could wrap around
uint32_t tail_mask = 0x007FFFFF;
tail_slot &= tail_mask;
new_ray_pool_low += 8;

// hit_x, hit_y, hit_z still in registers
uint16_t light = self.light_array;

uint32_t light_offsets[3];
light_offsets[0] = 12;
light_offsets[1] = 36;
light_offsets[2] = 60;

uint32_t i = 0;
//shadow_ray_loop
uint32_t slot_base = tail_slot;
slot_base &= tail_mask;
slot_base += new_ray_pool_low;

float lx = *(light + light_offsets[i]);
float ly = *(light + light_offsets[i] + 4);
float lz = *(light + light_offsets[i] + 8);
float sdx = lx - hit_x;
float sdy = ly - hit_y;
float sdz = lz - hit_z;

//ensure_empty
uint8_t is_empty = load_dram_byte(slot_base + 31);
if(!is_empty){
    goto ensure_empty;
}
uint32_t one = 1;
store_dram_word(slot_base,      hit_x);
store_dram_word(slot_base + 4,  hit_y);
store_dram_word(slot_base + 8,  hit_z);
store_dram_word(slot_base + 12, sdx);
store_dram_word(slot_base + 16, sdy);
store_dram_word(slot_base + 20, sdz);
uint32_t pix_xy = ray->pix_x;
pix_xy |= (ray->pix_y << 16);
store_dram_word(slot_base + 24, pix_xy);
uint32_t light_id = i + 1;
uint32_t meta = ray->bounce_count;
meta |= light_id << 8;
store_dram_word(slot_base + 28, meta);
store_dram_byte(slot_base + 31, one);

tail_slot += 32;
tail_slot &= tail_mask;
i += 1;
if(i < 3){
    goto shadow_ray_loop;
}

if(ray->bounce_count > 2) {
    ray->active_ray = 0;
    goto ray_done;
}
uint32_t random_table_high = self.random_table_addr_high;
set_address_bits(random_table_high);
uint32_t random_table_low = self.random_table_addr_low;
uint32_t index = atomic_add_dram(random_table_low, 12);
uint32_t mask = 0x0003FFFC;
index &= mask;
random_table_low += index;
random_table_low += 4;
float random1 = load_dram_word(random_table_low);
float random2 = load_dram_word(random_table_low + 4);
float random3 = load_dram_word(random_table_low + 8);
uint32_t or_mask = 0x3F800000;
uint32_t and_mask = 0x3FFFFFFF;
random1 &= and_mask;
random1 |= or_mask;

random2 &= and_mask;
random2 |= or_mask;

random3 &= and_mask;
random3 |= or_mask;

uint32_t one_point_five = 1.5f;
random1 -= one_point_five;
random2 -= one_point_five;
random3 -= one_point_five;
uint8_t bounce = ray->bounce_count;
bounce += 1;
ray->bounce_count = bounce;
uint8_t zero = 0;
ray->ray_depth = zero;
ray->check_left = zero;
ray->leaf_node_starting_point = 128;
ray->check_right = zero;
zero |= 0xFFFFFFFF;
uint32_t high_triangle = self.triangle_array_high;
uint32_t low_triangle = self.triangle_array_low;
set_address_bits(high_triangle);
uint32_t tri_index = ray->tri_index;
tri_index <<= 5;
low_triangle += tri_index;
float roughness = load_dram_word(low_triangle + 12);
float norm_x = load_dram_word(low_triangle + 20);
float norm_y = load_dram_word(low_triangle + 24);
float norm_z = load_dram_word(low_triangle + 28);
ray->tri_index = zero;
float hit_x = ray->ox + ray->t_max * ray->dx;
float hit_y = ray->oy + ray->t_max * ray->dy;
float hit_z = ray->oz + ray->t_max * ray->dz;
ray->ox = hit_x;
ray->oy = hit_y;
ray->oz = hit_z;
float dot_dn = ray->dx * norm_x + ray->dy * norm_y + ray->dz * norm_z;
dot_dn += dot_dn;  // 2 * dot(d, n)

ray->dx = ray->dx - dot_dn * norm_x;
ray->dy = ray->dy - dot_dn * norm_y;
ray->dz = ray->dz - dot_dn * norm_z;
random1 *= roughness;
random2 *= roughness;
random3 *= roughness;

float bdx = ray->dx;
float bdy = ray->dy;
float bdz = ray->dz;

bdx += random1;
bdy += random2;
bdz += random3;

// normalize before check flip
float len_sq = bdx * bdx;
float tmp = bdy * bdy;
len_sq += tmp;
tmp = bdz * bdz;
len_sq += tmp;

float inv_sqrt = fast_inv_sqrt(len_sq);

bdx *= inv_sqrt;
bdy *= inv_sqrt;
bdz *= inv_sqrt;

// check flip
float nx = norm_x;
float ny = norm_y;
float nz = norm_z;

float check = bdx * nx;
tmp = bdy * ny;
check += tmp;
tmp = bdz * nz;
check += tmp;

if (check < 0.0) {
    check += check;
    tmp = check * nx;
    bdx -= tmp;
    tmp = check * ny;
    bdy -= tmp;
    tmp = check * nz;
    bdz -= tmp;
}

ray->dx = bdx;
ray->dy = bdy;
ray->dz = bdz;

float float_max = 0x7F7FFFFF;
ray->t_max = float_max;
float recip_bdx = reciprocal(bdx);
ray->inv_dx = recip_bdx;
float recip_bdy = reciprocal(bdy);
ray->inv_dy = recip_bdy;
float recip_bdz = reciprocal(bdz);
ray->inv_dz = recip_bdz;

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



//reciprocal subroutine. Expands in precision with larger table
//load in x somehow
uint32_t neg_max = 0x80000000;
uint32_t sign = x & neg_max;
neg_max ^= 0xFFFFFFFF;
x &= neg_max;
uint32_t original_magnitude = x;

uint32_t exp = x >> 23;
uint32_t new_exp = 254 - exp;

uint32_t index = x >> 12;
index &= 0x7FF;
index <<= 2;
uint32_t table_addr = self.div_table_high;
set_address_bits(table_addr);
table_addr = self.div_table_low;
table_addr += index;
uint32_t reciprocal_lookup = load_dram_word(table_addr);

new_exp <<= 23;
reciprocal_lookup |= new_exp;

// NR: r1 = r0 * (2 - x * r0)
float xf = original_magnitude;  // move to float register
float r0 = reciprocal_lookup;   // move to float register
float t = xf * r0;              // x * r0
float two = 2.0f;
t = two - t;                   // 2 - x*r0
r0 = r0 * t;                    // r1 = r0 * (2 - x*r0)
r0 |= sign;
// r0 is returned



// fast_inv_sqrt:
// input: len_sq (float, in register)
// output: half_len_sq (float, 1/sqrt(len_sq))

// extract exponent and top mantissa bits as table index
uint32_t index = len_sq >> 17;         // top 15 bits of float (1 exp + 14 mantissa)
uint32_t inv_sqrt_table_high = self.inv_sqrt_table_high;
set_address_bits(inv_sqrt_table_high);
uint32_t inv_sqrt_table_low = self.inv_sqrt_table_low;
index <<= 2;                         // * 4 bytes per entry
inv_sqrt_table_low += index;
float est = load_dram_word(inv_sqrt_table_low);  // table gives ~14 bits of precision

// one Newton-Raphson refinement: est = est * (1.5 - 0.5 * len_sq * est * est)
uint32_t half = 0x3F000000;
float half_len_sq = len_sq * half;
uint32_t one_point_five = 0x3FC00000;
half_len_sq *= est;
half_len_sq *= est;
half_len_sq = one_point_five - half_len_sq;
half_len_sq *= est;
//resut is half_len_sq



/* Stuff left:
Interrupts
*/


