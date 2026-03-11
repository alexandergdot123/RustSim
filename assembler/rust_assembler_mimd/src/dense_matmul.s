.org 0x0028
.data 105
setctx 16
relinquish
# r0 starts at 0 (if not guaranteed, keep this)
and r15, r15, 0 #should have zero effect, just for testing purposes
and r0, r0, 0
and r6, r6, 0
and r5, r5, 0
and r7, r7, 0
add r7, r7, 4
setmembits r0
add r1, r0, 17384
add r0, r0, 1000 #base address M1 = 1000
# r14 = coreID = r15 >> 4   (MAX_THREADS = 16 => shift by 4)
and r14, r15, 0xF #r14 = tid
srl r12, r15, 4
and r12, r12, 0x3F #r12 = x
srl r13, r15, 10 #4 + log2(64)
and r13, r13, 0x3F #r13 = y

#each core has its offset now
mul r11, r13, 256 #4 * 64
add r0, r0, r11
mul r11, r12, 4
add r1, r1, r11
#next, i need thread by thread offsets
mul r11, r14, 4
add r0, r0, r11
mul r11, r14, 256 #4 * 64
add r1, r1, r11

START_LOOP: 
and r11, r11, 0
bne r11, r12, GET_X_FROM_MAILBOX, true
lw_d r2, r0
beq r15, r15, DO_Y, true
GET_X_FROM_MAILBOX: 
block r2, r14 
DO_Y: 
bne r11, r13, GET_Y_FROM_MAILBOX, true
lw_d r3, r1
beq r15, r15, START_MUL, true
GET_Y_FROM_MAILBOX: 
add r10, r14, 16
block r3, r10 
START_MUL: 
mul r4, r2, r3
add r5, r4, r5
and r11, r11, 0
add r11, r11, 63
beq r12, r11, SKIP_SEND_X, false
srl r10, r15, 4
add r10, r10, 1
sendflit r10, r2, r14
SKIP_SEND_X: 
beq r13, r11, SKIP_SEND_Y, false
srl r10, r15, 4
add r10, r10, 64
add r9, r14, 16
sendflit r10, r3, r9 #value, core, mailbox
SKIP_SEND_Y: 
add r0, r0, 64
add r1, r1, 4096
add r6, r6, 1
bgt r7, r6, START_LOOP, true
and r9, r9, 0
add r9, r9, ACCUM
mul r8, r14, 4
add r9, r9, r8
sw r5, r9
INF_LOOP: 
yield r8 #yield will not do interrupts since no interrupts were enabled
and r8, r8, 0
bne r8, r14, INF_LOOP, true
getowner
and r0, r0, 0
add r1, r0, 16
add r2, r0, ACCUM
and r4, r4, 0
LAST_LOOP: 
lw r3, r2
add r4, r4, r3
add r2, r2, 4
add r0, r0, 1
bgt r1, r0, LAST_LOOP, true
srl r14, r15, 4
mul r14, r14, 4
and r13, r13, 0
add r13, r13, 256
mul r13, r13, 256
mul r13, r13, 256
add r13, r13, r14
sw_d r4, r13
TRUE_INF_LOOP: 
sll r15, r15, 1
beq r15, r15, TRUE_INF_LOOP, true
ACCUM: 
.data 0
.data 0
.data 0
.data 0
.data 0
.data 0
.data 0
.data 0
.data 0
.data 0
.data 0
.data 0
.data 0
.data 0
.data 0
.data 0