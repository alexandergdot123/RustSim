.org 0x0028
.data 14
# r0 starts at 0 (if not guaranteed, keep this)
and r0, r0, 0
and r5, r5, 0
# r5 = 1000 (address we accumulate into)
add r5, r0, 1000

# r14 = coreID = r15 >> 4   (MAX_THREADS = 16 => shift by 4)
srl r14, r15, 4

# if coreID == 0, jump to aggregator
beq r0, r14, LOOP

# ---- Worker cores (coreID != 0) ----
setctx 16
relinquish

# r13 = floor(r15 / 7)  (Div supports immediate per your rule 1)
div r13, r15, 7

# sendflit MAILBOX=0, VALUE=r13, DEST_CORE=r0 (core 0)
sendflit 0, r13, r0

DONE_1:
    yield r6
    beq r15, r15, DONE_1

# Loop forever: receive one value and atomic add into M[1000]
LOOP:
    block r2, 0              # r2 = received value (mailbox 0)
    atomadd_d r9, r2, r5     # M[r5 + 0] += r2  => M[1000] += r2
    beq r15, r15, LOOP
