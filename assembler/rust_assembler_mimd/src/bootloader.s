.org 0x0000

and r0, r0, 0 #00
setmembits r0 #04

lw_d  r1, r0      #08   
add r2, r0, 4     #0C
add r3, r0, done #10

loop:
    lw_d  r4, r2 #14
    sw  r4, r3 #18
    add r2, r2, 4 #1C
    add r3, r3, 4 #20
    add r0, r0, 1 #24
    bgt r1, r0, loop #28
done: 
    .data 0xDEADBEEF # i should try to support blt and bgte as well