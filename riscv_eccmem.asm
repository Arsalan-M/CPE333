li x10, 0x00001000
li x11, 200
li x12, 300
li x13, 400
sw x11, 0(x10)
sw x12, 4(x10)
sw x13, 8(x10)
lw x14, 0(x10)
lw x15, 4(x10)
lw x16, 8(x10)
