.orig x3000

and r1, r1, #0
ld r1, count

loop
lea r0, msg
puts
add r1, r1, #-1
brz end_loop
br loop

end_loop
halt

msg .stringz "hello world!\n"
count .FILL #5

.end