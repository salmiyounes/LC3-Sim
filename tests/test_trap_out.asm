.orig x3000

ld r0, char_A
trap x21         

ld r0, newline
trap x21         

trap x25         ; HALT

char_A .FILL x41 ; ASCII 'A'
newline .FILL x0A ; ASCII line feed

.end

