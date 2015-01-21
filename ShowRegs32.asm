; ShowRegs.nasm  2012 Oct 19 show all registers based on code by John Commeau - thanks!

showstate:
    pusha  ; 8 registers in order AX, CX, DX, BX, SP, BP, SI, DI
    push cs
    push ds
    push es
    push ss
    push fs
    push gs
    push 0xb8000 >> 4  ;screen memory of color display
    pop es
    mov bp, sp
    xor di, di  ;point to bottom of screen memory (top of screen)
    mov si, 28  ;point to return address (past 14 pushed registers)
    mov ax, [bp+si]  ;get return address as a reference to data
    and ax, ~0x1ff  ;round down to nearest sector (512 bytes)
    shr ax, 4
    mov ds, ax
    call outheader

.loop:
    mov ax, [bp+si]  ;display return address followed by 14 registers
    call outword  ;from left of screen
    sub si, 2
    jnc .loop

    add sp, 6 ;toss top 3 saved segment registers
    pop es
    pop ds
    pop ax  ;discard saved CS register
    popa
    ret

outword:  ;assume registers saved by caller, we trash AX-CX and update DI
    mov cx, 4
    push ax
    mov ax, 0x700  ;show characters in silver (dim white)

.loop:
    pop bx
    rol bx, 4  ;show highest nybble first
    push bx
    and bx, 0xf
    mov al, [hextable+bx]
    stosw
    loop .loop

    xor al, al
    stosw  ;blank space after final hex byte
    pop ax
    ret

outheader:
    push si
    mov ah, 0xf  ;white text on black background
    mov si, header
    mov cx, 80
.loop:
    lodsb
    stosw
    loop .loop
    pop si
    ret

hextable:
    db '0123456789ABCDEF'
header:
    db 'IP   AX   CX   DX   BX   SP   BP   SI   DI   CS   DS   ES   SS   FS   GS'

times 80 - ($ - header) db 0  ;pad with nulls
times 510 - ($ - $$) db 0  ;pad with nulls to 55AA at end of boot sector
db 0x55, 0xaa
