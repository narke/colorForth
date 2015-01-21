; boot.asm  colorForth for 80x86 PC  Floppy disk boot sector  2012 Oct 19  Howerd Oakford  
; Ported to NASM format.
; Based on code by :
; Chuck Moore (inventor, MASM)
; Mark Slicker (ported to GNU Assembler)
; Peter Appelman (ported to NASM with qwerty mods)
; and others...	Thanks!!!
; Feedback welcome to howerd@inventio.co.uk
;

ORG 0			 		; actually 7c00
BITS 16

start: jmp short  start0
    nop
    db 'cmcf 4.1'
    dw 512     ; bytes/sector
    db 1       ; sector/cluster
    dw 1       ; sector reserved
    db 2       ; FATs
    dw 16*14   ; root directory entries
    dw 80*2*18 ; sectors
    db 0F0h    ; media
    dw 9       ; sectors/FAT
    dw 18      ; sectors/track
    dw 2       ; heads
    dd 0       ; hidden sectors
    dd 80*2*18 + 0xC7000000 ; sectors again  ; to match cfdos4.blk
    db 0       ; drive 
;    db 0
;    db 29h     ; signature
;    dd 44444444h     ; serial
;    db 'COLOR FORTH' ; label
;    db '        '

command  db 8	; to match cfdos4.blk
         db 0   ; head, drive 
cylinder db 0	
         db 0   ; head
         db 1   ; sector
         db 2   ; 512 bytes/sector
         db 18  ; sectors/track
         db 1bh ; gap
         db 0ffh            ; # $2D
	 db 89h  ; to match cfdos4.blk
	 db 0f6h ; to match cfdos4.blk
ALIGN 4
nc dd 16 ; Forth+Icons = 1 sector, plus Source blocks 18 to 252 = 17 sectors    ; # $30

;struct gdt_ptr_struct
; {
;    u16int limit;               // The upper 16 bits of all selector limits.
;    u32int base;                // The address of the first gdt_entry_t struct.
; }
;  __attribute__((packed));
; typedef struct gdt_ptr_struct gdt_ptr_t; 

gdt:
	dw 0x17
   	dd gdt0
	dd 0 ; to match cfdos4.blk
	dw 0 ; to match cfdos4.blk
	align 8

;struct gdt_entry_struct
; {
;    u16int limit_low;    0xFFFF // The lower 16 bits of the limit.
;    u16int base_low;     0x0000 // The lower 16 bits of the base.
;    u8int  base_middle;  0x9B   // The next 8 bits of the base.
;    u8int  access;       0x00   // Access flags, determine what ring this segment can be used in.
;    u8int  granularity;  0xCF
;    u8int  base_high;    0x00   // The last 8 bits of the base.
; } __attribute__((packed));
; typedef struct gdt_entry_struct gdt_entry_t;

gdt0:
    dw 0, 0, 0, 0
    dw 0x0FFFF, 0, 0x9B00, 0x00CF 		; code
    dw 0x0FFFF, 0, 0x9200, 0x00CF 		; data

start0:
	mov ax, 0x7c00				; executed from 7c00
	mov es, ax
;	xor di, di				; at the end of sector
	mov di,0x200
	mov ax, 0x4f01
	mov cx, vesa+0x4000			; ask for linear buffer
	int 0x10
	mov ax, 0x4F02 				; Video mode
    	mov bx, vesa+0x4000			; hp*vp rgb: 565
    	int 0x10	
	mov bp,[es:di+0x2a]
	shl ebp,0x10       
	mov bp,[es:di+0x28] 
    	cli  
	
    xor eax,eax				; Move code to 0
	mov ebx,eax
	mov bx,cs
	mov ds,bx
	mov es,ax
	mov edi,eax
	mov esi,eax
    	call $+3 				; Where are we? IP+4*CS
loc: 
	pop si
    	sub si, loc-start
    	mov cx, 512/4
    	rep movsd
	jmp 0x0:relocate

relocate:
	mov ds,ax	 ; 0x4A
	lgdt [gdt]
	mov al,0x1
	mov cr0,eax

	jmp 0x8:protected

	BITS 32
protected:
	mov al, 0x10
	mov ds, eax
	mov es, eax
	mov ss, eax
	mov esp, Gods
	xor ecx, ecx

A20:
	mov al, 0xd1
	out 0x64, al
.a20:	in al, 0x64
	and al, 2
	jnz .a20
	mov al, 0x4b
	out 0x60, al
  
cold:
	mov esi, Godd
	call spin
;	call dma
; dma:
	mov word [command+1], 0x02a1		; 2 6 16 ms (e 2)
	mov al, 3				; timing
	mov cl, 3
	call cmd
    mov  word [command+1], 0
    mov  eax, buffer*4           
    out  4, al                   
    mov  al, ah                  
    out  4, al                   
    shr  eax, 16                 
    out  0x81, al                 
    mov  eax, 512*18*2-1 ; dma 2 
    out  5, al                   
    mov  al, ah                  
    out  5, al                   
    mov  al, 0x0b                 
    out  0x0f, al                 
;    ret

	xor edi, edi				; Cylinder 0 on addr 0
	call read
	inc byte [cylinder]
	mov cl, byte [nc]
.cold:  push ecx
	call read
	pop ecx
	loop .cold
start2: call stop
	jmp start1

stop:						; Motor off
	mov cl, 0xc
onoff:
	DUP_
	mov al, cl
	mov dx, 0x3f2
	out dx, al
	out 0xe1, al
	DROP
	ret

us:	equ 1000/4
ms:	equ 1000*us

spin:
	mov cl, 0x1c
	call onoff
        mov  ECX, 400*ms                    ; (c4a)
.spin:	loop .spin
;	mov  [cylinder], cl ; calibrate     ; (c4a)
	mov  AL, 7
	mov  CL, 2
	jmp short cmdi

ready:
	mov edx, 0x3f4
.re:	in al, dx
	out 0xe1, al
	shl al, 1
	jnc .re
	lea edx, [edx+1]
	ret

cmd:	push esi
	lea esi, [command]
	mov [esi], al
Xcmd:   call ready
        jns  cmd0
	in   AL, DX
	OUT  0E1h,AL ; tll ; (c4a)
	jmp short Xcmd
cmd0:   lodsb
        out  DX, AL
        push    ecx        ; (c4a)
        mov     ecx,001eh  ; (c4a)
cmd1:   out 0e1h,al        ; (c4a)
        loop cmd1          ; (c4a)
        pop ecx            ; (c4a)
        loop Xcmd
    pop  ESI
    ret

sense_:
	mov al, 8
	mov cl, 1
	call cmd
.se:	call ready
;	jns .se
	in al, dx
	out 0xe1, al
	cmp al, 0x80
	ret

seek:	out  0x0b, al
Xseek:	call sense_
        jnz  Xseek
	mov  al, 0fh  
	mov  cl, 3    
cmdi:	call cmd      
Xcmdi    call sense_  
        jz Xcmdi
	ret

transfer:
	mov cl, 9
	call cmd
	inc byte [cylinder]
.transfer:	
	call ready
	jns .transfer
	ret
read:
    mov  AL, 16h ; Read DMA 2
	call seek
	mov al, 0xe6				; read normal data
	call transfer
	push esi
     mov  ESI, buffer*4
     mov  ECX, 512*18*2/4
     rep movsd
    pop  ESI
    ret

	times 512-2-($-$$) db 0
	dw 0xaa55
;	dd 0x44444444

write: mov  edi, buffer*4
       mov  ecx, 512*18*2/4
    rep movsd
    mov  al, 1ah ; write DMA 2 
    call seek                  
    mov  al, 0c5h              
    jmp short transfer              


flop:
	mov byte [cylinder], al
	DUP_
	mov dx, 0x3f2
	in al, dx
	out 0xe1, al
	test al, 0x10
	jnz .flo
	jmp spin
.flo	ret


readf:
	call flop				; ac-ac
	push edi
	mov edi, [esi+4]
	shl edi, 2
	call read
	pop edi
readf1: DROP
	inc eax
	add dword [esi], 0x1200
	ret


writef:
	call flop				; ac-ac
	push esi
	mov esi, [esi+4]
	shl esi, 2
	call write
	pop esi
	jmp short readf1

