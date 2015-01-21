; gen.nasm	colorForth for 80x86 PC  generic graphics  2012 Oct 19  Howerd Oakford 
;

	align 4

frame:	dd 0x2000000-hp*vp*2		; assume 32 meg
displ:	dd 0xf0000000			; fujitsu
fore:	dd 0xf7de
xc:	dd 0
yc:	dd 0

rgb:
	ror eax, 8
	shr ax, 2
	ror eax, 6
	shr al, 3
	rol eax, 6+5
;	and eax, 0xf7de
	ret

white:
	DUP_
	mov eax, 0xffffff
color:
	call rgb
	mov dword [fore], eax
	DROP
	ret

fifof:
	DROP
graphic:
	ret

switch:
	push esi
	mov esi, dword [frame]
	push edi
	mov edi, dword [displ]
	mov ecx, hp*vp/2
	rep movsd
	pop edi
	pop esi
	jmp pause

clip:
    mov edi, [xy]
    mov  ecx, edi
    test cx, cx
    jns  Xclip1
    xor  ecx, ecx
Xclip1:  and  ecx, 0x0ffff
    mov  [yc] , ecx
    imul ecx, hp*2
    sar  edi, 16
    jns  Xclip2
    xor  edi, edi
Xclip2:  mov  [xc], edi
    lea  edi, [edi*2+ecx]
    add  edi, [frame]
    ret

bit16: lodsw
    xchg al, ah
    mov  ecx, 16
Xb16: shl  ax, 1
    jnc  Xb162
    mov  [edi], dx
Xb162:  add edi, byte 2
    NEXT Xb16
    ret

bit32: lodsw
    xchg al, ah
    mov  ecx, 16
b32: shl  eax, 1
    jnc  Xb322
    mov  [edi], dx
    mov  [edi+2], dx
    mov  [edi+hp*2], dx
    mov  [edi+hp*2+2], dx
Xb322:  add edi, byte 4
    NEXT b32
    ret

emit:
	call qcr
	push esi
	push edi
	push edx
	imul eax, byte 16*24/8
	lea esi, [icons+eax]
	call clip
	mov edx, [fore]
	mov ecx, 24
.em:	push ecx
	call bit16
	add edi, (hp-16)*2
	pop ecx
	NEXT .em
	pop edx
	pop edi
	pop esi
bl_:	DROP
space:	add dword [xy], iw*0x10000
	ret

emit2:
	push esi
	push edi
	push edx
	imul eax, byte 16*24/8
	lea esi, [icons+eax]
	call clip
	mov edx, [fore]
	mov ecx, 24
.em2:   push ecx
	call bit32
	add edi, (2*hp-16*2)*2
	pop ecx
	NEXT .em2
	pop edx
	pop edi
	pop esi
	add dword [xy], iw*0x10000*2
	DROP
	ret

text1:
	call white
	mov dword [lm], 3
	mov dword [rm], hc*iw
	jmp top

line:
	call clip
	mov ecx, [esi]
	shl ecx, 1
	sub edi, ecx
	mov ecx, eax
	mov eax, [fore]
	rep stosw
	inc dword [xy]
	DROP
	DROP
	ret

box:
	call clip
	cmp eax, vp+1
	js .bo1
	  mov eax, vp
.bo1	mov ecx, eax
	sub ecx, dword [yc]
	jng nob
	cmp dword [esi], hp+1
	js .bo2
	  mov dword [esi], hp
.bo2	mov eax, [xc]
	sub [esi], eax
	jng nob
	mov edx, hp
	sub edx, [esi]
	shl edx, 1
	mov eax, [fore]
.bo3	push ecx
	mov ecx, [esi]
	rep stosw
	add edi, edx
	pop ecx
	NEXT .bo3
nob:	DROP
	DROP
	ret


