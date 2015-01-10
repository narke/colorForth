; color.nasm colorForth for 80x86 PC  main load file  2012 Oct 21 Howerd Oakford
; Ported to NASM format.
; Based on code by :
; Chuck Moore (inventor, MASM)
; Mark Slicker (ported to GNU Assembler)
; Peter Appelman (ported to NASM with qwerty mods)
; and others...	Thanks!!!
; Feedback welcome to howerd@inventio.co.uk
;

%macro	NEXT 1
	dec ecx
	jnz %1
%endmacro

%macro	DUP_	0
	lea esi,[esi-4]
	mov [esi], eax
%endmacro

%macro	DROP	0
	lodsd
%endmacro

hp	equ 1024	; horizontal pixels
vp	equ 768		; vertical pixels
vesa 	equ 0x117
buffer	equ 604*256
icons	equ 12*256*4				; 0x3000

Gods	equ 0x28000*4				; 0xa0000
Godd	equ Gods-750*4
mains	equ Godd-1500*4
maind	equ mains-750*4

%include 'boot.nasm'

; org = 0x261

warm:
	DUP_		     ; 0x261
;	jmp short strtx1     ; 0x266
start1:
	mov [displ], ebp
strtx1:	call show0
	mov dword [trash], $buffer * 4
	mov dword [forths], ((forth1-forth0)/4)
	mov dword [macros], ((macro1-macro0)/4)
	mov eax, 18
	call load
	jmp accept

	align 4
me:	dd God					;		; 0x2a0
screen:	dd 0					; logo		; 0x2a4

round:	call unpause						; 0x2a8
God:	dd 0					; Gods-2*4
	call unpause
main:	dd 0					; mains-2*4  ; 0x2B6
	jmp short round

pause:
	DUP_
	push esi
	mov eax, [me]
	mov [eax], esp
	add eax, byte +4
	jmp eax

unpause:
	pop eax
	mov esp, [eax]
	mov [me], eax
	pop esi
	DROP
	ret	

act:
	mov edx, maind-4
	mov [edx], eax
	mov eax, mains-4
	pop dword [eax]
	sub eax, byte +4
	mov [eax], edx
	mov dword [main], eax
	DROP
	ret

show0:
	call show
	ret

show:
	pop dword [screen]
	DUP_
	xor eax, eax
	call act
.show:	call graphic
	call [screen]
	call switch
	inc eax
	jmp short .show

c_:
	mov esi, Godd+4
	ret

mark:				       ; 0x324
	mov ecx, [macros]
	mov [mk], ecx
	mov ecx, [forths]
	mov [mk+4], ecx
	mov ecx, [H]
	mov [mk+2*4], ecx
	ret

empty:
	mov ecx, [mk+2*4]
	mov [H], ecx
	mov ecx, [mk+4]
	mov [forths], ecx
	mov ecx, [mk]
	mov [macros], ecx
	mov dword [class], 0
	ret

mfind:
	mov ecx, [macros]
	push edi
	lea edi, [macro0-4+ecx*4]
	jmp short ffind

find:
	mov ecx, [forths]
	push edi
	lea edi, [forth0-4+ecx*4]
ffind:	std
	repne scasd
	cld
	pop edi
	ret

ex1:						; 0x39C
	dec dword [words]			; from keyboard
	jz .ex
	DROP
	jmp short ex1
.ex:	call find
	jnz abort1
	DROP
	jmp [ecx*4+forth2]

execute:
	mov dword [lit], alit
	DUP_
	mov eax, [-4+edi*4]
	and eax, byte -0x10
	call find
	jnz abort
	DROP
	jmp [forth2+ecx*4]

abort:
	mov [curs], edi
	shr edi, 10-2
	mov [blk], edi
abort1:	mov esp, Gods
	mov dword [spaces+3*4], forthd
	mov dword [spaces+4*4], qcompile
	mov dword [spaces+5*4], cnum
	mov dword [spaces+6*4], cshort
	mov eax, 57q
	call echo_
	jmp accept

sdefine:
	pop dword [adefine]
	ret
macro_:	
	call sdefine
macrod:
	mov ecx, [macros]
	inc dword [macros]
	lea ecx, [macro0+ecx*4]
	jmp short forthdd

forth:
	call sdefine
forthd:
	mov ecx, [forths]
	inc dword [forths]
	lea ecx, [forth0+ecx*4]
forthdd:
	mov edx, [-4+edi*4]
	and edx, byte -20q
	mov [ecx], edx
	mov edx, [H]
	mov [forth2-forth0+ecx], edx
	lea edx, [forth2-forth0+ecx]
	shr edx, 2
	mov [last], edx
	mov [list], esp
	mov dword [lit], adup
	test dword [class], -1
	jz .fthd
	jmp [class]
.fthd:	ret

cdrop:
	mov edx, [H]
	mov [list], edx
	mov byte [edx], 0xad			; lodsd
	inc dword [H]
	ret

qdup:
	mov edx, [H]
	dec edx
	cmp dword [list], edx
	jnz cdup
	cmp byte [edx], 0xad
	jnz cdup
	mov [H], edx
	ret
cdup:	mov edx, [H]
	mov dword [edx], 0x89fc768d
	mov byte [edx+4], 06
	add dword [H], byte +5
	ret

adup:
	DUP_
	ret

var1:
	DUP_
	mov eax, [4+forth0+ecx*4]
	ret
variable:
	call forthd
	mov dword [forth2-forth0+ecx], var1
	inc dword [forths]		; dummy entry for source address
	mov [4+ecx], edi
	call macrod
	mov dword [forth2-forth0+ecx], .var
	inc dword [macros]
	mov [4+ecx], edi
	inc edi
	ret
.var:	call [lit]
	mov eax, [4+macro0+ecx*4]
	jmp short cshrt

cnum:
	call [lit]
	mov eax, [edi*4]
	inc edi
	jmp short cshrt

cshort:
	call [lit]
	mov eax, [-4+edi*4]
	sar eax, 5
cshrt:  call literal
	DROP
	ret

alit:	mov dword [lit], adup
literal:
	call qdup
	mov edx, [list]
	mov [list+4], edx
	mov edx, [H]
	mov [list], edx
	mov byte [edx], 0xb8
	mov dword [1+edx], eax
	add dword [H], byte +5
	ret


qcompile:
	call [lit]
	mov eax, [-4+edi*4]
	and eax, byte -20q
	call mfind
	jnz .qcomp
	DROP
	jmp [macro2+ecx*4]
.qcomp: call find
	mov eax, [forth2+ecx*4]
qcom1:  jnz abort
call_:  mov edx, [H]
	mov dword [list], edx
	mov byte [edx], 0xe8
	add edx, byte +5
	sub eax, edx
	mov dword [-4+edx], eax
	mov [H], edx
	DROP
	ret

compile:
	call [lit]
	mov eax, [-4+edi*4]
	and eax, byte -20q
	call mfind
	mov eax, [macro2+ecx*4]
	jmp short qcom1	

short_:
	mov dword [lit], alit
	DUP_
	mov eax, [-4+edi*4]
	sar eax, 5
	ret

num:
	mov dword [lit], alit
	DUP_
	mov eax, [edi*4]
	inc edi
	ret

comma:
	mov ecx, 4
dcomma:	mov edx, [H]
	mov [edx], eax
	mov eax, [esi]				; drop
	lea edx, [ecx+edx]
	lea esi, [esi+4]
	mov [H], edx
	ret

comma1:
	mov ecx, 1
	jmp short dcomma

comma2:
	mov ecx, 2
	jmp short dcomma

comma3:
	mov ecx, 3
	jmp short dcomma

semi:
	mov edx, [H]
	sub edx, byte 5
	cmp dword [list], edx
	jnz .semi
	cmp byte [edx], 0xe8
	jnz .semi
	inc byte [edx]				; jmp
	ret
.semi:  mov byte [5+edx], 0xc3			; ret
	inc dword [H]
	ret

then:
	mov [list], esp
	mov edx, [H]
	sub edx, eax
	mov [-1+eax], dl
	DROP
	ret

begin:
	mov [list], esp
here:
	DUP_
	mov eax, [H]
	ret

qlit:
	mov edx, [H]
	lea edx, [edx-5]
	cmp dword [list], edx
	jnz .ql2
	cmp byte [edx], 0xb8
	jnz .ql2
	DUP_
	mov eax, [list+4]
	mov [list], eax
	mov eax, [1+edx]
	cmp dword [edx-5], 0x89fc768d		; dup
	jz .ql1
	mov [H], edx
	jmp cdrop
.ql1:	add dword [H], byte -10			; flag nz
	ret
.ql2:   xor edx, edx  				; flag z
	ret

less:
	cmp [esi], eax
	jl .less				; flag nz
	xor ecx, ecx   				; flag z
.less:	ret

qignore:
	test dword [-4+edi*4], -20q
	jnz nul
	pop edi
	pop edi
nul:	ret

jump:
	pop edx
	add edx, eax
	lea edx, [5+eax*4+edx]
	add edx, [-4+edx]
	DROP
	jmp edx	

load:
	shl eax, 10-2
	push edi
	mov edi, eax
	DROP
inter:
	mov edx, [edi*4]
	inc edi
	and edx, byte 17q
	call [spaces+edx*4]
	jmp short inter

	align 4

spaces:	dd qignore, execute, num

adefine: ; where definitions go, either in macrod (dictionary) or forthd
	dd forthd	; default, the forth dictionary
	dd qcompile, cnum, cshort, compile
	dd short_, nul, nul, nul
	dd variable, nul, nul, nul

lit:	dd adup
mk:	dd 0, 0, 0
H:	dd 0x40000*4
last:	dd 0
class:	dd 0
list:	dd 0, 0
macros: dd 0
forths: dd 0

macro0:
	dd (170q<<25)						; ';'
	dd ((((140q<<7)+146q)<<7)+142q)<<11			; dup
	dd ((((((177q<<7)+140q)<<7)+146q)<<7)+142q)<<4		; ?dup
	dd ((((((140q<<4)+1)<<4)+3)<<7)+142q)<<10		; drop
	dd ((((((2<<7)+144q)<<4)+4)<<4)+6)<<13			; then
	dd ((((((((143q<<4)+4)<<5)+25q)<<4)+7)<<4)+6)<<8	; begin
macro1:
	times 128 dd 0

forth0:
	dd ((((((143q<<4)+3)<<4)+3)<<4)+2)<<13			; boot
	dd ((((((27q<<4)+5)<<4)+1)<<5)+21q)<<14			; warm
	dd ((((((((142q<<4)+5)<<7)+146q)<<5)+20q)<<4)+4)<<5	; pause
	dd ((((((((21q<<4)+5)<<5)+22q)<<4)+1)<<4)+3)<<10	; MACRO
	dd ((((((((26q<<4)+3)<<4)+1)<<4)+2)<<7)+144q)<<8	; FORTH
	dd (22q << 27)						; c
	dd ((((((20q << 4)+2)<< 4)+3)<< 7)+142q)<< 12		; stop
	dd ((((((1 << 4)+4)<< 4)+5)<< 7)+140q)<< 13		; read
	dd ((((((((27q << 4)+1)<< 4)+7)<< 4)+2)<< 4)+4)<< 11	; write
	dd ((6 << 5)+22q)<< 23					; nc
	dd ((((((((((22q<<4)+3)<<5)+21q)<<5)+21q)<<4)+5)<<4)+6)<<5
								; comman(d)
	dd ((((((20q<<4)+4)<<4)+4)<<7)+164q)<<12		; seek
	dd ((((((((1<<4)+4)<<4)+5)<<7)+140q)<<5)+23q)<<8	; ready
	dd ((((5<<5)+22q)<<4)+2)<<19				; act
	dd ((((((20q<<7)+144q)<<4)+3)<<5)+27q)<<11		; show
	dd ((((((24q<<4)+3)<<4)+5)<<7)+140q)<<12		; load
	dd ((((((144q<<4)+4)<<4)+1)<<4)+4)<<13			; here
	dd ((((((177q<<5)+24q)<<4)+7)<<4)+2)<<12		; ?lit
	dd ((153q<<7)+176q)<<18					; 3,
	dd ((152q<<7)+176q)<<18					; 2,
	dd ((151q<<7)+176q)<<18					; 1,
	dd (176q<<25)						; ,
	dd ((((((24q<<4)+4)<<5)+20q)<<5)+20q)<<13		; less
	dd ((((((162q<<7)+146q)<<5)+21q)<<7)+142q)<<6		; jump
	dd ((((((((((5<<5)+22q)<<5)+22q)<<4)+4)<<7)+142q)<<4)+2)<<3
								; accept
	dd ((((142q<<4)+5)<<7)+140q)<<14			; pad
	dd ((((((((4<<4)+1)<<4)+5)<<5)+20q)<<4)+4)<<11		; erase
	dd ((((((22q<<4)+3)<<7)+142q)<<5)+23q)<<11		; copy
	dd ((((((21q<<4)+5)<<4)+1)<<7)+164q)<<12		; mark
	dd ((((((4<<5)+21q)<<7)+142q)<<4)+2)<<12		; empt
	dd ((((((4<<5)+21q)<<4)+7)<<4)+2)<<15			; emit
	dd ((((((((140q<<4)+7)<<5)+25q)<<4)+7)<<4)+2)<<8	; digit
	dd ((((((((152q<<4)+4)<<5)+21q)<<4)+7)<<4)+2)<<8	; 2emit
	dd (165q<<25)						; .
	dd ((144q<<7)+165q)<<18					; h.
	dd ((((144q<<7)+165q)<<4)+6)<<14			; h.n
	dd ((22q<<4)+1)<<23					; cr
	dd ((((((((20q<<7)+142q)<<4)+5)<<5)+22q)<<4)+4)<<7	; space
	dd ((((((140q<<4)+3)<<5)+27q)<<4)+6)<<12		; down
	dd ((((((4<<7)+140q)<<4)+7)<<4)+2)<<13			; edit
	dd (4<<28)						; e
	dd ((24q <<5)+21q)<<22					; lm
	dd ((1<<5)+21q)<<23					; rm
	dd ((((((((25q<<4)+1)<<4)+5)<<7)+142q)<<7)+144q)<<5	; graph(ic)
	dd ((((((2<<4)+4)<<7)+145q)<<4)+2)<<13			; text
	dd ((((((((164q<<4)+4)<<5)+23q)<<7)+143q)<<4)+3)<<5	; keybo(ard)
	dd ((((((140q<<4)+4)<<7)+143q)<<7)+146q)<<7		; debu(g)
	dd ((5<<4)+2)<<24					; at
	dd ((((173q<<4)+5)<<4)+2)<<17				; +at
	dd ((145q<<5)+23q)<<20					; xy
	dd ((((26q<<4)+3)<<7)+141q)<<16				; fov
	dd ((((((26q<<4)+7)<<5)+26q)<<4)+3)<<14			; fifo
	dd ((((143q<<4)+3)<<7)+145q)<<14			; box
	dd ((((((24q<<4)+7)<<4)+6)<<4)+4)<<15			; line
	dd ((((((((22q<<4)+3)<<5)+24q)<<4)+3)<<4)+1)<<10	; color
	dd ((((((((((3<<5)+22q)<<4)+2)<<4)+5)<<4)+6)<<4)+2)<<7	; octant
	dd ((20q<<7)+142q)<<20					; sp
	dd ((((((24q<<4)+5)<<5)+20q)<<4)+2)<<14			; last
	dd (((((((((146q<<4)+6)<<7)+142q)<<4)+5)<<5)+22q))<<5	; unpac(k)
;       dd (((142o shl 4+5)shl 5+22o)shl 7+164o)shl 9 ; pack

;	dd ((143q<<5|24q)<<7|164q)<<13				; blk
;	dd (((22q<<7|146q)<<4|1q)<<5|20q)<<11			; curs
;	dd (((27q<<4|3)<<4|1)<<7|140q)<<12			; word
;	dd ((4<<7|164q)<<4|2)<<17				; ekt
;	dd ((4<<7|164q)<<4|2)<<17				; ekt
forth1:
	times 512 dd 0

macro2:
	dd semi
	dd cdup
	dd qdup
	dd cdrop
	dd then
	dd begin

	times 128 dd 0

forth2:
	dd boot
	dd warm
	dd pause
	dd macro_
	dd forth
	dd c_
	dd stop
	dd readf
	dd writef
	dd nc_
	dd 0 ; cmdf
	dd 0 ; seekf
	dd 0 ; readyf
	dd act
	dd show
	dd load
	dd here
	dd qlit
	dd comma3
	dd comma2
	dd comma1
	dd comma
	dd less
	dd jump
	dd accept
	dd pad
	dd erase
	dd copy
	dd mark
	dd empty
	dd emit
	dd edig
	dd emit2
	dd dot10
	dd hdot
	dd hdotn
	dd cr
	dd space
	dd down
 	dd edit
	dd e
	dd lms
	dd rms
	dd graphic
	dd text1
	dd keyboard
	dd debug
	dd at_
	dd pat
	dd xy_
	dd fov_
	dd fifof
	dd box
	dd line
	dd color
	dd octant
	dd sps
	dd last_
	dd unpack

	times 512 dd 0

boot:
	mov al, 0xfe				; Reset
	out 0x64, al
	jmp short $

erase:
	mov ecx, eax
	shl ecx, 8
	DROP
	push edi
	mov edi, eax
	shl edi, 2+8
	xor eax, eax
	rep stosd
	pop edi
	DROP
	ret

copy:
	cmp eax, byte 12
	jc abort1
	mov edi, eax
	shl edi, 2+8
	push esi
	mov esi, [blk]
	shl esi, 2+8
	mov ecx, 256
	rep movsd
	pop esi
	mov [blk], eax
	DROP
	ret

iw	equ 16 + 6	; icon width
ih	equ 24 + 6	; icon height
hc	equ hp/iw	; horizontal columns
vc	equ vp/ih	; vertical columns 

debug:
	mov dword [xy], 3*0x10000+(vc-2)*ih+3
	DUP_
	mov eax, [God]
	push dword [eax]
	call dot
	DUP_
	pop eax
	call dot
	DUP_
	mov eax, [main]
	call dot
	DUP_
	mov eax, esi
	jmp dot

	align 4
xy:	dd 3*0x10000+3
lm:	dd 3
rm:	dd hc*iw				; 1012
xycr:	dd 0
fov:	dd 10*(2*vp+vp/2)

nc_:
	DUP_
	mov eax, (nc-start)/4
	ret

xy_:
	DUP_
	mov eax, (xy-start)/4
	ret

fov_:
	DUP_
	mov eax, (fov-start)/4
	ret

sps:
	DUP_
	mov eax, (spaces-start)/4
	ret

last_:
	DUP_
	mov eax, (last-start)/4
	ret

%include 'gen.nasm'

yellow	equ 0xffff00

cyan:
	DUP_
	mov eax, 0xffff
	jmp color


magenta:
	DUP_
	mov eax, 0xff00ff
	jmp color

silver:
	DUP_
	mov eax, 0xc0c0c0
	jmp color

blue:
	DUP_
	mov eax, 0x4040ff
	jmp color

red:
	DUP_
	mov eax, 0xff0000
	jmp color

green:
	DUP_
	mov eax, 0x8000ff00
	jmp color

history:
	times 11 db 0

echo_:
	push esi
	mov ecx, 11-1
	lea edi, [history]
	lea esi, [edi+1]
	rep movsb
	pop esi
	mov byte [history+11-1], al
	DROP
	ret

right:
	DUP_
	mov ecx, 11
	lea edi, [history]
	xor eax, eax
	rep stosb
	DROP
	ret

down:
	DUP_
	xor edx, edx
	mov ecx, ih
	div ecx
	mov eax, edx
	add edx, 3*0x10000+0x8000-ih+3
	mov dword [xy], edx
zero:	test eax, eax
	mov eax, 0
	jnz .dw
	  inc eax
.dw:	ret

blank:
	DUP_
	xor eax, eax
	mov dword [xy], eax
	call color
	DUP_
	mov eax, hp
	DUP_
	mov eax, vp
	jmp box

top:
	mov ecx, dword [lm]
	shl ecx, 16
	add ecx, byte +3
	mov dword [xy], ecx
	mov dword [xycr], ecx
	ret

qcr:
	mov cx, word [xy+2]
	cmp cx, word [rm]
	js crr
cr:
	mov ecx, [lm]
	shl ecx, 16
	mov cx, word [xy]
	add ecx, byte ih
	mov dword [xy], ecx
crr:	ret

lms:
	mov dword [lm], eax
	DROP
	ret

rms:
	mov dword [rm], eax
	DROP
	ret

at_:
	mov word [xy], ax
	DROP
	mov word [xy+2], ax
	DROP
	ret

pat:
	add word [xy], ax
	DROP
	add word [xy+2], ax
	DROP
	ret

octant:
	DUP_
	mov eax, 0x43
	mov edx, [4+esi]
	test edx, edx
	jns .oc1
	  neg edx
	  mov [4+esi], edx
	  xor al, 1
.oc1:	cmp edx, [esi]
	jns .oc2
	  xor al, 4
.oc2:	ret

eight:
	add edi , byte 0x0c
	call four
	call space
	sub edi , byte 0x10
four:
	mov ecx, 4
four1:
	push ecx
	DUP_
	xor eax, eax
	mov al, [edi+4]
	inc edi
	call emit
	pop ecx
	NEXT four1
	ret

stack:
	mov edi, Godd-4
.st1:	mov edx, [God]
	cmp [edx], edi
	jnc .st2
	  DUP_
	  mov eax, [edi]
	  sub edi, byte +4
	  call qdot
	  jmp short .st1
.st2:	ret

keyboard: call text1
    mov  edi, [board]
    DUP_
    mov  eax, [keyc]
    call color
    ; right margin is 9 icons over from left margin
    mov dword [rm], hc*iw
    mov dword [lm], hp - (9 * iw) + 3
    mov dword [xy], ((hp - (9 * iw) + 3)*0x10000 ) + ( vp - (4 * ih) + 3 )
    call eight
    call eight
    call eight
    call cr   
    add dword [xy], 4*iw*10000h
    mov  EDI, [shift]
    add  EDI, byte 4*4-4
    mov  ECX, 3
    call four1
    mov  dword [lm], 3
    mov  word [xy+2], 3
    call stack
    mov  word [xy+2], hp - (11 + 9) * iw + 3
    lea  EDI, [history-4]
    mov  ECX, 11
    jmp  four1

alpha db 15o, 12o,  1 , 14o
      db 24o,  2 ,  6 , 10o
      db 23o, 11o, 17o, 21o
      db 22o, 13o, 16o,  7
      db  5 ,  3 ,  4 , 26o
      db 27o, 44o, 25o, 20o
graphics db 31o, 32o, 33o,  0 
         db 34o, 35o, 36o, 30o
         db 37o, 40o, 41o, 57o
         db 51o, 50o, 52o, 54o ; : ; ! @
         db 46o, 42o, 45o, 56o ; Z J . ,
         db 55o, 47o, 53o, 43o ; * / + -
numbers db 31o, 32o, 33o,  0
        db 34o, 35o, 36o, 30o
        db 37o, 40o, 41o,  0
        db  0,   0 ,  0 ,  0
        db  0,   0 ,  0 ,  0
        db  0,   0 ,  0 ,  0
octals db 31o, 32o, 33o,  0
       db 34o, 35o, 36o, 30o
       db 37o, 40o, 41o,  0
       db  0 ,  5 , 23o, 12o
       db  0 , 20o,  4 , 16o
       db  0 ,  0 ,  0 ,  0
letter: cmp  AL, 4
	js  xLetter
        mov  EDX, [board]
        mov al,[edx+eax]
xLetter: ret

keys db 16, 17, 18, 19,  0,  0,  4,  5 ; 20
     db  6,  7,  0,  0,  0,  0, 20, 21
     db 22, 23,  0,  0,  8,  9, 10, 11 ; 40
     db  0,  0,  0,  0, 24, 25, 26, 27
     db  0,  1, 12, 13, 14, 15,  0,  0 ; 60 N
     db  3,  2 ; alt space
key: DUP_
    xor  EAX, EAX
Xkey:     call pause
        in   AL, 144o
        test AL, 1
        jz   Xkey
    in   AL, 140o
    test AL, 360o
    jz   Xkey
    cmp  AL, 72o
    jnc  Xkey
    mov  AL, [keys-20o+EAX]
    ret

ALIGN 4

graph0 dd nul0, nul0, nul0, alph0
       db  0 ,  0 ,  5 , 0 ;     a
graph1 dd word0, x, lj, alph
       db 25o, 45o,  5 , 0 ; x . a
alpha0 dd nul0, nul0, number, star0
       db  0 , 41o, 55o, 0 ;   9 *
alpha1 dd word0, x, lj, graph
       db 25o, 45o, 55o, 0 ; x . *
 numb0 dd nul0, minus, alphn, octal
       db 43o,  5 , 16o, 0 ; - a f
 numb1 dd number0, xn, endn, number0
       db 25o, 45o,  0 , 0 ; x .

board   dd alpha-4
shift   dd alpha0
base    dd 10
current dd decimal
keyc    dd yellow
chars   dd 1
aword   dd ex1
anumber dd nul
words   dd 1

nul0:
	DROP
	jmp short accept2
accept:
	mov dword [shift], alpha0
	lea edi, [alpha-4]
;	xor edi, edi
accept1:
	mov dword [board], edi
accept2:
	call key
	cmp al, 4
	jns near first
;	neg al
	mov edx, [shift]
	jmp [edx+eax*4]

bits_:	db 28

pack0:	add eax, byte 120q
	mov cl, 7
	jmp short pack1
pack:
	cmp al, 20q
	jnc pack0
	mov cl, 4
	test al, 10q
	jz pack1
	inc ecx
	xor al, 30q
pack1:	mov edx, eax
	mov ch, cl
.pa2:	cmp byte [bits_], cl
	jnc .pa3
	shr al, 1
	jc full
	dec cl
	jmp short .pa2
.pa3:	shl dword [esi], cl
	xor [esi], eax
	sub byte [bits_], cl
	ret

lj0:
	mov cl, byte [bits_]
	add cl, 4
	shl dword [esi], cl
	ret

lj:
	call lj0
	DROP
	ret

full:
	call lj0
	inc dword [words]
	mov byte [bits_], 28
	sub byte [bits_], ch
	mov eax, edx
	DUP_
	ret

x:
	call right
	mov eax, [words]
	lea esi, [eax*4+esi]
	DROP
	jmp accept

word_:
	call right
	mov dword [words], 1
	mov dword [chars], 1
	DUP_
	mov dword [esi], 0
	mov byte [bits_], 28
word1:
	call letter
	jns .wo
;	neg al
	mov edx, [shift]
	jmp dword [edx+eax*4]
.wo:	test al, al
	jz word0
	DUP_
	call echo_
	call pack
	inc dword [chars]
word0: 	DROP
	call key
	jmp short word1

decimal:
	mov dword [base], 10
	mov dword [shift], numb0
	mov dword [board], (numbers - 4)
	ret

hex:
	mov dword [base], 16
	mov dword [shift], numb0
	mov dword [board], (octals - 4)
	ret

octal:
	xor dword [current], (decimal-start) ^ (hex-start)
	xor byte [numb0+18],(0x21 ^ 0x0E ) ;  0x2f
	call [current]
	jmp short number0

xn:
	DROP
	DROP
	jmp accept

digit:
	db 14, 10,  0,  0
	db  0,  0, 12,  0,  0,  0, 15,  0
	db 13,  0,  0, 11,  0,  0,  0,  0
	db  0,  1,  2,  3,  4,  5,  6,  7
	db  8,  9
	
sign:	db 0

minus:
	mov byte [sign], al
	jmp short number2

number0:
	DROP
	jmp short number3
number:
	call [current]
	mov byte [sign], 0
	xor eax, eax
number3:
	call key
	call letter
	jns .nu1
;	neg al
	mov edx, [shift]
	jmp dword [edx+eax*4]
.nu1:	test al, al
	jz number0
	mov al, [digit-4+eax]
	test byte [sign], 37q
	jz .nu2
	neg eax
.nu2:	mov edx, [esi]
	imul edx, dword [base]
	add edx, eax
	mov [esi], edx
number2:
	DROP
	mov dword [shift], numb1
	jmp short number3

endn:
	DROP
	call [anumber]
	jmp accept

alphn:  DROP
alph0:  mov dword [shift] , alpha0 
    lea  edi , [alpha - 4]
    jmp short Xstar0
star0:  mov dword [shift] , graph0
    lea  edi , [graphics - 4]
Xstar0:  DROP
    jmp accept1

alph:   mov dword [shift] , alpha1
    lea  edi , [alpha - 4]
    jmp short Xgraph
graph:  mov dword [shift] , graph1
    lea  edi , [graphics - 4]
Xgraph:  mov dword [board] , edi
    jmp word0

first:
	add dword [shift], byte 4*4+4
	call word_
	call [aword]
	jmp accept

hicon:
	db 30q, 31q, 32q, 33q, 34q, 35q, 36q, 37q
	db 40q, 41q,  5 , 23q, 12q, 20q,  4 , 16q 

edig1:
	DUP_
edig:
	push ecx
	mov al, [eax+hicon]
	call emit
	pop ecx
	ret

odig:
	rol eax, 4
	DUP_
	and eax, byte 0xf
	ret

hdotn:
	mov edx, eax
	neg eax
	lea ecx, [32+eax*4]
	DROP
	rol eax, cl
	mov ecx, edx
	jmp short hdot1
hdot:
	mov ecx, 8
hdot1:	call odig
	call edig
	NEXT hdot1
	DROP
	ret

dot:
	mov ecx, 7
.do1:	call odig
	jnz .do4
	DROP
	NEXT .do1
	inc ecx
.do2:	call odig
.do3:	call edig
	NEXT .do2
	call space
	DROP
	ret
.do4:	inc ecx
	jmp short .do3

qdot:
	cmp dword [base], byte 10
	jnz dot
dot10:
	mov edx, eax
	test edx, edx
	jns .dt1
	  neg edx
	  DUP_
	  mov eax, 43q
	  call emit
.dt1:	mov ecx, 8
.dt2:	mov eax, edx
	xor edx, edx
	div dword [ecx*4+tens]
	test eax, eax
	jnz d_1
	dec ecx
	jns .dt2
	jmp short d_2
d_0:	mov eax, edx
	xor edx, edx
	div dword [ecx*4+tens]
d_1:	call edig1
	dec ecx
	jns d_0
d_2:	mov eax, edx
	call edig1
	call space
	DROP
	ret

unpack:
	DUP_
	test eax, eax
	js .up1
	shl dword [esi], 4
	rol eax, 4
	and eax, byte 7
	ret
.up1:	shl eax, 1
	js  .up2
	shl dword [esi], 5
	rol eax, 4
	and eax, byte 7
	xor al, 10q
	ret
.up2:	shl dword [esi], 7
	rol eax, 6
	and eax, byte 77q
	sub al, 20q
	ret

qring:
	DUP_
	inc dword [esi]
	cmp dword [curs], edi
	jnz .ri1
	  mov dword [curs], eax
.ri1:	cmp eax, dword [curs]
	jz ring
	jns .ri2
	  mov dword [pcad], edi
.ri2:	DROP
	ret

ring:
	mov dword [cad], edi
	sub dword [xy], iw*0x10000		; bksp
	DUP_
	mov eax, 0x0e04000
	call color
	mov eax, 60q
	mov cx, word [xy+2]
	cmp cx, word [rm]
	js .ri1
	  call emit
	  sub dword [xy], iw*0x10000
	  ret
.ri1:	jmp emit

rw:
	mov cx, word [xy+2]
	cmp cx, word [lm]
	jz .rw1
	 call cr
.rw1:	call red
	jmp type_

gw:
	call green
	jmp type_
mw:
	call cyan
	jmp type_
ww:
	DUP_
	mov eax, yellow
	call color
	jmp short type_

type0:
	sub dword [xy], iw*0x10000		; call bspcr
	test dword [-4+edi*4], -20q
	jnz type_
	  dec edi
	  mov dword [lcad], edi
	  call space
	  call qring
	  pop edx				; end of block
	  DROP
	  jmp keyboard

cap:
	call white
	DUP_
	mov eax, [-4+edi*4]
	and eax, byte -0x10
	call unpack
	add al, 48
	call emit
	jmp short type2

caps:
	call white
	DUP_
	mov eax, [-4+edi*4]
	and eax, byte -0x10
.ca1:	  call unpack
	  jz type3
	  add al, 48
	  call emit
	  jmp short .ca1

text:
	call white
type_:
	DUP_
	mov eax, [-4+edi*4]
	and eax, byte -0x10
type2:	call unpack
	jz type3
	call emit
	jmp short type2
type3:	call space
	DROP
	DROP
	ret

gsw:
	mov edx, [-4+edi*4]
	sar edx, 5
	jmp short gnw1

var:
	call magenta
	call type_
gnw:	mov edx, [edi*4]
	inc edi
gnw1:	DUP_
	mov eax, 0x0f800			; green
	cmp dword [bas], dot10
	jz nw2
	mov eax, 0xc000				; dark green
	jmp short nw2

sw:
	mov edx, [-4+edi*4]
	sar edx, 5
	jmp short nw1

nw:
	mov edx, [edi*4]
	inc edi
nw1:	DUP_
	mov eax, yellow
	cmp dword [bas], dot10
	jz nw2
	mov eax, 0xc0c000			; dark yellow
nw2:	call color
	DUP_
	mov eax, edx
	jmp [bas]

refresh:
	call show
	call blank
	call text1
	DUP_					; counter
	mov eax, [lcad]
	mov dword [cad], eax			; for curs beyond end
	xor eax, eax
	mov edi, [blk]
	shl edi, 10-2
	mov dword [pcad], edi			; for curs=0
ref1:	test dword [edi*4], 0xf
	jz .re1
	call qring
.re1:	mov edx, [edi*4]
	inc edi
	mov dword [bas], dot10
	test dl, 20q
	jz .re2
	mov dword [bas], dot
.re2:	and edx, byte 0xf
	call [edx*4+display]
	jmp short ref1

	align 4
display:
	dd type0, ww, nw, rw
	dd gw, gnw, gsw, mw
	dd sw, text, cap, caps
	dd var, nul, nul, nul

tens:
	dd 10, 100, 1000, 10000, 100000, 1000000
	dd 10000000, 100000000, 1000000000

bas:	dd dot10
blk:	dd 18
curs:	dd 0
cad:	dd 0
pcad:	dd 0
lcad:	dd 0
trash:	dd buffer*4

ekeys:	dd nul, del, eout, destack
	dd act1, act3, act4, shadow 
	dd mcur, mmcur, ppcur, pcur 
	dd mblk, actv, act7, pblk   
	dd nul, act11, act10, act9  
	dd nul, nul, nul, nul       
ekbd0:	dd nul, nul, nul, nul
	db 0x15, 0x25, 0x07, 0x00
ekbd:	db 0x0f, 0x01, 0x0d, 0x2d
	db 0x0c, 0x16, 0x10, 0x01 ; l  u  d  r
	db 0x23, 0x09, 0x0a, 0x2b
	db 0x00, 0x38, 0x3a, 0x02
	db 0x00, 0x00, 0x00, 0x00
	db 0x00, 0x00, 0x00, 0x00

actc:	dd yellow, 0, 0xff0000, 0xc000, 0, 0, 0xffff
	dd 0, 0xffffff, 0xffffff, 0xffffff, 0x8080ff
vector: dd 0
action: db 1

act1:	mov al, 1
	jmp short actt
act3:	mov al, 3
	jmp short actt
act4:	mov al, 4
	jmp short actt
act9:	mov al, 9
	jmp short actt
act10:	mov al, 10
	jmp short actt
act11:	mov al, 11
	jmp short actt
act7:	mov al, 7
actt:	mov byte [action], al
	mov eax, [actc-4+eax*4]
	mov dword [aword], insert
actn:	mov dword [keyc], eax
	pop eax
	DROP
	jmp accept

actv:	mov byte [action], 12
	mov eax, 0xff00ff			; magenta
	mov dword [aword], .act1
	jmp short actn

.act1:	DUP_
	xor eax, eax
	inc dword [words]
	jmp near insert

mcur:	dec dword [curs]
	jns pcr1
pcur:	inc dword [curs]
pcr1:	ret

mmcur:	sub dword [curs], byte 8
	jns mmc1
hcur:	mov dword [curs], 0
mmc1:	ret

ppcur:	add dword [curs], byte 8
	ret

pblk:	add dword [blk], byte 2
	add dword [esi], byte 2
	ret

mblk:	cmp dword [blk], byte 20
	js .mb
	sub dword [blk], byte 2
	sub dword [esi], byte 2
.mb:	ret

shadow:
	xor dword [blk], byte 1
	xor dword [esi], byte 1
	ret

e0:	DROP
	jmp short e_1

edit:	mov dword [blk], eax
	DROP
e:	DUP_
	mov eax, [blk]
	mov dword [anumber], format
	mov byte [alpha0+4*4], 45q		; .
	mov dword [alpha0+4], e0
	call refresh
e_1:	mov dword [shift], ekbd0
	mov dword [board], (ekbd - 4)
	mov dword [keyc], yellow
.eed:	call key
	call [eax*4+ekeys]
	DROP
	jmp short .eed

eout:	pop eax
	DROP
	DROP
	mov dword [aword], ex1
	mov dword [anumber], nul
	mov byte [alpha0+4*4], 0
	mov dword [alpha0+4], nul0
	mov dword [keyc], yellow
	jmp accept

destack:
	mov edx, [trash]
	cmp edx, buffer*4
	jnz .de1
	ret
.de1:	sub edx, byte 2*4
	mov ecx, [edx+1*4]
	mov dword [words], ecx
.de2:	DUP_
	mov eax, [edx]
	sub edx, byte 1*4
	NEXT .de2
	add edx, byte 1*4
	mov dword [trash], edx

insert0:
	mov ecx, [lcad]				; room available?
	add ecx, [words]
	xor ecx, [lcad]
	and ecx, -0x100
	jz insert1
	mov ecx, [words]			; no
.in1:	DROP
	NEXT .in1
	ret
insert1:
	push esi
	mov esi, [lcad]
	mov ecx, esi
	dec esi
	mov edi, esi
	add edi, [words]
	shl edi, 2
	sub ecx, [cad]
	js .in2
	shl esi, 2
	std 
	rep movsd
	cld
.in2:	pop esi
	shr edi, 2
	inc edi
	mov dword [curs], edi			; like abort
	mov ecx, [words]
.in3:	dec edi
	mov dword [edi*4], eax
	DROP					; requires cld
	NEXT .in3
	ret

insert:
	call insert0
	mov cl, [action]
	xor byte [edi*4], cl
	jmp accept

format:
	test byte [action], 12q			; ignore 3 and 9
	jz .fo1
	DROP
	ret
.fo1:	mov edx, eax
	and edx, 0xfc000000
	jz .fo2
	cmp edx, 0xfc000000
	jnz format2
.fo2:	shl eax, 5
	xor al, 2				; 6
	cmp byte [action], 4
	jz .fo3
	xor al, 13q				; 8
.fo3:	cmp dword [base], byte 10
	jz .fo4
	xor al, 20q
.fo4:	mov dword [words], 1
	jmp short insert

format2:
	DUP_
	mov eax, 1				; 5
	cmp byte [action], 4
	jz .fo5
	mov al, 3				; 2
.fo5:	cmp dword [base], byte 10
	jz .fo6
	xor al, 20q
.fo6:	xchg eax, [esi]
	mov dword [words], 2
	jmp insert

del:
	call enstack
	mov edi, [pcad]
	mov ecx, [lcad]
	sub ecx, edi
	shl edi, 2
	push esi
	mov esi, [cad]
	shl esi, 2
	rep movsd
	pop esi
	jmp mcur

enstack:
	DUP_
	mov eax, [cad]
	sub eax, [pcad]
	jz ens
	mov ecx, eax
	xchg eax, edx
	push esi
	mov esi, [cad]
	lea esi, [esi*4-4]
	mov edi, [trash]
.en:	std
	lodsd
	cld
	stosd
	NEXT .en
	xchg eax, edx
	stosd
	mov dword [trash], edi
	pop esi
ens:	DROP
	ret

pad:
	pop edx
	mov dword [vector], edx
	add edx, 28*5
	mov dword [board], edx
	sub edx, byte 4*4
	mov dword [shift], edx
.pa1:	call key
	mov edx, [vector]
	add edx, eax
	lea edx, [5+eax*4+edx]
	add edx, [-4+edx]
	DROP
	call edx
	jmp short .pa1


	times ((0x3000 - 0x60) - ($-$$)) db 0 ; leave 4 x 32 character lines at the end

; version: 
 	db 'cf2012 1V0 2012Oct21 Chuck Moore'
 	db 'colorForth Explorer NASM edition'
	db 'Howerd Oakford cf@inventio.co.uk'

