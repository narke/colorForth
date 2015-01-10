all:
	nasm -l color.lst -o color.bin color.asm
	cat color.bin icons.fnt cfdos4cf.blk > cf2012.img

ndisasm:
	# Disassemble the colorForth image
	# The image starts in 16 bit Real Mode   : cf2012.img.16.dasm
	ndisasm -e0x0000 -i -b 16 -k0x00B3,0x3FF4D cf2012.img > cf2012.img.16.dasm
	# Then switches to 32 bit Protected Mode : cf2012.img.32.dasm
	ndisasm -e0x00B3 -i -b 32 -k0x2F4D,0x3D000 cf2012.img > cf2012.img.32.dasm

clean:
	rm -f color.lst color.bin cf2012.img cf2012.img.16.dasm cf2012.img.32.dasm
