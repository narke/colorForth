all:
	nasm -l cf2019.lst -o cf2019.img cf2019.asm

ndisasm:
	echo "start address  avoid n bytes from address  filename     output filename"
	#ndisasm -b 32    -e 0x00000712     -k 0x00010000,0x00000FAD cf2019.img > cf2019.dasm
	echo "start address   filename     output filename"
	#ndisasm -b 32 -e 0x00000F2D    cf2019.img > cf2019.dasm
	#ndisasm -b 32 -e 0x00000712   cf2019.img > cf2019.dasm
	ndisasm -b 32 -e 0x00000000   cf2019.img > cf2019.dasm

qemu:
	qemu-system-i386 -drive format=raw,file=cf2019.img,if=floppy

bochs:
	bochs -f cf2019.bxrc

clean:
	rm -f cf2019.lst cf2019.img cf2019.dasm
