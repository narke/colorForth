readme.txt colorForth cf2012 V1.0 Howerd Oakford 2012 Oct 21

The plan is to convert this to run from address 0x10000 using the FAT32 file system on a USB drive...

colorForth is a dialect of Forth from Chuck Moore, the creator of Forth
It is a self-contained operating system and language which runs directly on 80x86 PC hardware using the BIOS

Feedback welcome : howerd@inventio.co.uk

Quick start guide - bochs
=================
Double click on "go.bat" 
This will re-assemble the bootable colorForth image  cf2012.img  and run it under  bochs
To run colorForth under bochs without re-assembling it, change to the bochs directory and double click  go.bat  there

Quick start guide - floppy disk version
=================
Put a blank, formatted floppy disk in the floppy drive and double click on "gofloppy.bat"
This will create a bootable colorForth floppy disk
Reboot your computer with the floppy in the drive
You may need to adjust your BIOS settings to get the floppy to boot -
please see http://www.inventio.co.uk/getbios.htm from http://www.inventio.co.uk/cfdos.htm for how to do this...
Note : this version requires a built-in floppy drive to run colorForth - it will not work from an external USB drive.
It also requires Windows XP - it will not work under Windows 7

Changes from cf2010 V4.1
========================
New version of nasm.exe that runs under Windows 7, updated some links, changed some filenames,
put the floppy disk option in gofloppy.bat, with a default of go.bat which runs colorForth under the latest version of bochs
The colorForth system is unchanged

Credits
=======
Ported to NASM format - please see http://sourceforge.net/projects/nasm/
Based on code by :
Chuck Moore (inventor, MASM)
Mark Slicker (ported to GNU Assembler)
Peter Appelman (ported to NASM with qwerty mods)
and others...   Thanks!!!

Details
=======
The 256K binary image cf2012.img is created from color.bin ( 12K ) concatenated with icons.fnt ( 6K ) and cfdos4cf.blk ( 238K )

color.bin       colorForth system, assembled from three .nasm assembler files
icons.fnt       character icons ( the font in blocks 12 to 17 of CFDOS4BLK )
cfdos4cf.blk    colorForth source ( blocks 18 to 255 from CFDOS4BLK )

Note that just as in the CFDOS4.BLK release, the last 4 blocks are not saved/restored
by the colorForth system - so don't use them! ( 14 sectors x 18K = 252K ).

The file color.bin is assembled from color.asm , boot.asm and gen.asm by batch file  go.bat , which also runs the output image under bochs
NASM is included here for this purpose

color.nasm      colorForth system
boot.nasm       floppy disk boot
gen.nasm        generic video display using a VESA call

These files are from Chuck Moore's 2001 Jul 22 release, which include modifications to
the floppy disk code by Terry Loveall and Kris Johnstone.
Thanks also to Mark Slicker (who ported colorForth to the GNU Assembler)
and Peter Appelman (who ported it to NASM with qwerty modifications),
Jeff Fox for many online explanations, and many others who provided colorForth programs.

I ( Howerd ) have made some small changes to merge with my CFDOS4.BLK release :
1. changed the version number in the boot sector
2. changed the assembler output length to 12K
3. changed the go.bat file to merge color.com, icons.fnt and cfdos4cf.blk
4. added  floppy.com to download automatically to the floppy disk
5. ported to NASM, merging the Mark Slicker GAS version with the Peter Appelman
   NASM version to give a binary file that is "nearly identical" to the original
   CFDOS4.blk file. The differences are certain opcodes assembled by NASM that are
   different to GAS, and some state variables that were saved in the original
   binary file.

"Modified from the original written by Chuck Moore. Thanks Chuck, nice job!"
I second that!

From the original readme.txt file :

Readme for colorForth on an Asus P2B-D motherboard.
 Modified from the orginal written by Chuck Moore. Thanks Chuck, nice job!
 07/19/01 Terry Loveall
 07/22/01 Kris Johnson     kristopher_d_johnson@yahoo.com

THIS SOFTWARE COMES WITH NO WARRANTY. USE AT YOUR OWN RISK.

The floppy modifications were derived from examination of the BIOS
 int 13 code. They are empirically determined and work on my (TL's)
 system.  Your Mileage May Vary.

TL's system is an Asus P2B-D dual PII 450 Mhz motherboard. It uses the
 440BX chipset, a Winbond W83977tf super I/O chip and PII4XE PCIset
 (Southbridge, I think).

The modified source for the floppy is located mainly in boot.asm. It
 consists of adding a write strobe to IO address 0E1h to all I/O to
 the floppy controller at IO address range 03F0h. This additional
 code expanded the boot sector to a size larger than 512 bytes, which
 necessitated a re-structuring of the boot sector code. The
 'call stop, call dma' code was moved from boot.asm to the 'start1'
 label in color.asm

Links
=====

For colorForth operation and programming see :

http://colorforth.com/                                              Chuck Moore' web site
http://www.inventio.co.uk/cfdos.htm                                 My own web site
http://www.greenarraychips.com/home/documents/greg/cf-elements.htm  Green Arrays, Inc. ArrayForth is very similar to colorForth
http://sourceforge.net/projects/colorforth/files/                   SourceForge repository
http://primarycolorforth.blogspot.com/                              John M Drake's blog
http://colorforthray.info/                                          Ray St. Marie's info site
http://c2.com/cgi/wiki?ColorForth                                   Wiki (interesting, but I don't agree with everything there ;-)
http://en.wikipedia.org/wiki/ColorForth                             Wikipedia

And of course there is always Google...

Enjoy!!!

Howerd  8^)

howerd@inventio.co.uk  http://www.inventio.co.uk
