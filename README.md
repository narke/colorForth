# colorForth
Native colorForth for 32-bit PCs, at least compilable on Linux and runnable on both Bochs and Qemu.
It is adapted from http://sourceforge.net/projects/colorforth.

Running
-------

Get sources:

    $ git clone https://github.com/narke/colorForth.git

Compile:

	$ make

Run on Bochs:

	$ bochs -f bochs_colorforth

Or run on Qemu:

    $ qemu-system-i386 -boot a -fda cf2012.img

On Bochs:
![colorForth](https://raw.githubusercontent.com/narke/colorForth/master/screenshots/colorforth_bochs.png "colorForth on Bochs")

On Qemu:
![colorForth](https://raw.githubusercontent.com/narke/colorForth/master/screenshots/colorforth_qemu.png "colorForth on Qemu")
