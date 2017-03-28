all: prepare make_build make_copy make_image

prepare:
	mkdir ../bin
	mkdir ../bin/boot
	mkdir ../bin/disk
		
	
make_build:
	make -C bootloader/
	make -C tools/
	make -C kernel/

make_copy:
	cp bootloader/*.bin ../bin/boot
	cp kernel/*.cfg ../bin/disk
	cp kernel/*.bin ../bin/disk

make_image:
	dd if=../bin/boot/bios-boot.bin of=../bin/boot/bios.bin bs=512 count=1
	dd if=../bin/boot/bios-boot.bin of=../bin/disk/boot.bin bs=1 skip=512
	tools/fs_tools.tls of=../image/ZodiacOS.img bs=512 size=2880 boot=../bin/boot/bios.bin src=../bin/disk
clean:
	rm -r ../bin
	make -C bootloader/ clean
	make -C tools/ clean
	make -C kernel/ clean
	