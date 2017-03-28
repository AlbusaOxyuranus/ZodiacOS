all: prepare make_build make_copy clean

prepare:
	mkdir ../bin
	mkdir ../bin/boot
		
	
make_build:
	make -C bootloader/
make_copy:
	cp bootloader/*.bin ../bin/boot
clean:
	make -C bootloader/ clean
	rm -r ../bin