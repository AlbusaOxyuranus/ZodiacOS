BUILD_IMAGE_FOLDER=../image
ASM=fasm.x64
SCRIPT=ld
LOADERS_PATH=bootloader
BIOS_LOADER=bios-boot.asm
BIOS_LOADER_BIN=bios-boot.bin
BIOS=bios.bin
BOOT_LOADER_BIN=boot.bin
PATH_BIN=bin
PATH_KERNEL=kernel
FOLDER_DISK=disk
PATH_TOOLS=tools
FS_TOOLS=fs_tools
FS_TOOLS_EXT=$(FS_TOOLS).tls
IMAGE=ZodiacOS.img
CC=gcc

all: make_clear make_tools make_loader make_kernel make_img
	
make_loader:	
		$(ASM) $(LOADERS_PATH)/$(BIOS_LOADER) $(PATH_BIN)/$(BIOS_LOADER_BIN)

make_img: 
		 #dd if=/dev/zero of=$(PATH_BIN)/$(IMAGE) bs=512 count=2880
		 #dd if=$(PATH_BIN)/$(BIOS_LOADER_BIN) of=$(PATH_BIN)/$(IMAGE)
		 dd if=$(PATH_BIN)/$(BIOS_LOADER_BIN) of=$(PATH_BIN)/$(BIOS) bs=512 count=1
		 dd if=$(PATH_BIN)/$(BIOS_LOADER_BIN) of=$(FOLDER_DISK)/$(BOOT_LOADER_BIN) bs=1 skip=512
		 ./bin/$(FS_TOOLS_EXT) of=$(PATH_BIN)/$(IMAGE) bs=512 size=2880 boot=bin/$(BIOS) src=./$(FOLDER_DISK)

make_kernel: make_libs
			cp $(PATH_KERNEL)/*.cfg $(FOLDER_DISK)
			$(ASM) $(PATH_KERNEL)/startup.asm $(PATH_BIN)/startup.o
			$(CC) -c -m32 -ffreestanding -o $(PATH_BIN)/main.o $(PATH_KERNEL)/main.c
			$(SCRIPT) --oformat=binary -melf_i386 -T $(PATH_KERNEL)/script.ld -o $(FOLDER_DISK)/kernel.bin $(PATH_BIN)/startup.o $(PATH_BIN)/stdlib.o $(PATH_BIN)/main.o 

make_libs:	
			$(CC) -c -m32 -ffreestanding -o $(PATH_BIN)/stdlib.o $(PATH_KERNEL)/stdlib.c

make_tools:
			$(CC) -o $(PATH_BIN)/$(FS_TOOLS_EXT) $(PATH_TOOLS)/$(FS_TOOLS).c
make_clear:
			rm -f  $(PATH_BIN)/*.*
			rm -f  $(FOLDER_DISK)/*.*
			rm -f  $(FOLDER_DISK)/*/*.*


release: all
		cp $(PATH_BIN)/$(IMAGE) $(BUILD_IMAGE_FOLDER)