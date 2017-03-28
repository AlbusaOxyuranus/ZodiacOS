BUILD_IMAGE_FOLDER=../image
ASM=fasm.x64
LOADERS_PATH=bootloader
BIOS_LOADER=bios-boot.asm
BIOS_LOADER_BIN=bios-boot.bin
PATH_BIN=bin
IMAGE=ZodiacOS.img

release: all
		cp $(PATH_BIN)/$(IMAGE) $(BUILD_IMAGE_FOLDER)
all: make_clear make_loader make_img
	
make_loader:	
		$(ASM) $(LOADERS_PATH)/$(BIOS_LOADER) $(PATH_BIN)/$(BIOS_LOADER_BIN)

make_img: 
		 dd if=/dev/zero of=$(PATH_BIN)/$(IMAGE) bs=512 count=2880
		 dd if=$(PATH_BIN)/$(BIOS_LOADER_BIN) of=$(PATH_BIN)/$(IMAGE)

make_clear:
	rm -f  $(PATH_BIN)/*.*

