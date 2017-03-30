#include<string.h>

typedef struct {
	unsigned long long base;
	unsigned long long size;
} BootModuleInfo;

void kernel_main(char boot_disk_id, void *memory_map, BootModuleInfo *boot_module_list) {
	char *screen_buffer = (void*)0xB8000;
	char *msg = "Zodiac OS - HELLO!";
	unsigned int i = 24 * 80;
	while (*msg) {
		screen_buffer[i * 2] = *msg;
		msg++;
		i++;
	}
} 