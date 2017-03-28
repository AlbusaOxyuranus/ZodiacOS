CFLAGS=-c -m32 -ffreestanding -fno-stack-protector
LFLAGS=--oformat=binary -melf_i386 -T script.ld

all: startup.o stdlib.o main.o script.ld 
	ld $(LFLAGS) -o kernel.bin startup.o stdlib.o main.o 
	
startup.o: startup.i386.asm
	fasm.x64 startup.i386.asm startup.o
	
stdlib.o: stdlib.c stdlib.h
	gcc $(CFLAGS) -o stdlib.o stdlib.c
main.o: main.c stdlib.h
	gcc $(CFLAGS) -o main.o main.c
	
clean:
	rm -v *.o kernel.bin 
