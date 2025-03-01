KERNEL=/home/nam/bbb/kernelbuildscripts/KERNEL
TOOLCHAIN=/home/nam/bbb/kernelbuildscripts/dl/gcc-8.5.0-nolibc/arm-linux-gnueabi/bin/arm-linux-gnueabi-
ARMGCC=arm-linux-gnueabihf-
obj-m += Driver/lora-spi.o

all:
	make ARCH=arm CROSS_COMPILE=$(TOOLCHAIN) -C $(KERNEL) M=`pwd` modules
	${ARMGCC}gcc -o Test/test_app Test/test-app.c
clean:
	make ARCH=arm CROSS_COMPILE=$(TOOLCHAIN) -C $(KERNEL) M=`pwd` clean
	rm Test/test_app
