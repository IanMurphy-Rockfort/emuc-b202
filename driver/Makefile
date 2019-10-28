Q                := @
PWD              ?= $(shell pwd)
KERNEL_SRC       ?= /lib/modules/`uname -r`/build
INCLUDE_DIR      ?= $(PWD)/include

CFILES           := main.c emuc_parse.c transceive.c
TARGET           := emuc2socketcan.ko
obj-m            := emuc2socketcan.o
emuc2socketcan-y := $(CFILES:.c=.o)
ccflags-y        := -I$(INCLUDE_DIR) -std=gnu99 -Wno-declaration-after-statement


default:
	$(MAKE) -C$(KERNEL_SRC) M=$(PWD) modules
	$(Q)cp $(TARGET) ../

clean:
	$(MAKE) -C$(KERNEL_SRC) M=$(PWD) clean
	$(Q)rm -f ../$(TARGET)