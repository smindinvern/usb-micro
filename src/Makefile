EXE_PREFIX:=arm-none-eabi-

CXX_EXE:=${EXE_PREFIX}g++
CC_EXE:=${EXE_PREFIX}gcc
LD_EXE:=${EXE_PREFIX}ld
AS_EXE:=${EXE_PREFIX}as
OBJCOPY_EXE:=${EXE_PREFIX}objcopy

PREFIX:=/usr/bin/

CXX:=${PREFIX}${CXX_EXE}
CC:=${PREFIX}${CC_EXE}
LD:=${PREFIX}${LD_EXE}
AS:=${PREFIX}${AS_EXE}
OBJCOPY:=${PREFIX}${OBJCOPY_EXE}

OBJS:=
include config.mk

ifdef DEBUG
	DEBUG:=-g
endif
ifdef OPT_LEVEL
	OPT_LEVEL:=-O$(OPT_LEVEL)
else
	OPT_LEVEL:=-O0
endif

ASFLAGS+= $(DEBUG)
CXXFLAGS+= -std=c++14 -pedantic -Wall -I$(CURDIR) -I$(CURDIR)/../include -nostdinc -nostdlib -ffreestanding -fstrict-aliasing $(OPT_LEVEL) -fshort-wchar -finline -fno-use-cxa-get-exception-ptr -fno-exceptions -fno-rtti $(DEBUG)
CFLAGS:=-I$(CURDIR) -I$(CURDIR)/../include -pedantic -nostdinc -nostdlib -ffreestanding -fstrict-aliasing -std=c99 $(OPT_LEVEL) -fshort-wchar -finline $(DEBUG)

OBJS+= div.o usb.o usbtmc.o usbtmc488.o mm.o main.o

bin: all
	$(OBJCOPY) -O binary UsbController.elf UsbController.bin

hex: all
	$(OBJCOPY) -O ihex UsbController.elf UsbController.hex
all: $(OBJS)
	$(CXX) $(CXXFLAGS) $(LINKFLAGS) $^ -o UsbController.elf

.PHONY: clean
clean:
	rm -f $(OBJS) *.elf *.bin *.hex
