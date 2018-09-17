EXE_PREFIX:=

CXX_EXE:=${EXE_PREFIX}g++
CC_EXE:=${EXE_PREFIX}gcc
LD_EXE:=${EXE_PREFIX}ld
AS_EXE:=${EXE_PREFIX}as
OBJCOPY_EXE:=${EXE_PREFIX}objcopy

PREFIX:=

CXX:=${PREFIX}${CXX_EXE}
CC:=${PREFIX}${CC_EXE}
LD:=${PREFIX}${LD_EXE}
AS:=${PREFIX}${AS_EXE}
OBJCOPY:=${PREFIX}${OBJCOPY_EXE}

CPU:=
ENDIANNESS:=
THUMB:=
DEBUG:=

ifdef CPU
	CPU:=-mcpu=$(CPU)
endif
ifdef ENDIANNESS
	ENDIANNESS:=-m${ENDIANNESS}-endian
endif
ifdef THUMB
	THUMB:=-mthumb
endif
ifdef DEBUG
	DEBUG:=-g
endif

LINK_SCRIPT:=


ASFLAGS:=$(CPU) $(THUMB) $(ENDIANNESS) $(DEBUG)
CXXFLAGS:=-std=c++14 -pedantic -Wall -I. -Iatmel -Iarm -Iinclude $(ENDIANNESS) -nostdinc -nostdlib -ffreestanding -fstrict-aliasing -O3 -fshort-wchar $(CPU) $(THUMB) -finline -fno-use-cxa-get-exception-ptr -fno-exceptions -fno-rtti $(DEBUG)
CFLAGS:=-I. -Iatmel -Iarm -pedantic $(ENDIANNESS) -nostdinc -nostdlib -ffreestanding -fstrict-aliasing -std=c99 -O3 -fshort-wchar $(CPU) $(THUMB) -finline $(DEBUG)
LINKFLAGS:=-T $(LINK_SCRIPT) -Wl,-G,0 -Wl,-N

OBJS:=arm/interrupt_table.o arm/interrupt_handlers.o arm/arm.o atmel/microchip_init.o usb.o atmel/samd_usb.o usbtmc.o usbtmc488.o mm.o main.o end.o

bin: all
	$(OBJCOPY) -O binary UsbController.elf UsbController.bin

hex: all
	$(OBJCOPY) -O ihex UsbController.elf UsbController.hex
all: $(OBJS)
	$(CXX) $(CXXFLAGS) $(LINKFLAGS) $(OBJS) -o UsbController.elf

.PHONY: clean
clean:
	rm -f *.elf *.o atmel/*.o arm/*.o *.bin
