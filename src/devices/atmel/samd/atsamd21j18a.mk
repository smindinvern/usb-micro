include $(CURDIR)/devices/arm/cortex-m0plus.mk

DEVDIR:=$(CURDIR)/devices/atmel/samd

ENDIANNESS:=little
LINK_SCRIPT:=$(DEVDIR)/samd21-linker.ld

FLAGS:=-mlittle-endian -DATSAMD21 -DCHIP_NAME="ATSAMD21J18A" -I$(DEVDIR) -I$(DEVDIR)/atsamd21 -include "atsamd21x18.hh"

CFLAGS+= $(FLAGS)
CXXFLAGS+= $(FLAGS)
LINKFLAGS+= -T $(LINK_SCRIPT)

SAMD_OBJS:=interrupt_table.o interrupt_handlers.o samd_init.o samd_i2c.o samd_usb.o 
OBJS+= $(patsubst %,$(DEVDIR)/%,$(SAMD_OBJS))
undefine SAMD_OBJS
