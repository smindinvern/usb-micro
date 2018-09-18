include $(CURDIR)/devices/arm/cortex-m4.mk

DEVDIR:=$(CURDIR)/devices/atmel/sam4s

ENDIANNESS:=little
LINK_SCRIPT:=$(DEVDIR)/sam4s-linker.ld

FLAGS:=-mlittle-endian -DATSAM4S -DCHIP_NAME="ATSAM4S16C" -I$(DEVDIR)

CFLAGS+= $(FLAGS)
CXXFLAGS+= $(FLAGS)
LINKFLAGS+= -T $(LINK_SCRIPT)

SAM4S_OBJS:=interrupt_table.o interrupt_handlers.o atmel_init.o atmel_timer.o sam.o sam_usb.o atmel_usb.o 
OBJS+= $(patsubst %,$(DEVDIR)/%,$(SAM4S_OBJS))
undefine SAM4S_OBJS
