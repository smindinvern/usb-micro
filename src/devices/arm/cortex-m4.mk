CPU:=cortex-m4
THUMB:=1

CFLAGS+= -mcpu=cortex-m4 -mthumb -I$(CURDIR)/devices/arm
CXXFLAGS+= -mcpu=cortex-m4 -mthumb -I$(CURDIR)/devices/arm
ASFLAGS+= -mcpu=cortex-m4 -mthumb
LINKFLAGS=-Wl,-G,0

OBJS+= $(CURDIR)/devices/arm/arm.o

