CPU:=cortex-m0plus
THUMB:=1

CFLAGS+= -mcpu=cortex-m0plus -mthumb -I$(CURDIR)/devices/arm
CXXFLAGS+= -mcpu=cortex-m0plus -mthumb -I$(CURDIR)/devices/arm
ASFLAGS+= -mcpu=cortex-m0plus -mthumb
LINKFLAGS+= -Wl,-G,0 -Wl,-N

OBJS+= $(CURDIR)/devices/arm/arm.o $(CURDIR)/devices/arm/primitives.o
