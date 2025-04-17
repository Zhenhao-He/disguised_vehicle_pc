CROSS_COMPILE =

.PHONY : clean

AS              = $(CROSS_COMPILE)as
LD              = $(CROSS_COMPILE)ld
CC              = $(CROSS_COMPILE)gcc
CPP             = $(CROSS_COMPILE)g++
AR              = $(CROSS_COMPILE)ar
NM              = $(CROSS_COMPILE)nm
STRIP           = $(CROSS_COMPILE)strip
OBJCOPY         = $(CROSS_COMPILE)objcopy
OBJDUMP         = $(CROSS_COMPILE)OBJDUMP
RANLIB		= $(CROSS_COMPILE)ranlib
LDFLAGS		+= -lusbcan-4e

all: ex_loopback 

ex_loopback: ex_loopback.o 
	$(CC)  ex_loopback.o -o $@ $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -fPIC -c  $< -o $@ 

clean :
	rm -fr *.o  ex_loopback
