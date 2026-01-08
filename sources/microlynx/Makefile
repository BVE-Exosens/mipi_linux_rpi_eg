microlynx_basic-objs := \
	microlynx_basic_kmod.o \
	lib/libunio.o \
	lib/libunio_extras.o \
	lib/nb_timer.o \
	lib/gencp_common.o \
	lib/gencp_client.o

obj-m := microlynx_basic.o

BUILD ?= normal
EXTRA_FLAGS :=
ifeq ($(BUILD),debug)
    EXTRA_FLAGS := -DDEBUG -g -O0
endif


KVERSION = $(shell uname -r)

all:
	$(MAKE) -C /lib/modules/$(shell uname -r)/build \
	    M=$(PWD) EXTRA_CFLAGS="$(EXTRA_FLAGS)" modules

clean:
	$(MAKE) -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
