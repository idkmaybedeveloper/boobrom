CC ?= cc
CFLAGS ?= -O2 -Wall -Wextra -Werror
PKG_CONFIG ?= pkg-config

LIBUSB_CFLAGS := $(shell $(PKG_CONFIG) --cflags libusb-1.0)
LIBUSB_LIBS := $(shell $(PKG_CONFIG) --libs libusb-1.0)

boobrom: boobrom.c
	$(CC) $(CFLAGS) $(LIBUSB_CFLAGS) -o $@ $< $(LIBUSB_LIBS)

clean:
	rm -f boobrom