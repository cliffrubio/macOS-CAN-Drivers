# SPDX-License-Identifier: GPL-2.0-or-later
# PeakUSB: macOS user-space, PCAN-Basic compatible driver for PEAK adapters.
#
#   make                 build the library and the command line tools
#   make install         install into $(PREFIX) (default /usr/local)
#   make install-compat  additionally install a libPCBUSB.dylib symlink, for
#                        apps that hard-code PEAK's library name (SavvyCAN)
#   make ARCHS="arm64 x86_64"   build universal (requires libusb for both archs)
#   make clean

CC      ?= cc
PREFIX  ?= /usr/local
DESTDIR ?=
BUILD   ?= build
# Set MACOSX_DEPLOYMENT_TARGET in the environment to target an older macOS;
# clang picks it up on its own. Left unset, the build targets the host.

LIBVER  := 1.0.0
DYLIB   := libPeakUSB.1.dylib
SYMLINK := libPeakUSB.dylib
COMPAT  := libPCBUSB.dylib
HEADER  := include/PeakUSB.h
TOOLS   := pcan_ls pcan_recv pcan_send

# --- libusb: pkg-config first, Homebrew as a fallback ------------------------
LIBUSB_CFLAGS := $(shell pkg-config --cflags libusb-1.0 2>/dev/null)
LIBUSB_LIBS   := $(shell pkg-config --libs libusb-1.0 2>/dev/null)
ifeq ($(strip $(LIBUSB_LIBS)),)
  BREW_LIBUSB := $(shell brew --prefix libusb 2>/dev/null)
  ifneq ($(strip $(BREW_LIBUSB)),)
    LIBUSB_CFLAGS := -I$(BREW_LIBUSB)/include/libusb-1.0
    LIBUSB_LIBS   := -L$(BREW_LIBUSB)/lib -lusb-1.0
  endif
endif
ifeq ($(strip $(LIBUSB_LIBS)),)
  $(error libusb-1.0 not found. Install it with: brew install libusb)
endif

ARCH_FLAGS := $(foreach a,$(ARCHS),-arch $(a))

WARNINGS := -Wall -Wextra -Wshadow -Wstrict-prototypes -Wno-unused-parameter
CFLAGS   ?= -O2 -g
CFLAGS   += -std=c11 $(WARNINGS) -fPIC -fvisibility=hidden $(ARCH_FLAGS) \
            -Iinclude -Isrc $(LIBUSB_CFLAGS) -MMD -MP
LDFLAGS  += $(ARCH_FLAGS)
LIBS     := $(LIBUSB_LIBS) -lpthread

SRCS := src/usb.c src/queue.c src/bitrate.c src/pcanbasic.c src/canxl.c
OBJS := $(SRCS:src/%.c=$(BUILD)/obj/%.o)
DEPS := $(OBJS:.o=.d) $(TOOLS:%=$(BUILD)/obj/%.d)

TOOL_BINS := $(TOOLS:%=$(BUILD)/%)

.PHONY: all clean install install-compat uninstall

all: $(BUILD)/$(DYLIB) $(BUILD)/$(SYMLINK) $(TOOL_BINS)

$(BUILD)/$(DYLIB): $(OBJS)
	@mkdir -p $(@D)
	$(CC) -dynamiclib \
		-install_name @rpath/$(DYLIB) \
		-current_version $(LIBVER) \
		-compatibility_version 1.0.0 \
		-o $@ $^ $(LDFLAGS) $(LIBS)

$(BUILD)/$(SYMLINK): $(BUILD)/$(DYLIB)
	ln -sf $(DYLIB) $@

$(BUILD)/obj/%.o: src/%.c
	@mkdir -p $(@D)
	$(CC) $(CFLAGS) -c -o $@ $<

# Tools link against the freshly built library and find it either next to
# themselves (build tree) or in $(PREFIX)/lib once installed.
$(BUILD)/%: tools/%.c $(BUILD)/$(SYMLINK)
	@mkdir -p $(@D)
	$(CC) $(CFLAGS) -MF $(BUILD)/obj/$*.d -o $@ $< \
		-L$(BUILD) -lPeakUSB \
		-Wl,-rpath,@loader_path -Wl,-rpath,$(PREFIX)/lib $(LDFLAGS)

install: all
	install -d $(DESTDIR)$(PREFIX)/lib $(DESTDIR)$(PREFIX)/include $(DESTDIR)$(PREFIX)/bin
	install -m 0755 $(BUILD)/$(DYLIB) $(DESTDIR)$(PREFIX)/lib/
	ln -sf $(DYLIB) $(DESTDIR)$(PREFIX)/lib/$(SYMLINK)
	install -m 0644 $(HEADER) $(DESTDIR)$(PREFIX)/include/
	install -m 0755 $(TOOL_BINS) $(DESTDIR)$(PREFIX)/bin/
	@echo "Installed PeakUSB into $(DESTDIR)$(PREFIX)."

install-compat: install
	ln -sf $(DYLIB) $(DESTDIR)$(PREFIX)/lib/$(COMPAT)
	@echo "Linked $(COMPAT) -> $(DYLIB) for PCBUSB-compatible applications."

uninstall:
	rm -f $(DESTDIR)$(PREFIX)/lib/$(DYLIB) $(DESTDIR)$(PREFIX)/lib/$(SYMLINK)
	rm -f $(DESTDIR)$(PREFIX)/lib/$(COMPAT)
	rm -f $(DESTDIR)$(PREFIX)/include/PeakUSB.h
	rm -f $(TOOLS:%=$(DESTDIR)$(PREFIX)/bin/%)

clean:
	rm -rf $(BUILD)

-include $(DEPS)
