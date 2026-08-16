# SPDX-License-Identifier: GPL-2.0-or-later OR BSD-3-Clause
# macOS CAN drivers. Builds both libraries.
#
#   make                        build everything
#   make -C PeakUSB             build one driver only
#   make ARCHS="arm64 x86_64"   universal binaries
#   sudo make install           install both into $(PREFIX) (default /usr/local)

DRIVERS := PeakUSB KvaserUSB

.PHONY: all clean install uninstall $(DRIVERS)

all: $(DRIVERS)

$(DRIVERS):
	$(MAKE) -C $@

clean install uninstall:
	@for d in $(DRIVERS); do $(MAKE) -C $$d $@ || exit 1; done
