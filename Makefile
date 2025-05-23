# Makefile - make script to build USBUART Library
#
# Copyright (C) 2016 Eugene Hutorny <eugene@hutorny.in.ua>
#
# This file is part of USBUART Library. http://usbuart.info
#
# The USBUART Library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License v2
# as published by the Free Software Foundation;
#
# The USBUART Library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with the USBUART Library; if not, see
# <http://www.gnu.org/licenses/gpl-2.0.html>.

# This makefile builds unit tests for cojson
# It is not intended to build any other applications

MAKEFLAGS += --no-builtin-rules
TARGET-DIR := bin
BUILD-DIR ?= ~/usbuart-termux
BOLD:=$(shell tput bold)
NORM:=$(shell tput sgr0)

CC  := $(if $(V),,@)$(PREFIX)/bin/clang$(SUFFIX)
CXX := $(if $(V),,@)$(PREFIX)/bin/clang++$(SUFFIX)
LD  := $(if $(V),,@)$(PREFIX)/bin/clang++$(SUFFIX)

MAKEFLAGS += --no-builtin-rules

SRC-DIRS := src

INCLUDES := include libusb/libusb # Removed $(PREFIX)/include/libusb-1.0

.PHONY: all

.DEFAULT:

all : $(TARGET-DIR)/libusbuart.so

Makefile :: ;

%.mk :: ;

OBJS :=																		\
  capi.o 																	\
  ch34x.o																	\
  core.o																	\
  ftdi.o																	\
  generic.o																	\
  log.o																		\
  pl2303.o																	\


CPPFLAGS += 																\
  $(addprefix -I,$(INCLUDES))												\
  $(shell pkg-config --cflags libusb-1.0)                                   \
  $(addprefix -D,$(CXX-DEFS))												\
  $(if $(V),-v,)															\
  -Wall																		\
  -Wextra																	\
  -O0																		\
  -std=c++1y  																\
  #-fmessage-length=0														\
  #-ffunction-sections  														\
  #-fdata-sections															\

CPPFLAGS_DEBUG += 																\
  $(addprefix -I,$(INCLUDES))												\
  $(addprefix -D,$(CXX-DEFS))												\
  $(if $(V),-v,)															\
  -Wall																		\
  -Wextra																	\
  -O0																		\
  -std=c++1y  																\
  #-fmessage-length=0														\
  #-ffunction-sections  														\
  #-fdata-sections															\
  #-g \

CFLAGS += 																	\
  $(addprefix -I,$(INCLUDES))												\
  $(shell pkg-config --cflags libusb-1.0)                                   \
  $(addprefix -D,$(CXX-DEFS))												\
  $(if $(V),-v,)															\
  -Wall																		\
  -O0																		\
  #-ffunction-sections 														\
  #-fdata-sections 															\
  #-std=gnu99 																\


LDFLAGS +=																	\
  -s -shared 				 												\


LDFLAGS_DEBUG +=																	\

vpath %.cpp $(subst $(eval) ,:,$(SRC-DIRS))
vpath %.c   $(subst $(eval) ,:,$(SRC-DIRS))

$(BUILD-DIR)/%.o: %.c | $(BUILD-DIR)
	@echo "     $(BOLD)cc$(NORM)" $(notdir $<)
	$(CC) $(CFLAGS) -c -o $@ $<

$(BUILD-DIR)/%.o: %.cpp | $(BUILD-DIR)
	@echo "    $(BOLD)c++$(NORM)" $(notdir $<)
	$(CXX) $(CPPFLAGS) -c -o $@ $<

.SECONDARY:

$(TARGET-DIR)/libusbuart.so: $(addprefix $(BUILD-DIR)/,$(OBJS)) | $(TARGET-DIR)
	@echo "    $(BOLD)ld$(NORM) " $(notdir $@)
	$(LD) $(LDFLAGS) -o $@ $^

$(BUILD-DIR)::
	@mkdir -p $@

$(TARGET-DIR)::
	@mkdir -p $@

clean:
	@rm -f $(BUILD-DIR)/*.o *.map $(TARGET-DIR)/*.so


vpath %.cpp $(subst $(eval) ,:,examples)

examples/%.o: %.cpp
	@echo "    $(BOLD)c++$(NORM)" $(notdir $<)
	@echo $(CXX) $(CPPFLAGS_DEBUG) -c -o $@ $<
	$(CXX) $(CPPFLAGS_DEBUG) -c -o $@ $<

uartcat: examples/uartcat-termux.o
	@echo "    $(BOLD)ld$(NORM) " $(notdir $@)
	@echo $(LD) $(LDFLAGS_DEBUG) -Lusb -Lusbuart -o $@ $^
	$(LD) $(LDFLAGS_DEBUG) -L$(PREFIX)/lib/libusb-1.0 -L./bin -lusb-1.0 -lusbuart -o $@ $^
	

test: examples/test.o
	@echo "    $(BOLD)ld$(NORM) " $(notdir $@)
	@echo $(LD) $(LDFLAGS_DEBUG) -o $@ $^
	$(LD) $(LDFLAGS_DEBUG) -L$(PREFIX)/lib/libusb-1.0 -L./bin -lusb-1.0 -lusbuart -o $@ $^

termux_uart_bridge: examples/termux_uart_bridge.o $(TARGET-DIR)/libusbuart.so
	@echo "    $(BOLD)ld$(NORM) " $(notdir $@)
	@echo $(LD) $(LDFLAGS_DEBUG) -L$(TARGET-DIR) -Iinclude -o $@ $< -lusbuart -lusb-1.0 $(shell pkg-config --libs libusb-1.0)
	$(LD) $(LDFLAGS_DEBUG) -L$(TARGET-DIR) -Iinclude -o $@ $< -lusbuart -lusb-1.0 $(shell pkg-config --libs libusb-1.0)
