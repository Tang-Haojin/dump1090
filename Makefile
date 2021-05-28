PROGNAME=dump1090

DUMP1090_VERSION ?= unknown

CPPFLAGS += -I. -DMODES_DUMP1090_VERSION=\"$(DUMP1090_VERSION)\" -DMODES_DUMP1090_VARIANT=\"dump1090-fa\" -fPIC 

DIALECT = -std=c11
CFLAGS += $(DIALECT) -O3 -g -Wall -Wmissing-declarations -Werror -W -D_DEFAULT_SOURCE -fno-common -fPIC 
LIBS = -lpthread -lm
SDR_OBJ = cpu.o fifo.o dsp/helpers/tables.o

UNAME := $(shell uname)

ifeq ($(UNAME), Linux)
  CPPFLAGS += -D_DEFAULT_SOURCE -fPIC 
  LIBS += -lrt
  LIBS_USB += -lusb-1.0
  LIBS_CURSES := -lncurses
  CPUFEATURES ?= yes
endif

ifeq ($(UNAME), Darwin)
  ifneq ($(shell sw_vers -productVersion | egrep '^10\.([0-9]|1[01])\.'),) # Mac OS X ver <= 10.11
    CPPFLAGS += -DMISSING_GETTIME
    COMPAT += compat/clock_gettime/clock_gettime.o
  endif
  CPPFLAGS += -DMISSING_NANOSLEEP
  COMPAT += compat/clock_nanosleep/clock_nanosleep.o
  LIBS_USB += -lusb-1.0
  LIBS_CURSES := -lncurses
  CPUFEATURES ?= yes
endif

ifeq ($(UNAME), OpenBSD)
  CPPFLAGS += -DMISSING_NANOSLEEP
  COMPAT += compat/clock_nanosleep/clock_nanosleep.o
  LIBS_USB += -lusb-1.0
  LIBS_CURSES := -lncurses
endif

ifeq ($(UNAME), FreeBSD)
  CPPFLAGS += -D_DEFAULT_SOURCE
  LIBS += -lrt
  LIBS_USB += -lusb
  LIBS_CURSES := -lncurses
endif

ifeq ($(UNAME), NetBSD)
  CFLAGS += -D_DEFAULT_SOURCE
  LIBS += -lrt
  LIBS_USB += -lusb-1.0
  LIBS_CURSES := -lcurses
endif

CPUFEATURES ?= no

ifeq ($(CPUFEATURES),yes)
  include Makefile.cpufeatures
  CPPFLAGS += -DENABLE_CPUFEATURES -Icpu_features/include -fPIC 
endif

SDR_OBJ += sdr_hackrf.o
CPPFLAGS += -DENABLE_HACKRF -fPIC 
CFLAGS += $(shell pkg-config --cflags libhackrf) -fPIC 
LIBS_SDR += $(shell pkg-config --libs libhackrf)


##
## starch (runtime DSP code selection) mix, architecture-specific
##

ARCH ?= $(shell uname -m)
ifneq ($(CPUFEATURES),yes)
  # need to be able to detect CPU features at runtime to enable any non-standard compiler flags
  STARCH_MIX := generic
  CPPFLAGS += -DSTARCH_MIX_GENERIC -fPIC 
else
  ifeq ($(ARCH),x86_64)
    # AVX, AVX2
    STARCH_MIX := x86
    CPPFLAGS += -DSTARCH_MIX_X86 -fPIC 
  else ifeq ($(findstring arm,$(ARCH)),arm)
    # ARMv7 NEON
    STARCH_MIX := arm
    CPPFLAGS += -DSTARCH_MIX_ARM -fPIC 
  else ifeq ($(findstring aarch,$(ARCH)),aarch)
    STARCH_MIX := aarch64
    CPPFLAGS += -DSTARCH_MIX_AARCH64 -fPIC 
  else
    STARCH_MIX := generic
    CPPFLAGS += -DSTARCH_MIX_GENERIC -fPIC 
  endif
endif
all: showconfig dump1090 dump1090.so starch-benchmark mode_s_dll.o dump1090_dll.o

STARCH_COMPILE := $(CC) $(CPPFLAGS) $(CFLAGS) -c
include dsp/generated/makefile.$(STARCH_MIX)

showconfig:
	@echo "Building with:" >&2
	@echo "  Version string:  $(DUMP1090_VERSION)" >&2
	@echo "  DSP mix:         $(STARCH_MIX)" >&2
	@echo "  HackRF support:  yes" >&2

%.o: %.c *.h
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

mode_s_dll.o: mode_s.c mode_s.h
	$(CC) -DDYNAMIC_LINK=1 $(CPPFLAGS) $(CFLAGS)  -c $< -o $@

dump1090_dll.o: dump1090.c dump1090.h
	$(CC) -DDYNAMIC_LINK=1 $(CPPFLAGS) $(CFLAGS)  -c $< -o $@

dump1090: dump1090.o mode_ac.o mode_s.o comm_b.o crc.o demod_2400.o cpr.o icao_filter.o track.o util.o convert.o ais_charset.o $(SDR_OBJ) $(COMPAT) $(CPUFEATURES_OBJS) $(STARCH_OBJS)
	$(CC) -g -o $@ $^ $(LDFLAGS) $(LIBS) $(LIBS_SDR) $(LIBS_CURSES)

dump1090.so: dump1090_dll.o mode_ac.o mode_s_dll.o comm_b.o crc.o demod_2400.o cpr.o icao_filter.o track.o util.o convert.o ais_charset.o $(SDR_OBJ) $(COMPAT) $(CPUFEATURES_OBJS) $(STARCH_OBJS)
	$(CC) -fPIC -shared -g -o $@ $^ $(LDFLAGS) $(LIBS) $(LIBS_SDR) $(LIBS_CURSES)

starch-benchmark: cpu.o dsp/helpers/tables.o $(CPUFEATURES_OBJS) $(STARCH_OBJS) $(STARCH_BENCHMARK_OBJ)
	$(CC) -g -o $@ $^ $(LDFLAGS) $(LIBS)

clean:
	rm -f *.o compat/clock_gettime/*.o compat/clock_nanosleep/*.o cpu_features/src/*.o dsp/generated/*.o dsp/helpers/*.o $(CPUFEATURES_OBJS) dump1090 dump1090.so dump1090_dll.o cprtests crctests starch-benchmark

test: cprtests
	./cprtests

cprtests: cpr.o cprtests.o
	$(CC) $(CPPFLAGS) $(CFLAGS) -g -o $@ $^ -lm

crctests: crc.c crc.h
	$(CC) $(CPPFLAGS) $(CFLAGS) -g -DCRCDEBUG -o $@ $<

starchgen:
	dsp/starchgen.py .

.PHONY: wisdom.local
wisdom.local: starch-benchmark
	./starch-benchmark -i 5 -o wisdom.local mean_power_u16 mean_power_u16_aligned magnitude_uc8 magnitude_uc8_aligned
	./starch-benchmark -i 5 -r wisdom.local -o wisdom.local
