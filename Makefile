TOOLCHAIN ?= riscv-none-elf
CC      = $(TOOLCHAIN)-gcc
OBJCOPY = $(TOOLCHAIN)-objcopy
SIZE    = $(TOOLCHAIN)-size

ARCH = -march=rv32imac_zicsr -mabi=ilp32

PROTOCOL ?= hurra
ifeq ($(PROTOCOL),hurra)
  # 921600 = WCH-LinkE virtual-COM ceiling (default link is USART3 via the
  # on-board WCH-Link, board.h). Override on the make line (CMD_BAUD=4000000)
  # if you wire an external USB-UART bridge to USART2 PD5/PD6 instead.
  CMD_BAUD  ?= 921600
  PROTO_DEF  = -DPROTOCOL_HURRA
  PROTO_SRC  = src/hurra.c src/third_party/TinyFrame/TinyFrame.c
else ifeq ($(PROTOCOL),ferrum)
  CMD_BAUD  ?= 115200
  PROTO_DEF  = -DPROTOCOL_FERRUM
  PROTO_SRC  = src/ferrum.c
else
  $(error PROTOCOL must be 'hurra' or 'ferrum')
endif

VPATH_INC = -Iinclude -Isrc -Icore -Isrc/third_party/TinyFrame \
            -Ivendor/wch/Core -Ivendor/wch/Peripheral/inc -Ivendor/wch/Debug

DEFINES = -DCH32H417 -DCMD_BAUD=$(CMD_BAUD) $(PROTO_DEF)
CFLAGS  = $(ARCH) $(DEFINES) $(VPATH_INC) -Os -Wall -Wno-unused-variable \
          -ffunction-sections -fdata-sections -fsingle-precision-constant
LDBASE  = $(ARCH) -nostartfiles -Wl,--gc-sections --specs=nano.specs --specs=nosys.specs -lm

LIBSRC = $(wildcard vendor/wch/Peripheral/src/*.c) \
         vendor/wch/Core/core_riscv.c \
         vendor/wch/Debug/debug.c

V3F_SRC = src/main_v3f.c src/icc.c src/led.c src/uart.c src/kmbox_cmd.c \
          src/actions.c src/humanize.c core/timebase.c $(PROTO_SRC) \
          core/system_ch32h417.c $(LIBSRC)
V3F_ASM = core/startup_v3f.S
V3F_DEF = -DCore_V3F
v3f: build
	$(CC) $(CFLAGS) $(V3F_DEF) $(LDBASE) -Tcore/link_v3f.ld \
	   -o build/v3f.elf $(V3F_ASM) $(V3F_SRC)
	$(OBJCOPY) -O ihex build/v3f.elf build/v3f.hex
	$(SIZE) build/v3f.elf

# Task 5.1: usb_host.c (USBHS host driver) defines usb_host_control_transfer(),
# so desc_capture.c (which calls it) links cleanly into the V5F image.
# Task 5.2: full MITM relay. usb_merge.c (HID-aware merge, ICC-fed) +
# core/timebase_v5f.c (TIM4 millis for release scheduling) + humanize.c
# (humanize_filter, referenced by the merge). actions.c is linked for the
# physical-mask state (g_phys_mask / act_phys_*) the merge references; its
# kmbox_inject_* path (-> kmbox_cmd_*) is never exercised on V5F, so
# src/kmbox_cmd_v5f_stub.c provides the otherwise-undefined kmbox_cmd_* sinks.
V5F_SRC = src/main_v5f.c src/icc.c src/usb_host.c src/usb_device.c \
          src/usb_merge.c src/desc_capture.c src/actions.c src/humanize.c \
          src/kmbox_cmd_v5f_stub.c src/led.c core/timebase_v5f.c \
          core/system_ch32h417.c $(LIBSRC)
V5F_ASM = core/startup_v5f.S
V5F_DEF = -DCore_V5F -Dsystick2
v5f: build
	# Trailing -lm: the merge pulls humanize.c, whose humanize_filter() calls
	# sqrtf(). With -lm only in LDBASE (before the objects) the linker has not
	# yet seen the undefined sqrtf when it scans libm, so resolve it by listing
	# -lm again after the sources.
	$(CC) $(CFLAGS) $(V5F_DEF) $(LDBASE) -Tcore/link_v5f.ld \
	   -o build/v5f.elf $(V5F_ASM) $(V5F_SRC) -lm
	$(OBJCOPY) -O ihex build/v5f.elf build/v5f.hex
	$(SIZE) build/v5f.elf

build:
	mkdir -p build

all: build v3f v5f merge

merge: v3f v5f
	$(OBJCOPY) -I ihex -O binary build/v3f.hex build/v3f.bin
	$(OBJCOPY) -I ihex -O binary build/v5f.hex build/v5f.bin
	python3 scripts/merge_images.py build/v3f.bin build/v5f.bin 0x10000 build/Merge.bin

flash: merge
	openocd -f wch-riscv.cfg -c "program build/Merge.bin 0x08000000 verify reset exit"

clean:
	rm -rf build

test:
	cc -std=c11 -O2 -DHUMANIZE_HOSTTEST -Isrc -o /tmp/humanize_test test/humanize_test.c src/humanize.c -lm
	/tmp/humanize_test
	cc -std=c11 -O2 -Isrc -o /tmp/motion_test test/motion_test.c src/actions.c -lm
	/tmp/motion_test

.PHONY: v3f v5f all merge flash clean test build
