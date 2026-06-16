TOOLCHAIN ?= riscv-none-elf
CC      = $(TOOLCHAIN)-gcc
OBJCOPY = $(TOOLCHAIN)-objcopy
SIZE    = $(TOOLCHAIN)-size

ARCH = -march=rv32imac_zicsr -mabi=ilp32

PROTOCOL ?= hurra
ifeq ($(PROTOCOL),hurra)
  # 921600 = WCH-LinkE virtual-COM ceiling (default link is USART1 PA9/PA10 via
  # the on-board WCH-Link, board.h). Override on the make line (e.g.
  # CMD_BAUD=4000000) if you repoint board.h to an external USB-UART bridge.
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
EXTRADEF ?=
CFLAGS  = $(ARCH) $(DEFINES) $(EXTRADEF) $(VPATH_INC) -Os -Wall -Wno-unused-variable \
          -ffunction-sections -fdata-sections -fsingle-precision-constant $(FP_CFLAGS)
# Per-core FP codegen flags (empty for V3F soft-float; set on the v5f target).
FP_CFLAGS ?=
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
# V5F claims the QingKe hardware FPU (powered on in startup_v5f.S, mstatus FS=3).
# ilp32f is an ABI break vs the ilp32 baseline — every object in the V5F image
# (vendor libs, libm, libgcc) is rebuilt with this -mabi because each core links
# as its own ELF (merge_images.py concatenates the two .bin's; no cross-core
# object linking), so V3F can stay ilp32 while V5F is ilp32f. Requires a clean
# build. ISRs are FP-free (relay hot path runs in the main loop), so no FP
# context save/restore is needed. See docs/TODO-hardfloat-fpu.md.
v5f: ARCH = -march=rv32imafc_zicsr -mabi=ilp32f
# -fno-math-errno lets GCC inline the hot-path sqrtf() (humanize.c) to a single
# hardware `fsqrt.s` instead of newlib's software __ieee754_sqrtf. Safe here:
# both call sites are sqrtf(x*x + y*y) (always >= 0, no domain error) and the
# firmware never reads errno. It does not reassociate or change rounding (unlike
# full -ffast-math), so the humanize math stays bit-stable.
v5f: FP_CFLAGS = -fno-math-errno
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

# ── Flashing via the on-board WCH-LinkE ──────────────────────────────────────
# The CH32H417's flash is one physical region exposed at two aliases: the WCH
# StdPeriph FLASH_BASE is 0x08000000, while WCH's OpenOCD flash bank is 0x00000000.
# Our linker lays V3F at 0x00000000 / V5F at 0x00010000 and Merge.bin is built to
# that layout, so either alias programs the same bytes. We support two CLI tools:
#
#   wlink       — ch32-rs/wlink (Rust). Explicitly lists CH32H417. Uses the
#                 0x08000000 alias: `wlink flash --address 0x08000000 img.bin`.
#                 Install: `cargo install --git https://github.com/ch32-rs/wlink`
#                 (+ `brew install libusb`).
#   wch-openocd — WCH's OpenOCD fork (cjacker/wch-openocd or openwch/openocd_wch),
#                 binary usually `wch-openocd`. Uses scripts/wch-riscv.cfg
#                 (adapter driver wlinke, sdi transport, flash bank @0x00000000).
#                 NOTE: mainline `openocd` does NOT work — no wlinke driver.
#
# `make flash` auto-detects whichever is installed (wlink first). Override the
# tool with FLASH_TOOL=wlink|openocd, or the binary names with WLINK=/WCH_OPENOCD=.
WLINK        ?= wlink
WCH_OPENOCD  ?= wch-openocd
WCH_CFG      ?= scripts/wch-riscv.cfg
# wlink programs at the 0x08000000 alias; wch-openocd at the 0x00000000 bank
# base (same physical flash). Keep values bare — trailing chars before a `#` on
# a `?=` line become part of the value and leak into the command.
FLASH_ADDR   ?= 0x08000000
WCH_OCD_ADDR ?= 0x00000000
FLASH_TOOL   ?= auto

# Resolve the flasher: explicit FLASH_TOOL, else prefer wlink, else wch-openocd.
# Prints ONLY the resolved tool name ("wlink"/"openocd") to stdout; the install
# hint goes to stderr and the script exits non-zero so callers can abort (the
# stdout capture must not silently turn a "not found" into a wrong tool choice).
define _PICK_FLASHER
sh -c 'tool="$(FLASH_TOOL)"; \
  if [ "$$tool" = "auto" ]; then \
    if command -v $(WLINK) >/dev/null 2>&1; then tool=wlink; \
    elif command -v $(WCH_OPENOCD) >/dev/null 2>&1; then tool=openocd; \
    else tool=""; fi; \
  fi; \
  if [ "$$tool" = wlink ] && ! command -v $(WLINK) >/dev/null 2>&1; then tool=""; fi; \
  if [ "$$tool" = openocd ] && ! command -v $(WCH_OPENOCD) >/dev/null 2>&1; then tool=""; fi; \
  if [ -z "$$tool" ]; then \
    { echo "ERROR: no usable WCH flash tool found (FLASH_TOOL=$(FLASH_TOOL))."; \
      echo "Install one (the WCH-LinkE must be in RV/RISC-V mode):"; \
      echo "  wlink (recommended, lists CH32H417 support):"; \
      echo "    brew install libusb && cargo install --git https://github.com/ch32-rs/wlink"; \
      echo "  or WCH-OpenOCD fork (binary $(WCH_OPENOCD)): https://github.com/openwch/openocd_wch"; \
    } 1>&2; \
    exit 127; \
  fi; \
  echo "$$tool"'
endef

# $(1)=image .bin  $(2)=program address (wlink only). Abort if no flasher.
define _FLASH_IMG
	@tool=`$(_PICK_FLASHER)` || exit $$?; \
	if [ "$$tool" = wlink ]; then \
	  echo "==> wlink flash $(1) @ $(2)"; \
	  $(WLINK) flash --address $(2) $(1); \
	else \
	  echo "==> $(WCH_OPENOCD) program $(1) @ $(WCH_OCD_ADDR) (per $(WCH_CFG))"; \
	  $(WCH_OPENOCD) -f $(WCH_CFG) -c init -c halt \
	    -c "program $(1) $(WCH_OCD_ADDR) verify" -c "wlink_reset_resume" -c exit; \
	fi
endef

# Flash the full dual-core image (default). Depends on merge so the bits are fresh.
flash: merge
	$(call _FLASH_IMG,build/Merge.bin,$(FLASH_ADDR))

# Flash a single core image (debug aid). These program the same flash region at
# the core's own origin; flashing one core alone is for bring-up, not normal use.
flash-v3f: v3f
	$(OBJCOPY) -I ihex -O binary build/v3f.hex build/v3f.bin
	$(call _FLASH_IMG,build/v3f.bin,0x08000000)

flash-v5f: v5f
	$(OBJCOPY) -I ihex -O binary build/v5f.hex build/v5f.bin
	$(call _FLASH_IMG,build/v5f.bin,0x08010000)

# Full-chip erase.
erase:
	@tool=`$(_PICK_FLASHER)` || exit $$?; \
	if [ "$$tool" = wlink ]; then \
	  echo "==> wlink erase"; $(WLINK) erase; \
	else \
	  echo "==> $(WCH_OPENOCD) flash erase"; \
	  $(WCH_OPENOCD) -f $(WCH_CFG) -c init -c halt \
	    -c "flash erase_sector wch_riscv 0 last" -c "wlink_reset_resume" -c exit; \
	fi

clean:
	rm -rf build

test:
	cc -std=c11 -O2 -DHUMANIZE_HOSTTEST -Isrc -o /tmp/humanize_test test/humanize_test.c src/humanize.c -lm
	/tmp/humanize_test
	cc -std=c11 -O2 -Isrc -o /tmp/motion_test test/motion_test.c src/actions.c -lm
	/tmp/motion_test

.PHONY: v3f v5f all merge flash flash-v3f flash-v5f erase clean test build
