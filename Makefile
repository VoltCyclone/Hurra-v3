TOOLCHAIN ?= riscv-none-elf

# Locate the toolchain's bin dir. Prefer one already on PATH; otherwise fall back
# to the newest xPack install (the MounRiver/xPack GNU RISC-V GCC ships here and
# is NOT on PATH in non-login shells, so `make` and scripts/flash.py would fail
# with "riscv-none-elf-gcc: No such file or directory"). Override with TOOLCHAIN
# for a different prefix, or TOOLCHAIN_BIN to point at a specific bin dir.
TOOLCHAIN_BIN ?= $(shell \
  if command -v $(TOOLCHAIN)-gcc >/dev/null 2>&1; then \
    dirname `command -v $(TOOLCHAIN)-gcc`; \
  else \
    ls -d $(HOME)/Library/xPacks/@xpack-dev-tools/riscv-none-elf-gcc/*/.content/bin \
      2>/dev/null | sort -V | tail -1; \
  fi)
TOOLCHAIN_PREFIX = $(if $(TOOLCHAIN_BIN),$(TOOLCHAIN_BIN)/,)$(TOOLCHAIN)

CC      = $(TOOLCHAIN_PREFIX)-gcc
OBJCOPY = $(TOOLCHAIN_PREFIX)-objcopy
SIZE    = $(TOOLCHAIN_PREFIX)-size

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

# ── Build role (V5F image) ───────────────────────────────────────────────────
# The product is a TWO-BOARD USB MITM: Board B captures the real device on its
# USBHS host port and ships it over a board-to-board SPI link (SPI1/PA3-PA7) to
# Board A, which clones it to the PC. Both boards run the SAME V3F image; only the
# V5F image differs by role. `make` builds both role images (see `all`):
#   BoardB.bin  (BOARD=host)   — SPI master + real USBHS capture
#   BoardA.bin  (BOARD=device) — SPI slave + USB clone to the PC
#
# BOARD is set per-role by `all`; leave it empty only for the opt-in single-board
# relay (`make relay`), where one board does host+device itself.
BOARD ?=
ifeq ($(BOARD),host)
  BOARD_DEF = -DBOARD_ROLE_HOST -DDISPLAY_PRESENT
else ifeq ($(BOARD),device)
  BOARD_DEF = -DBOARD_ROLE_DEVICE
else ifneq ($(BOARD),)
  $(error BOARD must be 'host' or 'device' (empty = single-board relay))
else
  BOARD_DEF = -DDISPLAY_PRESENT
endif
DEFINES += $(BOARD_DEF)

# Opt-in bench/bring-up flags:
#   HOST_SYNTH=1   (BOARD=host)   drive Board B from a synthetic mouse, no real
#                                 USB host — isolated SPI link/transport test.
#   DEVICE_LOCAL=1 (BOARD=device) drive the clone from a local synthetic mouse
#                                 with NO SPI — isolates the device USB bring-up.
#   SELFTEST=master|slave         board-to-board SPI echo harness instead of the
#                                 relay (src/spi_link_selftest.c). Flash one board
#                                 each way, watch PC3: slow ~1 Hz = link healthy,
#                                 fast ~8 Hz = errors/no link.
ifeq ($(HOST_SYNTH),1)
  DEFINES += -DTWO_BOARD_HOST_SYNTH
endif
ifeq ($(DEVICE_LOCAL),1)
  DEFINES += -DTWO_BOARD_DEVICE_LOCAL
endif
SELFTEST ?=
ifeq ($(SELFTEST),master)
  DEFINES += -DSPI_LINK_SELFTEST -DSPI_LINK_ROLE_MASTER
else ifeq ($(SELFTEST),slave)
  DEFINES += -DSPI_LINK_SELFTEST
else ifneq ($(SELFTEST),)
  $(error SELFTEST must be 'master' or 'slave')
endif

# Merged-image name, derived from the role so `all` can build both side by side.
ifeq ($(BOARD),host)
  IMAGE ?= BoardB
else ifeq ($(BOARD),device)
  IMAGE ?= BoardA
else
  IMAGE ?= Merge
endif

# EXTRADEF: extra flags appended to CFLAGS for BOTH cores (intended for -D
# defines, e.g. EXTRADEF=-DV5F_STAGE_DIAG_OFF). NOTE: the v5f target additionally
# appends -funroll-loops (see the `v5f: EXTRADEF +=` line below), so a conflicting
# optimization flag passed here is overridden on V5F.
EXTRADEF ?=
# Warning policy. -Werror is deliberately NOT applied globally: the vendored WCH
# StdPeriph sources (vendor/wch/) emit warnings we do not control, and a global
# -Werror would break the build on toolchain drift. Treat firmware-source
# warnings as errors case-by-case instead.
WARNINGS = -Wall -Wno-unused-variable
CFLAGS  = $(ARCH) $(DEFINES) $(EXTRADEF) $(VPATH_INC) -Os $(WARNINGS) \
          -ffunction-sections -fdata-sections -fsingle-precision-constant $(FP_CFLAGS)
# Per-core FP codegen flags (empty for V3F soft-float; set on the v5f target).
FP_CFLAGS ?=
LDBASE  = $(ARCH) -nostartfiles -Wl,--gc-sections --specs=nano.specs --specs=nosys.specs -lm

LIBSRC = $(wildcard vendor/wch/Peripheral/src/*.c) \
         vendor/wch/Core/core_riscv.c \
         vendor/wch/Debug/debug.c

V3F_SRC = src/main_v3f.c src/icc.c src/icc_status.c src/led.c src/uart.c src/kmbox_cmd.c \
          src/actions.c src/humanize.c src/st7789.c src/display.c src/temp.c core/timebase.c \
          $(PROTO_SRC) core/system_ch32h417.c $(LIBSRC)
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
V5F_SRC = src/main_v5f.c src/icc.c src/icc_status.c src/usb_host.c \
          src/usb_device.c src/usb_device_hs.c src/usb_device_fs.c \
          src/usb_merge.c src/hid_layout.c src/desc_capture.c src/actions.c src/humanize.c \
          src/kmbox_cmd_v5f_stub.c src/led.c src/spi_link.c src/spi_frame.c \
          src/spi_frame_stream.c \
          src/usb_hs_desc.c src/synth_mouse.c src/desc_xfer.c src/report_xfer.c src/desc_serialize.c src/two_board.c \
          src/spi_link_selftest.c core/timebase_v5f.c \
          src/ws2812.c src/ws2812_pioc_code.c \
          core/system_ch32h417.c $(LIBSRC)
# Board B (host role) parses commands on V5F: link the protocol parser, the
# inject FIFO, and the FIFO-backed inject sinks (kmbox_cmd_host.c) in place of the
# no-op kmbox_cmd_v5f_stub.c. Board B never clones a captured device to a PC.
#
# USBFS_IRQHandler ownership: drop src/usb_device_fs.c (the HID-clone USBFS driver)
# so its USBFS_IRQHandler is gone, and add src/usb_cdc_fs.c, which becomes the SOLE
# USBFS_IRQHandler in the host image (a CDC-ACM virtual COM port on the otherwise
# idle USBFS controller). We KEEP src/usb_device.c (the speed dispatcher) and
# src/usb_device_hs.c (which owns USBHS_IRQHandler, a different vector — no
# conflict): main_v5f.c's single-board relay path and usb_merge.c reference the
# usb_device_* API and would fail to link without the dispatcher. On the host image
# those clone paths are dead code (two_board_host_run() never returns), so the
# usbfsd_* sinks the dispatcher's FS branch needs are provided as no-op stubs in
# usb_cdc_fs.c. (Dropping the whole usb_device*.c trio, as first scoped, breaks the
# link on main_v5f.c / usb_merge.c usb_device_* references — verified at build.)
ifeq ($(BOARD),host)
  V5F_SRC := $(filter-out src/kmbox_cmd_v5f_stub.c src/usb_device_fs.c,$(V5F_SRC)) \
             src/inject_link.c src/kmbox_cmd_host.c src/usb_cdc_fs.c $(PROTO_SRC)
  # gesture engine: host+V5F only (two_board.c calls are guarded by BOARD_ROLE_HOST;
  # gesture_stream_v5f bridges the act_move stream-filter hook to the residual filter;
  # gesture.c uses timebase_v5f_us which is V5F-only).
  V5F_SRC += src/gesture.c src/gesture_stream_v5f.c
else
  V5F_SRC += src/inject_link.c
endif
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
# Unroll the V5F per-report hot path (humanize + HID field merge + FIFO copies),
# all pinned to ITCM. ITCM is ~86% free (18.5K of 128K), so trading a few hundred
# bytes for fewer per-report bound checks is worth it on the latency-critical core.
# v5f-scoped: V3F stays pure -Os. -funroll-loops only touches GCC-bounded loops, so
# the FIFO/field loops unroll while the unbounded poll/NAK spins are left alone.
v5f: EXTRADEF += -funroll-loops
# SPI link master uses DMA1 ch2/ch3 by default (frees V5F during each 32-byte slot
# instead of per-word TXE/RXNE polling). Master-only; the slave stays IRQ-driven.
# Cross-core-safe: only V5F compiles/calls spi_link.c, so nothing on V3F races the
# RCC->HBPCENR RMW (see the CROSS-CORE HAZARD note in spi_link.c). Bench-gated:
# verify spi_link_master_wedges stays 0 and USB enum holds. Escape hatch / rollback:
# build with `make v5f SPI_LINK_DMA=0` to fall back to the polled path (no code edit).
SPI_LINK_DMA ?= 1
ifeq ($(SPI_LINK_DMA),1)
v5f: EXTRADEF += -DSPI_LINK_DMA
endif
# SPI link SCK rate. Default /2 (~50 MHz); fall back to /4 (~25 MHz) with
# `make v5f SPI_LINK_FAST=0` if spi_link_rx_overflows / _master_wedges rise off 0
# under load (the slave samples SCK against its own crystal). See spi_link.c.
SPI_LINK_FAST ?= 1
ifneq ($(SPI_LINK_FAST),1)
v5f: EXTRADEF += -DLINK_SPI_SLOW
endif
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

# Default product: build BOTH two-board role images.
#   build/BoardB.bin = host (SPI master + real USBHS capture)
#   build/BoardA.bin = device (SPI slave + USB clone to the PC)
# Each is a full merged dual-core image; flash one per board (see flash-boardb /
# flash-boarda). Recurses so each role gets its own BOARD= define cleanly.
all:
	$(MAKE) merge BOARD=host
	$(MAKE) merge BOARD=device

# Opt-in single-board relay (one board does host+device itself) -> build/Merge.bin.
relay:
	$(MAKE) merge BOARD=

# Merge the two core images into build/$(IMAGE).bin (IMAGE derives from BOARD).
merge: v3f v5f
	$(OBJCOPY) -I ihex -O binary build/v3f.hex build/v3f.bin
	$(OBJCOPY) -I ihex -O binary build/v5f.hex build/v5f.bin
	python3 scripts/merge_images.py build/v3f.bin build/v5f.bin 0x10000 build/$(IMAGE).bin

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
# Running firmware disables SWJ (PB8/PB9) during USB init, so wlink commands then
# NAK with `protocol error: 0x55`. A power-off erase gates the target rail via the
# WCH-LinkE and attaches in the clean reset window, so flashing always works
# without button-tapping. Passed as --chip; the part is the CH32H41X family.
WLINK_CHIP   ?= CH32H41X
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
# wlink path: power-off erase first (clears the 0x55 SWJ-disabled NAK from running
# firmware — see WLINK_CHIP), then flash with -e at the given address.
define _FLASH_IMG
	@tool=`$(_PICK_FLASHER)` || exit $$?; \
	if [ "$$tool" = wlink ]; then \
	  echo "==> wlink erase (power-off) + flash $(1) @ $(2)"; \
	  $(WLINK) --chip $(WLINK_CHIP) erase --method power-off; \
	  $(WLINK) --chip $(WLINK_CHIP) flash -e --address $(2) $(1); \
	else \
	  echo "==> $(WCH_OPENOCD) program $(1) @ $(WCH_OCD_ADDR) (per $(WCH_CFG))"; \
	  $(WCH_OPENOCD) -f $(WCH_CFG) -c init -c halt \
	    -c "program $(1) $(WCH_OCD_ADDR) verify" -c "wlink_reset_resume" -c exit; \
	fi
endef

# Flash the two-board role images. Plug ONE WCH-LinkE at a time (or pass
# WLINK_DEV='-d <index>' via WLINK to pick a probe). Build fresh, then program.
# Preferred two-board flow: scripts/flash.py --host-serial <S> --device-serial <S>
# (serial-addressed, retries, --json for CI/AI). These make targets remain for
# single-probe / single-core use.
# Board B = host (SPI master + USBHS capture).
flash-boardb:
	$(MAKE) merge BOARD=host
	$(call _FLASH_IMG,build/BoardB.bin,$(FLASH_ADDR))

# Board A = device (SPI slave + USB clone to the PC).
flash-boarda:
	$(MAKE) merge BOARD=device
	$(call _FLASH_IMG,build/BoardA.bin,$(FLASH_ADDR))

# Flash the single-board relay image (opt-in; pairs with `make relay`).
flash: relay
	$(call _FLASH_IMG,build/Merge.bin,$(FLASH_ADDR))

# Flash a single core image (debug aid). These program the same flash region at
# the core's own origin; flashing one core alone is for bring-up, not normal use.
flash-v3f: v3f
	$(OBJCOPY) -I ihex -O binary build/v3f.hex build/v3f.bin
	$(call _FLASH_IMG,build/v3f.bin,0x08000000)

flash-v5f: v5f
	$(OBJCOPY) -I ihex -O binary build/v5f.hex build/v5f.bin
	$(call _FLASH_IMG,build/v5f.bin,0x08010000)

# Full-chip erase. wlink uses power-off so it works even against running firmware
# (the 0x55 SWJ-disabled NAK), same as the flash path.
erase:
	@tool=`$(_PICK_FLASHER)` || exit $$?; \
	if [ "$$tool" = wlink ]; then \
	  echo "==> wlink erase (power-off)"; \
	  $(WLINK) --chip $(WLINK_CHIP) erase --method power-off; \
	else \
	  echo "==> $(WCH_OPENOCD) flash erase"; \
	  $(WCH_OPENOCD) -f $(WCH_CFG) -c init -c halt \
	    -c "flash erase_sector wch_riscv 0 last" -c "wlink_reset_resume" -c exit; \
	fi

clean:
	rm -rf build

test-report-xfer:
	cc -std=c11 -O2 -Wall -Wextra -Isrc -o /tmp/report_xfer_test test/report_xfer_test.c src/report_xfer.c
	/tmp/report_xfer_test

test: test-report-xfer
	cc -std=c11 -O2 -DHUMANIZE_HOSTTEST -Isrc -o /tmp/humanize_test test/humanize_test.c src/humanize.c -lm
	/tmp/humanize_test
	cc -std=c11 -O2 -Isrc -o /tmp/motion_test test/motion_test.c src/actions.c -lm
	/tmp/motion_test
	cc -std=c11 -O2 -Isrc -o /tmp/display_test test/display_test.c src/display.c src/icc_status.c
	/tmp/display_test
	cc -std=c11 -O2 -Wall -Wextra -Isrc -o /tmp/spi_frame_test test/spi_frame_test.c src/spi_frame.c
	/tmp/spi_frame_test
	cc -std=c11 -O2 -Wall -Wextra -Isrc -o /tmp/usb_hs_desc_test test/usb_hs_desc_test.c src/usb_hs_desc.c
	/tmp/usb_hs_desc_test
	cc -std=c11 -O2 -Wall -Wextra -Isrc -o /tmp/desc_xfer_test test/desc_xfer_test.c src/desc_xfer.c
	/tmp/desc_xfer_test
	cc -std=c11 -O2 -Wall -Wextra -Isrc -o /tmp/desc_serialize_test test/desc_serialize_test.c src/desc_serialize.c
	/tmp/desc_serialize_test
	cc -std=c11 -O2 -Wall -Wextra -Isrc -o /tmp/spi_frame_stream_test test/spi_frame_stream_test.c src/spi_frame_stream.c src/spi_frame.c
	/tmp/spi_frame_stream_test
	cc -std=c11 -O1 -g -fsanitize=address -DHUMANIZE_HOSTTEST -Isrc \
	   -o /tmp/usb_merge_test test/usb_merge_test.c src/usb_merge.c \
	   src/hid_layout.c src/humanize.c -lm
	/tmp/usb_merge_test
	cc -std=c11 -O1 -g -fsanitize=address -Isrc -o /tmp/inject_apply_test test/inject_apply_test.c src/usb_merge.c src/hid_layout.c
	/tmp/inject_apply_test
	cc -std=c11 -O2 -Wall -Isrc -o /tmp/inject_link_test test/inject_link_test.c src/inject_link.c src/spi_frame.c
	/tmp/inject_link_test
	cc -std=c11 -O2 -Wall -Isrc -o /tmp/usb_host_time_test test/usb_host_time_test.c
	/tmp/usb_host_time_test
	cc -std=c11 -O2 -Wall -Isrc -o /tmp/usb_desc_interval_test test/usb_desc_interval_test.c
	/tmp/usb_desc_interval_test
	cc -std=c11 -O2 -Wall -Isrc -o /tmp/uart_rx_class_test test/uart_rx_class_test.c
	/tmp/uart_rx_class_test
	cc -std=c11 -O2 -Wall -Isrc -o /tmp/hid_iface_index_test test/hid_iface_index_test.c
	/tmp/hid_iface_index_test
	cc -std=c11 -O2 -Wall -Isrc -o /tmp/display_rowpick_test test/display_rowpick_test.c
	/tmp/display_rowpick_test
	cc -std=c11 -O2 -Wall -DWS2812_HOSTTEST -Isrc -o /tmp/ws2812_test test/ws2812_test.c src/ws2812.c
	/tmp/ws2812_test
	cc -std=c11 -O2 -DGESTURE_HOSTTEST -Isrc -o /tmp/gesture_test test/gesture_test.c src/gesture.c -lm
	/tmp/gesture_test
	python3 -m unittest test.flash_py_test
	python3 -m unittest test.humanization_analyze_test
	python3 -m unittest test.raw_input_classifier_trace_test
	python3 -m unittest test.raw_input_classifier_encoders_test
	python3 -m unittest test.raw_input_classifier_windowing_test
	python3 -m unittest test.raw_input_classifier_metrics_test
	python3 -m unittest test.raw_input_classifier_capture_test

test-torch:
	python3 -m unittest test.raw_input_classifier_model_test
	python3 -m unittest test.raw_input_classifier_train_test

.PHONY: v3f v5f all relay merge flash flash-boarda flash-boardb flash-v3f flash-v5f erase clean test test-report-xfer test-torch build
