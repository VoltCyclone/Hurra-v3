#!/usr/bin/env python3
"""Dump full USB descriptors for a HID mouse plugged directly into the Mac.

Ground-truth reference for the MITM clone: prints the device descriptor, the
full config blob (raw bytes + parsed interface/HID/endpoint tree), and for every
HID interface the *actual* report-descriptor length and bytes (via a class
GET_DESCRIPTOR(REPORT) on interface). This is the authority we compare the
board's capture against to explain rdlen=0 on if2/if3.

Run with the mouse on a Mac USB port (NOT the board). Needs sudo on macOS to
claim the HID interfaces for the report-descriptor request.
"""
import sys
import usb.core
import usb.util

HID_DESCRIPTOR = 0x21
HID_REPORT = 0x22


def find_candidates():
    devs = []
    for d in usb.core.find(find_all=True):
        try:
            # Skip obvious non-mice: the WCH-Link and Yubikey we already know.
            if d.idVendor == 0x1a86 and d.idProduct == 0x8010:
                continue
            if d.idVendor == 0x1050:  # Yubico
                continue
        except Exception:
            pass
        devs.append(d)
    return devs


def hexbytes(b):
    return " ".join("%02x" % x for x in b)


def parse_config(raw):
    """Walk the config blob, print interface/HID/endpoint tree, return per-iface
    HID report length parsed from the embedded HID descriptor."""
    p = 0
    cur_if = None
    iface_report_len = {}  # bInterfaceNumber -> wDescriptorLength (REPORT)
    while p + 1 < len(raw):
        blen = raw[p]
        btype = raw[p + 1]
        if blen < 2 or p + blen > len(raw):
            break
        if btype == 0x04:  # INTERFACE
            ifnum, alt, neps, cls, sub, proto = raw[p+2], raw[p+3], raw[p+4], raw[p+5], raw[p+6], raw[p+7]
            cur_if = ifnum
            print(f"  IFACE #{ifnum} alt={alt} nEP={neps} class={cls:#04x} sub={sub:#04x} proto={proto:#04x}")
        elif btype == HID_DESCRIPTOR:  # HID
            num = raw[p+5]
            print(f"    HID desc bcdHID={raw[p+3]|(raw[p+4]<<8):#06x} nDesc={num}")
            for i in range(num):
                off = 6 + i*3
                if off+2 < blen:
                    rtype = raw[p+off]
                    rlen = raw[p+off+1] | (raw[p+off+2] << 8)
                    print(f"      subdesc type={rtype:#04x} len={rlen}")
                    if rtype == HID_REPORT and cur_if is not None:
                        iface_report_len[cur_if] = rlen
        elif btype == 0x05:  # ENDPOINT
            addr = raw[p+2]; attr = raw[p+3]; mps = raw[p+4] | (raw[p+5] << 8); interval = raw[p+6]
            print(f"    EP {addr:#04x} attr={attr:#04x} mps={mps} interval={interval}")
        p += blen
    return iface_report_len


def main():
    cands = find_candidates()
    if not cands:
        print("No candidate device found. Plug the mouse into the Mac.")
        return 1
    for d in cands:
        try:
            man = usb.util.get_string(d, d.iManufacturer) if d.iManufacturer else "?"
            prod = usb.util.get_string(d, d.iProduct) if d.iProduct else "?"
        except Exception:
            man = prod = "?"
        print("=" * 70)
        print(f"VID:PID = {d.idVendor:#06x}:{d.idProduct:#06x}  '{man}' '{prod}'")
        print(f"  bDeviceClass={d.bDeviceClass:#04x} bMaxPacketSize0={d.bMaxPacketSize0} "
              f"bcdUSB={d.bcdUSB:#06x} numConfigs={d.bNumConfigurations}")
        try:
            cfg = d.get_active_configuration()
            wtotal = cfg.wTotalLength
        except Exception as e:
            print("  get_active_configuration failed:", e)
            wtotal = None
        # Raw config blob via control transfer (type=device, GET_DESCRIPTOR config 0)
        try:
            raw = d.ctrl_transfer(0x80, 0x06, (0x02 << 8) | 0, 0, 4096)
            raw = bytes(raw)
            wtotal = raw[2] | (raw[3] << 8)
            print(f"  config wTotalLength={wtotal}  (captured {len(raw)} bytes)")
            print("  RAW CONFIG:", hexbytes(raw))
            iface_report_len = parse_config(raw)
        except Exception as e:
            print("  raw config fetch failed:", e)
            iface_report_len = {}

        # Now fetch each HID interface's report descriptor for the TRUE length.
        for ifnum, rlen in sorted(iface_report_len.items()):
            try:
                # GET_DESCRIPTOR(REPORT) to interface: bmRequestType=0x81, wValue=0x2200, wIndex=iface
                rep = d.ctrl_transfer(0x81, 0x06, (HID_REPORT << 8) | 0, ifnum, rlen)
                rep = bytes(rep)
                print(f"  >> iface {ifnum}: REPORT desc reported_len={rlen} got={len(rep)}")
                print(f"     {hexbytes(rep)}")
            except Exception as e:
                print(f"  >> iface {ifnum}: REPORT desc fetch FAILED (reported_len={rlen}): {e}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
