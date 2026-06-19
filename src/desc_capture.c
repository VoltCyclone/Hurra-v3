// desc_capture.c — capture a device's full descriptor set on the USBHS host
// port for the clone to replay. See desc_capture.h.
#include <string.h>
#include "usb_host.h"
#include "desc_capture.h"

extern void delay(uint32_t msec);

static usb_setup_t make_get_descriptor(uint8_t type, uint8_t index,
	uint16_t langid, uint16_t length)
{
	usb_setup_t s;
	s.bmRequestType = 0x80; // Device-to-Host, Standard, Device
	s.bRequest = USB_REQ_GET_DESCRIPTOR;
	s.wValue = (type << 8) | index;
	s.wIndex = langid;
	s.wLength = length;
	return s;
}

static usb_setup_t make_get_iface_descriptor(uint8_t type, uint8_t index,
	uint16_t iface, uint16_t length)
{
	usb_setup_t s;
	s.bmRequestType = 0x81; // Device-to-Host, Standard, Interface
	s.bRequest = USB_REQ_GET_DESCRIPTOR;
	s.wValue = (type << 8) | index;
	s.wIndex = iface;
	s.wLength = length;
	return s;
}

static bool parse_config_descriptor(captured_descriptors_t *desc)
{
	const uint8_t *p = desc->config_desc;
	const uint8_t *end = p + desc->config_desc_len;
	captured_iface_t *cur_iface = NULL;
	desc->num_ifaces = 0;

	while (p < end) {
		uint8_t dlen = p[0];
		uint8_t dtype = p[1];

		if (dlen < 2 || p + dlen > end) break;

		if (dtype == USB_DESC_INTERFACE && dlen >= 9) {
			uint8_t alt_setting = p[3];
			// Skip alternate settings; alt 0 precedes higher alts per USB spec
			// descriptor ordering.
			if (alt_setting != 0) {
				cur_iface = NULL;
				p += dlen;
				continue;
			}

			if (desc->num_ifaces < MAX_INTERFACES) {
				cur_iface = &desc->ifaces[desc->num_ifaces++];
				memset(cur_iface, 0, sizeof(*cur_iface));
				cur_iface->iface_num        = p[2];
				cur_iface->iface_class      = p[5];
				cur_iface->iface_subclass   = p[6];
				cur_iface->iface_protocol   = p[7];
				cur_iface->iface_string_idx = p[8];  // iInterface
			} else {
				cur_iface = NULL;
			}
		} else if (dtype == USB_DESC_HID && dlen >= 9 && cur_iface != NULL) {
			cur_iface->has_hid_desc = true;
			uint8_t num_descs = p[5];
			for (uint8_t i = 0; i < num_descs; i++) {
				if (6 + i * 3 + 2 < dlen) {
					uint8_t rtype = p[6 + i * 3];
					uint16_t rlen = p[7 + i * 3] | (p[8 + i * 3] << 8);
					if (rtype == USB_DESC_HID_REPORT) {
						cur_iface->hid_report_desc_len = rlen;
					}
				}
			}
		} else if (dtype == USB_DESC_ENDPOINT && dlen >= 7 && cur_iface != NULL) {
			uint8_t ep_addr = p[2];
			uint8_t ep_attr = p[3];
			uint16_t ep_maxpkt = p[4] | (p[5] << 8);
			uint8_t ep_interval = p[6];
			if ((ep_attr & 3) == 3) {  // Interrupt endpoint (Type bits = 11b)
				if (ep_addr & 0x80) {
					// IN direction (bit 7 set in address)
					cur_iface->interrupt_in_ep       = ep_addr;
					cur_iface->interrupt_in_maxpkt   = ep_maxpkt;
					cur_iface->interrupt_in_interval = ep_interval;
				} else {
					// OUT direction
					cur_iface->interrupt_out_ep       = ep_addr;
					cur_iface->interrupt_out_maxpkt   = ep_maxpkt;
					cur_iface->interrupt_out_interval = ep_interval;
				}
			}
		}

		p += dlen;
	}

	return desc->num_ifaces > 0;
}

static void capture_bos(captured_descriptors_t *desc)
{
	// Probe BOS header (5 bytes: bLength, bDescriptorType, wTotalLength, bNumDeviceCaps)
	usb_setup_t setup = make_get_descriptor(USB_DESC_BOS, 0, 0, 5);
	uint8_t hdr[5];
	int ret = usb_host_control_transfer(desc->dev_addr, desc->ep0_maxpkt,
		&setup, hdr, 2000);

	// A device without BOS support STALLs; not fatal.
	if (ret < 5 || hdr[1] != USB_DESC_BOS) {
		desc->bos_desc_len = 0;
		return;
	}

	uint16_t total_len = hdr[2] | (hdr[3] << 8);
	if (total_len < 5) {
		desc->bos_desc_len = 0;
		return;
	}
	if (total_len > MAX_BOS_DESC_SIZE) total_len = MAX_BOS_DESC_SIZE;

	setup = make_get_descriptor(USB_DESC_BOS, 0, 0, total_len);
	ret = usb_host_control_transfer(desc->dev_addr, desc->ep0_maxpkt,
		&setup, desc->bos_desc, 2000);
	if (ret < 5 || desc->bos_desc[1] != USB_DESC_BOS) {
		desc->bos_desc_len = 0;
		return;
	}
	desc->bos_desc_len = (uint16_t)ret;
}

static void capture_ms_os_1_0(captured_descriptors_t *desc)
{
	// MS OS 1.0 lives at string index 0xEE. Fixed 18-byte response with
	// signature "MSFT100" at offset 2 (UTF-16LE).
	usb_setup_t setup = make_get_descriptor(USB_DESC_STRING, 0xEE, 0,
		MS_OS_1_0_STRING_SIZE);
	uint8_t buf[MS_OS_1_0_STRING_SIZE];
	int ret = usb_host_control_transfer(desc->dev_addr, desc->ep0_maxpkt,
		&setup, buf, 2000);

	if (ret < MS_OS_1_0_STRING_SIZE || buf[1] != USB_DESC_STRING) {
		desc->ms_os_desc_len = 0;
		return;
	}

	// Verify "MSFT100" signature at offset 2 (UTF-16LE: M S F T 1 0 0)
	static const uint8_t sig[] = {
		'M', 0, 'S', 0, 'F', 0, 'T', 0, '1', 0, '0', 0, '0', 0
	};
	if (memcmp(&buf[2], sig, sizeof(sig)) != 0) {
		desc->ms_os_desc_len = 0;
		return;
	}

	memcpy(desc->ms_os_desc, buf, MS_OS_1_0_STRING_SIZE);
	desc->ms_os_desc_len = MS_OS_1_0_STRING_SIZE;
	desc->ms_os_vendor_code = buf[16]; // bMS_VendorCode at offset 0x10
}

// CAP_DBG/CAP_RET were UART-oracle markers ({step, last-ret}) for locating a
// failed control transfer during enumeration; kept as no-ops to leave the
// capture/retry flow undisturbed.
#define CAP_DBG(step)  ((void)0)
#define CAP_RET(r)     ((void)0)

// Wait for the port to settle after a bus reset: poll for a stable CONNECT (>6
// consecutive reads, mirroring WCH's USBH_EnableRootHubPort) before the first
// control transfer, so a device slow to recover from reset is not misreported as
// a hard failure.
static bool enum_wait_port_stable(void)
{
	// Mirror the EVT post-reset enable loop: on each successful connect read,
	// re-read the port speed (caches s_dev_speed for the PRE-PID decision) and
	// re-assert SOF, keeping the port clocked while the device settles.
	uint8_t stable = 0;
	for (uint16_t i = 0; i < 300; i++) {   // ~300 ms worst case
		if (usb_host_device_connected()) {
			(void)usb_host_device_speed();   // EVT: CheckRootHubPortSpeed each pass
			usb_host_power_on();             // EVT: CFG |= SOF_EN each pass
			if (++stable > 6) return true;
		} else {
			stable = 0;
		}
		delay(1);
	}
	return false;
}

bool capture_descriptors(captured_descriptors_t *desc)
{
	memset(desc, 0, sizeof(*desc));
	int ret;
	usb_setup_t setup;
	uint16_t total_len = 0;

	// Enumeration retry loop (ports WCH USBH_EnumRootDevice): up to 6 attempts,
	// each re-resets the port, waits with an escalating settle delay, then runs the
	// address-assignment handshake. The reset returns the device to address 0, so a
	// retry after a partial failure is always clean.
	bool enumerated = false;
	for (uint8_t attempt = 0; attempt < 6 && !enumerated; attempt++) {
		CAP_DBG(0x10 | attempt);   // 0x1N = enum attempt N

		// Escalating settle + re-reset + stability poll, matching the EVT reference
		// (Delay_Ms(100); Delay_Ms(8<<enum_cnt), enum_cnt from 1) for settle times of
		// 116,132,164,228,356,612 ms. A slow-booting device needs this margin after
		// bus reset before the first SETUP.
		delay(100u);                 // EVT: fixed 100 ms device-stabilize
		delay(8u << (attempt + 1));  // EVT: Delay_Ms(8 << enum_cnt), enum_cnt from 1
		usb_host_port_reset();
		if (!enum_wait_port_stable()) {
			CAP_DBG(0x20 | attempt);   // 0x2N = port never stabilized this attempt
			continue;
		}
		(void)usb_host_device_speed();

		// Step 1: GET_DESCRIPTOR(device, 8) @ addr 0 — just bMaxPacketSize0.
		CAP_DBG(1);
		setup = make_get_descriptor(USB_DESC_DEVICE, 0, 0, 8);
		ret = usb_host_control_transfer(0, 8, &setup, desc->device_desc, 2000);
		CAP_RET(ret);
		if (ret < 8 || desc->device_desc[0] != 18 ||
		    desc->device_desc[1] != USB_DESC_DEVICE) {
			continue;   // retry whole enumeration
		}
		desc->ep0_maxpkt = desc->device_desc[7];
		if (desc->ep0_maxpkt != 8  && desc->ep0_maxpkt != 16 &&
		    desc->ep0_maxpkt != 32 && desc->ep0_maxpkt != 64) {
			CAP_DBG(2); CAP_RET(desc->ep0_maxpkt);
			continue;
		}

		// Step 2: SET_ADDRESS(1).
		desc->dev_addr = 1;
		setup.bmRequestType = 0x00;
		setup.bRequest = USB_REQ_SET_ADDRESS;
		setup.wValue = desc->dev_addr;
		setup.wIndex = 0;
		setup.wLength = 0;
		CAP_DBG(3);
		ret = usb_host_control_transfer(0, desc->ep0_maxpkt, &setup, NULL, 2000);
		CAP_RET(ret);
		if (ret < 0) continue;
		delay(5);   // USB spec 2ms recovery, with margin for the device to switch addr

		// Step 3: GET_DESCRIPTOR(device, 18) @ the new address.
		CAP_DBG(4);
		setup = make_get_descriptor(USB_DESC_DEVICE, 0, 0, 18);
		ret = usb_host_control_transfer(desc->dev_addr, desc->ep0_maxpkt,
			&setup, desc->device_desc, 2000);
		CAP_RET(ret);
		if (ret < 18 || desc->device_desc[0] != 18 ||
		    desc->device_desc[1] != USB_DESC_DEVICE) {
			continue;
		}
		desc->device_desc_len = 18;

		// Step 4: GET_DESCRIPTOR(config, 9) — header to learn wTotalLength.
		CAP_DBG(5);
		setup = make_get_descriptor(USB_DESC_CONFIGURATION, 0, 0, 9);
		ret = usb_host_control_transfer(desc->dev_addr, desc->ep0_maxpkt,
			&setup, desc->config_desc, 2000);
		CAP_RET(ret);
		if (ret < 9 || desc->config_desc[1] != USB_DESC_CONFIGURATION) {
			continue;
		}
		total_len = desc->config_desc[2] | (desc->config_desc[3] << 8);
		if (total_len < 9) continue;
		if (total_len > MAX_CONFIG_DESC_SIZE)
			total_len = MAX_CONFIG_DESC_SIZE;

		// Step 5: GET_DESCRIPTOR(config, wTotalLength) — full config blob.
		CAP_DBG(6);
		setup = make_get_descriptor(USB_DESC_CONFIGURATION, 0, 0, total_len);
		ret = usb_host_control_transfer(desc->dev_addr, desc->ep0_maxpkt,
			&setup, desc->config_desc, 2000);
		CAP_RET(ret);
		if (ret < 9 || desc->config_desc[1] != USB_DESC_CONFIGURATION) {
			continue;
		}
		enumerated = true;   // all required steps passed
	}
	if (!enumerated) {
		CAP_DBG(0x2F);   // enumeration exhausted all retries
		return false;
	}
	desc->config_desc_len = (uint16_t)ret;
	desc->config_string_idx = desc->config_desc[6]; // iConfiguration
	parse_config_descriptor(desc);
	for (uint8_t i = 0; i < desc->num_ifaces; i++) {
		captured_iface_t *iface = &desc->ifaces[i];
		if (!iface->has_hid_desc) continue;
		if (iface->hid_report_desc_len == 0) continue;

		uint16_t rdlen = iface->hid_report_desc_len;
		if (rdlen > MAX_HID_REPORT_DESC_SIZE)
			rdlen = MAX_HID_REPORT_DESC_SIZE;

		setup = make_get_iface_descriptor(USB_DESC_HID_REPORT, 0,
			iface->iface_num, rdlen);
		ret = usb_host_control_transfer(desc->dev_addr, desc->ep0_maxpkt,
			&setup, iface->hid_report_desc, 2000);
		if (ret < 0) {
			iface->hid_report_desc_len = 0;
		} else {
			iface->hid_report_desc_len = (uint16_t)ret;
		}
	}

	// Capture the full LANGID descriptor (string index 0), replayed verbatim so
	// the host sees the device's actual language list.
	setup = make_get_descriptor(USB_DESC_STRING, 0, 0, MAX_LANGID_DESC_SIZE);
	ret = usb_host_control_transfer(desc->dev_addr, desc->ep0_maxpkt,
		&setup, desc->langid_desc, 2000);
	desc->langid = 0x0409; // Default if device returns garbage / STALLs
	desc->langid_desc_len = 0;
	if (ret >= 4 && desc->langid_desc[1] == USB_DESC_STRING) {
		desc->langid_desc_len = (uint8_t)ret;
		// If the claimed bLength exceeds the captured length (more LANGIDs than
		// MAX_LANGID_DESC_SIZE holds), clamp it so downstream consumers stay in
		// bounds.
		if (desc->langid_desc[0] > desc->langid_desc_len) {
			desc->langid_desc[0] = desc->langid_desc_len;
		}
		desc->langid = desc->langid_desc[2] | (desc->langid_desc[3] << 8);
	}
	uint16_t langid = desc->langid;  // local alias used by subsequent string fetches
	// Collect every string index referenced by the device, config, and interface
	// descriptors, deduped to avoid fetching the same index twice.
	uint8_t string_indices[3 + 1 + MAX_INTERFACES];
	uint8_t string_indices_count = 0;
	string_indices[string_indices_count++] = desc->device_desc[14]; // iManufacturer
	string_indices[string_indices_count++] = desc->device_desc[15]; // iProduct
	string_indices[string_indices_count++] = desc->device_desc[16]; // iSerialNumber
	string_indices[string_indices_count++] = desc->config_string_idx;
	for (uint8_t i = 0; i < desc->num_ifaces; i++) {
		string_indices[string_indices_count++] = desc->ifaces[i].iface_string_idx;
	}

	desc->num_strings = 0;
	for (uint8_t i = 0; i < string_indices_count; i++) {
		uint8_t idx = string_indices[i];
		if (idx == 0) continue;

		// Dedup: skip if already captured
		bool already = false;
		for (uint8_t j = 0; j < desc->num_strings; j++) {
			if (desc->string_index[j] == idx) { already = true; break; }
		}
		if (already) continue;
		if (desc->num_strings >= MAX_STRINGS) break;

		setup = make_get_descriptor(USB_DESC_STRING, idx,
			langid, MAX_STRING_DESC_SIZE);
		ret = usb_host_control_transfer(desc->dev_addr, desc->ep0_maxpkt,
			&setup, desc->string_desc[desc->num_strings], 2000);
		if (ret > 0) {
			desc->string_desc_len[desc->num_strings] = ret;
			desc->string_index[desc->num_strings] = idx;
			desc->num_strings++;
		}
	}

	setup.bmRequestType = 0x00;
	setup.bRequest = USB_REQ_SET_CONFIG;
	setup.wValue = desc->config_desc[5]; // bConfigurationValue
	setup.wIndex = 0;
	setup.wLength = 0;
	ret = usb_host_control_transfer(desc->dev_addr, desc->ep0_maxpkt,
		&setup, NULL, 2000);
	if (ret < 0) return false;

	capture_bos(desc);
	capture_ms_os_1_0(desc);

	// SET_IDLE for each HID interface — report only on change
	for (uint8_t i = 0; i < desc->num_ifaces; i++) {
		if (!desc->ifaces[i].has_hid_desc) continue;
		setup.bmRequestType = 0x21; // Host-to-Device, Class, Interface
		setup.bRequest = 0x0A;      // HID SET_IDLE
		setup.wValue = 0;           // duration=0, report_id=0
		setup.wIndex = desc->ifaces[i].iface_num;
		setup.wLength = 0;
		usb_host_control_transfer(desc->dev_addr, desc->ep0_maxpkt,
			&setup, NULL, 2000);
		// Ignore errors; some devices legally STALL SET_IDLE.
	}

	desc->valid = true;
	return true;
}

