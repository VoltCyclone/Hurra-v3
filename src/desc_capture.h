#pragma once
#include <stdint.h>
#include <stdbool.h>

#define MAX_CONFIG_DESC_SIZE    512
#define MAX_HID_REPORT_DESC_SIZE 512
#define MAX_STRING_DESC_SIZE    128
#define MAX_STRINGS             16
#define MAX_INTERFACES          8
#define MAX_BOS_DESC_SIZE       256
#define MAX_LANGID_DESC_SIZE    8
#define MS_OS_1_0_STRING_SIZE   18  // Fixed: bLength=0x12, signature "MSFT100", vendor code byte
typedef struct {
	uint8_t  iface_num;            // bInterfaceNumber
	uint8_t  iface_class;          // bInterfaceClass (3 = HID)
	uint8_t  iface_subclass;       // bInterfaceSubClass
	uint8_t  iface_protocol;       // bInterfaceProtocol
	uint8_t  iface_string_idx;     // iInterface string index (0 if none)
	uint8_t  interrupt_in_ep;       // IN EP addr (0x81 etc), 0 if none
	uint16_t interrupt_in_maxpkt;   // max packet size for interrupt IN EP
	uint8_t  interrupt_in_interval; // polling interval for IN
	uint8_t  interrupt_out_ep;      // OUT EP addr (0x01 etc), 0 if none
	uint16_t interrupt_out_maxpkt;  // max packet size for interrupt OUT EP
	uint8_t  interrupt_out_interval;// polling interval for OUT
	bool     has_hid_desc;         // true if interface contained a HID descriptor
	uint8_t  hid_report_desc[MAX_HID_REPORT_DESC_SIZE];
	uint16_t hid_report_desc_len;  // 0 if not HID or not fetched
} captured_iface_t;

typedef struct {
	uint8_t  device_desc[18];
	uint8_t  device_desc_len;
	uint8_t  config_desc[MAX_CONFIG_DESC_SIZE];
	uint16_t config_desc_len;
	uint8_t  config_string_idx;                     // iConfiguration

	captured_iface_t ifaces[MAX_INTERFACES];
	uint8_t  num_ifaces;

	uint8_t  string_desc[MAX_STRINGS][MAX_STRING_DESC_SIZE];
	uint8_t  string_desc_len[MAX_STRINGS];
	uint8_t  string_index[MAX_STRINGS];             // Original USB string index for each
	uint8_t  num_strings;

	uint8_t  langid_desc[MAX_LANGID_DESC_SIZE];     // String-0 LANGID table, captured verbatim
	uint8_t  langid_desc_len;                       // 0 if capture failed (replay falls back)
	uint16_t langid;                                // First language ID parsed from langid_desc

	uint8_t  bos_desc[MAX_BOS_DESC_SIZE];           // BOS descriptor blob, replayed verbatim
	uint16_t bos_desc_len;                          // 0 if device has no BOS (USB 2.0)

	uint8_t  ms_os_desc[MS_OS_1_0_STRING_SIZE];     // MS OS 1.0 string at index 0xEE
	uint8_t  ms_os_desc_len;                        // 0 if device has no MS_OS_1.0
	uint8_t  ms_os_vendor_code;                     // Byte at offset 0x10 of ms_os_desc

	uint8_t  ep0_maxpkt;
	uint8_t  dev_addr;
	uint8_t  speed;                                 // USB_SPEED_FULL/LOW/HIGH — the
	                                                // captured device's link speed,
	                                                // so the clone presents at the
	                                                // SAME speed (faithful MITM).
	bool     valid;
} captured_descriptors_t;

bool capture_descriptors(captured_descriptors_t *desc);
