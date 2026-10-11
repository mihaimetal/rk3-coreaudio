/*
 * usb_iokit.h — IOKit/IOUSBLib transport for the RigKontrol 3.
 *
 * Opens the device (VID 0x17cc / PID 0x1940), sets configuration 1,
 * opens interface #0 at alternate setting 1 (the streaming alt that
 * exposes the isoc endpoints), and resolves the four pipe references:
 *
 *   cmd_out  -> EP 0x01 OUT bulk   (commands / MIDI out)
 *   cmd_in   -> EP 0x81 IN  bulk   (replies / MIDI in)
 *   iso_in   -> EP 0x82 IN  isoc   (audio capture)
 *   iso_out  -> EP 0x06 OUT isoc   (audio playback)
 *
 * Pipe references are discovered by matching (endpoint number, direction,
 * transfer type) rather than hard-coded indices, so the layer is robust
 * to descriptor ordering changes.
 *
 * This replaces the Linux URB transport in the caiaq driver. Bulk helpers
 * are synchronous (good enough for the EP1 command channel). Isoc streaming
 * is added in a later step and uses the IOUSBInterfaceInterface isoc calls.
 */

#ifndef RK3_USB_IOKIT_H
#define RK3_USB_IOKIT_H

#include <IOKit/usb/IOUSBLib.h>
#include <stddef.h>
#include <stdint.h>

#define RK3_VID 0x17cc
#define RK3_PID 0x1940

struct rk3_usb {
	IOUSBDeviceInterface300    **dev;
	IOUSBInterfaceInterface300 **intf;

	/* IOKit pipe reference numbers (1-based) for the streaming alt. */
	uint8_t pipe_cmd_out;   /* EP 0x01 OUT bulk */
	uint8_t pipe_cmd_in;    /* EP 0x81 IN  bulk */
	uint8_t pipe_iso_in;    /* EP 0x82 IN  isoc */
	uint8_t pipe_iso_out;   /* EP 0x06 OUT isoc */

	uint16_t iso_in_maxpacket;
	uint16_t iso_out_maxpacket;

	uint64_t entry_id;      /* IORegistry entry ID of the device we opened */
};

/*
 * 1 while the device instance this handle was opened on is still attached.
 * A re-plugged RK3 is a NEW registry entry, so this goes to 0 on unplug and
 * stays 0 — the caller must close and reopen. Cheap; safe from any thread.
 */
int rk3_usb_present(const struct rk3_usb *u);

/*
 * Find, open, configure the RK3 and open interface #0 at alt 1.
 * Returns 0 on success, negative on error (message printed to stderr).
 */
int  rk3_usb_open(struct rk3_usb *u);

/* Close interface + device and release everything. Safe on a zeroed struct. */
void rk3_usb_close(struct rk3_usb *u);

/* Synchronous bulk write to EP1 OUT. Returns 0 on success. */
int  rk3_bulk_write(struct rk3_usb *u, const void *buf, size_t len,
		    unsigned timeout_ms);

/*
 * Synchronous bulk read from EP1 IN. *len is in/out: pass buffer capacity,
 * receives actual bytes read. Returns 0 on success, -2 (RK3_ETIMEOUT) on
 * timeout (no data) so callers can poll/retry.
 */
#define RK3_ETIMEOUT (-2)
int  rk3_bulk_read(struct rk3_usb *u, void *buf, size_t *len,
		   unsigned timeout_ms);

/* Clear a halted (stalled) bulk pipe on both ends. Pass a pipe ref from
 * struct rk3_usb (e.g. u->pipe_cmd_out). No-op on a null pipe/interface. */
void rk3_clear_stall(struct rk3_usb *u, uint8_t pipe);

#endif /* RK3_USB_IOKIT_H */
