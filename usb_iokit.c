/* usb_iokit.c — see usb_iokit.h */

#include "usb_iokit.h"

#include <stdio.h>
#include <string.h>
#include <CoreFoundation/CoreFoundation.h>
#include <IOKit/IOKitLib.h>
#include <IOKit/IOCFPlugIn.h>

#define STREAMING_ALT 1

/* ---- device lookup -------------------------------------------------- */

static io_service_t find_device(void)
{
	CFMutableDictionaryRef matching =
		IOServiceMatching(kIOUSBDeviceClassName);
	if (!matching)
		return IO_OBJECT_NULL;

	SInt32 vid = RK3_VID, pid = RK3_PID;
	CFNumberRef vidRef = CFNumberCreate(NULL, kCFNumberSInt32Type, &vid);
	CFNumberRef pidRef = CFNumberCreate(NULL, kCFNumberSInt32Type, &pid);
	CFDictionarySetValue(matching, CFSTR(kUSBVendorID), vidRef);
	CFDictionarySetValue(matching, CFSTR(kUSBProductID), pidRef);
	CFRelease(vidRef);
	CFRelease(pidRef);

	io_iterator_t iter = 0;
	if (IOServiceGetMatchingServices(kIOMainPortDefault, matching,
					 &iter) != KERN_SUCCESS)
		return IO_OBJECT_NULL;

	io_service_t svc = IOIteratorNext(iter);
	IOObjectRelease(iter);
	return svc;
}

static IOUSBDeviceInterface300 **open_device(io_service_t svc)
{
	IOCFPlugInInterface **plugIn = NULL;
	IOUSBDeviceInterface300 **dev = NULL;
	SInt32 score = 0;

	if (IOCreatePlugInInterfaceForService(svc,
			kIOUSBDeviceUserClientTypeID, kIOCFPlugInInterfaceID,
			&plugIn, &score) != kIOReturnSuccess || !plugIn)
		return NULL;

	(*plugIn)->QueryInterface(plugIn,
			CFUUIDGetUUIDBytes(kIOUSBDeviceInterfaceID300),
			(LPVOID *)&dev);
	IODestroyPlugInInterface(plugIn);
	return dev;
}

/* ---- interface + pipe resolution ------------------------------------ */

static IOUSBInterfaceInterface300 **open_interface(IOUSBDeviceInterface300 **dev)
{
	IOUSBFindInterfaceRequest req = {
		.bInterfaceClass    = kIOUSBFindInterfaceDontCare,
		.bInterfaceSubClass = kIOUSBFindInterfaceDontCare,
		.bInterfaceProtocol = kIOUSBFindInterfaceDontCare,
		.bAlternateSetting  = kIOUSBFindInterfaceDontCare,
	};
	io_iterator_t iter = 0;
	if ((*dev)->CreateInterfaceIterator(dev, &req, &iter) != kIOReturnSuccess)
		return NULL;

	io_service_t svc = IOIteratorNext(iter);
	IOObjectRelease(iter);
	if (!svc)
		return NULL;

	IOCFPlugInInterface **plugIn = NULL;
	IOUSBInterfaceInterface300 **intf = NULL;
	SInt32 score = 0;

	if (IOCreatePlugInInterfaceForService(svc,
			kIOUSBInterfaceUserClientTypeID, kIOCFPlugInInterfaceID,
			&plugIn, &score) == kIOReturnSuccess && plugIn) {
		(*plugIn)->QueryInterface(plugIn,
			CFUUIDGetUUIDBytes(kIOUSBInterfaceInterfaceID300),
			(LPVOID *)&intf);
		IODestroyPlugInInterface(plugIn);
	}
	IOObjectRelease(svc);
	return intf;
}

/* Walk the pipes of the current alt setting and fill in pipe refs. */
static int resolve_pipes(struct rk3_usb *u)
{
	UInt8 nEP = 0;
	(*u->intf)->GetNumEndpoints(u->intf, &nEP);

	for (UInt8 pipe = 1; pipe <= nEP; pipe++) {
		UInt8 dir = 0, num = 0, type = 0, interval = 0;
		UInt16 maxPacket = 0;

		if ((*u->intf)->GetPipeProperties(u->intf, pipe, &dir, &num,
				&type, &maxPacket, &interval) != kIOReturnSuccess)
			continue;

		if (type == kUSBBulk && dir == kUSBOut && num == 1)
			u->pipe_cmd_out = pipe;
		else if (type == kUSBBulk && dir == kUSBIn && num == 1)
			u->pipe_cmd_in = pipe;
		else if (type == kUSBIsoc && dir == kUSBIn && num == 2) {
			u->pipe_iso_in = pipe;
			u->iso_in_maxpacket = maxPacket;
		} else if (type == kUSBIsoc && dir == kUSBOut && num == 6) {
			u->pipe_iso_out = pipe;
			u->iso_out_maxpacket = maxPacket;
		}
	}

	if (!u->pipe_cmd_out || !u->pipe_cmd_in ||
	    !u->pipe_iso_in || !u->pipe_iso_out) {
		fprintf(stderr, "resolve_pipes: missing pipe(s) "
			"(cmd_out=%u cmd_in=%u iso_in=%u iso_out=%u)\n",
			u->pipe_cmd_out, u->pipe_cmd_in,
			u->pipe_iso_in, u->pipe_iso_out);
		return -1;
	}
	return 0;
}

/* ---- public API ----------------------------------------------------- */

int rk3_usb_open(struct rk3_usb *u)
{
	IOReturn kr;

	memset(u, 0, sizeof(*u));

	io_service_t svc = find_device();
	if (!svc) {
		fprintf(stderr, "rk3_usb_open: device not found "
			"(VID 0x%04x PID 0x%04x) — plug in the RK3\n",
			RK3_VID, RK3_PID);
		return -1;
	}

	IORegistryEntryGetRegistryEntryID(svc, &u->entry_id);
	u->dev = open_device(svc);
	IOObjectRelease(svc);
	if (!u->dev) {
		fprintf(stderr, "rk3_usb_open: open_device failed\n");
		return -1;
	}

	kr = (*u->dev)->USBDeviceOpen(u->dev);
	if (kr != kIOReturnSuccess) {
		fprintf(stderr, "rk3_usb_open: USBDeviceOpen 0x%08x "
			"(claimed by another process?)\n", kr);
		goto fail;
	}

	/* Configure (interface nubs only appear once configured). */
	IOUSBConfigurationDescriptorPtr cfg = NULL;
	if ((*u->dev)->GetConfigurationDescriptorPtr(u->dev, 0, &cfg)
			!= kIOReturnSuccess || !cfg) {
		fprintf(stderr, "rk3_usb_open: GetConfigurationDescriptorPtr failed\n");
		goto fail;
	}
	kr = (*u->dev)->SetConfiguration(u->dev, cfg->bConfigurationValue);
	if (kr != kIOReturnSuccess) {
		fprintf(stderr, "rk3_usb_open: SetConfiguration 0x%08x\n", kr);
		goto fail;
	}

	u->intf = open_interface(u->dev);
	if (!u->intf) {
		fprintf(stderr, "rk3_usb_open: open_interface failed\n");
		goto fail;
	}

	kr = (*u->intf)->USBInterfaceOpen(u->intf);
	if (kr != kIOReturnSuccess) {
		fprintf(stderr, "rk3_usb_open: USBInterfaceOpen 0x%08x\n", kr);
		goto fail;
	}

	/* Switch to the streaming alt setting (mirrors usb_set_interface(0,1)). */
	kr = (*u->intf)->SetAlternateInterface(u->intf, STREAMING_ALT);
	if (kr != kIOReturnSuccess) {
		fprintf(stderr, "rk3_usb_open: SetAlternateInterface(%d) 0x%08x\n",
			STREAMING_ALT, kr);
		goto fail;
	}

	if (resolve_pipes(u) != 0)
		goto fail;

	return 0;

fail:
	rk3_usb_close(u);
	return -1;
}

void rk3_usb_close(struct rk3_usb *u)
{
	if (u->intf) {
		(*u->intf)->USBInterfaceClose(u->intf);
		(*u->intf)->Release(u->intf);
		u->intf = NULL;
	}
	if (u->dev) {
		(*u->dev)->USBDeviceClose(u->dev);
		(*u->dev)->Release(u->dev);
		u->dev = NULL;
	}
	u->entry_id = 0;
}

int rk3_usb_present(const struct rk3_usb *u)
{
	if (!u->entry_id)
		return 0;
	/* IORegistryEntryIDMatching's dictionary is consumed by the call. */
	io_service_t svc = IOServiceGetMatchingService(kIOMainPortDefault,
			IORegistryEntryIDMatching(u->entry_id));
	if (!svc)
		return 0;
	IOObjectRelease(svc);
	return 1;
}

void rk3_clear_stall(struct rk3_usb *u, uint8_t pipe)
{
	if (u->intf && pipe)
		(*u->intf)->ClearPipeStallBothEnds(u->intf, pipe);
}

int rk3_bulk_write(struct rk3_usb *u, const void *buf, size_t len,
		   unsigned timeout_ms)
{
	IOReturn kr = (*u->intf)->WritePipeTO(u->intf, u->pipe_cmd_out,
			(void *)buf, (UInt32)len, timeout_ms, timeout_ms);
	if (kr == kIOUSBPipeStalled) {
		/* A rejected/unsupported command halts the pipe; clear it so the
		 * device isn't wedged for every later command, then retry once. */
		rk3_clear_stall(u, u->pipe_cmd_out);
		kr = (*u->intf)->WritePipeTO(u->intf, u->pipe_cmd_out,
				(void *)buf, (UInt32)len, timeout_ms, timeout_ms);
		/* If the command is genuinely rejected the retry re-stalls; clear
		 * again so the NEXT command starts on a clean (un-halted) pipe. */
		if (kr == kIOUSBPipeStalled)
			rk3_clear_stall(u, u->pipe_cmd_out);
	}
	if (kr != kIOReturnSuccess) {
		fprintf(stderr, "rk3_bulk_write: 0x%08x\n", kr);
		return -1;
	}
	return 0;
}

int rk3_bulk_read(struct rk3_usb *u, void *buf, size_t *len,
		  unsigned timeout_ms)
{
	UInt32 size = (UInt32)*len;
	IOReturn kr = (*u->intf)->ReadPipeTO(u->intf, u->pipe_cmd_in,
			buf, &size, timeout_ms, timeout_ms);
	if (kr == kIOReturnTimeout || kr == kIOUSBTransactionTimeout)
		return RK3_ETIMEOUT;
	if (kr == kIOUSBPipeStalled) {
		rk3_clear_stall(u, u->pipe_cmd_in);
		return RK3_ETIMEOUT; /* treat as "no data" so callers retry */
	}
	if (kr != kIOReturnSuccess) {
		fprintf(stderr, "rk3_bulk_read: 0x%08x\n", kr);
		return -1;
	}
	*len = size;
	return 0;
}
