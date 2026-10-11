/* caiaq_protocol.c — see caiaq_protocol.h */

#include <mach/mach_time.h>
#include "caiaq_protocol.h"

#include <stdio.h>
#include <string.h>

/* Matches CLOCK_DRIFT_TOLERANCE / CHANNELS_PER_STREAM / sample sizing in audio.c. */
#define CLOCK_DRIFT_TOLERANCE 5
#define CHANNELS_PER_STREAM   2
#define BYTES_PER_SAMPLE      3
#define MAX_ENDPOINT_SIZE     512

int caiaq_send_command(struct rk3_usb *u, uint8_t command,
		       const uint8_t *buffer, int len)
{
	uint8_t out[EP1_BUFSIZE];

	if (len > EP1_BUFSIZE - 1)
		len = EP1_BUFSIZE - 1;

	out[0] = command;
	if (buffer && len > 0)
		memcpy(out + 1, buffer, len);

	return rk3_bulk_write(u, out, len + 1, 200);
}

/*
 * Read EP1 IN until we see a reply whose first byte == want_cmd, or we run
 * out of attempts. The device may interleave unsolicited messages (MIDI,
 * analog/ERP) once auto-messages are on, so we filter by command byte.
 */
#define REPLY_WINDOW_MS 250

static uint64_t now_ms(void)
{
	static mach_timebase_info_data_t tb;
	if (tb.denom == 0) mach_timebase_info(&tb);
	return mach_absolute_time() * tb.numer / tb.denom / 1000000ull;
}

static int read_reply(struct rk3_usb *u, uint8_t want_cmd,
		      uint8_t *buf, size_t cap, size_t *out_len)
{
	/*
	 * Bounded by TIME, not by packet count. The device answers a command in
	 * ~15 ms (measured, incl. AUDIO_PARAMS across every rate change), but it
	 * also sends unsolicited 0x03 control reports on this pipe whenever a pedal
	 * ADC flickers; counting packets let a trickle of those either end the wait
	 * early or stretch it to many seconds. REPLY_WINDOW_MS is the whole budget.
	 */
	uint64_t deadline = now_ms() + REPLY_WINDOW_MS;
	for (;;) {
		uint64_t now = now_ms();
		if (now >= deadline)
			return -1;
		size_t len = cap;
		int rc = rk3_bulk_read(u, buf, &len, (unsigned)(deadline - now));
		if (rc == RK3_ETIMEOUT)
			return -1; /* no reply inside the window */
		if (rc != 0)
			return -1;
		if (len >= 1 && buf[0] == want_cmd) {
			*out_len = len;
			return 0;
		}
		/* else: unsolicited message, keep reading */
	}
}

int caiaq_get_device_info(struct rk3_usb *u, struct caiaq_device_spec *spec)
{
	uint8_t buf[EP1_BUFSIZE];
	size_t len = 0;
	int rc;

	/*
	 * Ask up to three times. The RK3 does not reset its EP 0x81 data toggle when
	 * the host does (see the note in rk3_engine_audio_start), and re-opening the
	 * interface after another owner used it resets the host's. When the two
	 * disagree the host silently discards exactly one packet — the reply to the
	 * first command — after which they agree again and the second request is
	 * answered. Measured: first request after such a re-open 0/1, the next
	 * 20/20. Without the retry that state needed a physical replug.
	 */
	for (int attempt = 0; attempt < 3; attempt++) {
		rc = caiaq_send_command(u, EP1_CMD_GET_DEVICE_INFO, NULL, 0);
		if (rc != 0)
			return rc;
		rc = read_reply(u, EP1_CMD_GET_DEVICE_INFO, buf, sizeof(buf), &len);
		if (rc == 0)
			break;
	}
	if (rc != 0) {
		fprintf(stderr, "caiaq_get_device_info: no reply\n");
		return -1;
	}

	if (len < 1 + sizeof(*spec)) {
		fprintf(stderr, "caiaq_get_device_info: short reply (%zu bytes)\n",
			len);
		return -1;
	}

	/* Payload starts at buf[1]; struct is packed and matches wire layout. */
	memcpy(spec, buf + 1, sizeof(*spec));
	/* fw_version is little-endian on the wire; macOS arm64 is LE too. */
	return 0;
}

int caiaq_compute_bpp(int rate, int data_alignment, int n_streams)
{
	int bytes_per_sample = BYTES_PER_SAMPLE;
	int bpp;

	if (data_alignment >= 2)
		bytes_per_sample++;

	bpp = ((rate / 8000) + CLOCK_DRIFT_TOLERANCE)
		* bytes_per_sample * CHANNELS_PER_STREAM * n_streams;

	if (bpp > MAX_ENDPOINT_SIZE)
		bpp = MAX_ENDPOINT_SIZE;

	return bpp;
}

int caiaq_set_audio_params(struct rk3_usb *u, int rate, int depth, int bpp)
{
	uint8_t tmp[5];
	uint8_t buf[EP1_BUFSIZE];
	size_t len = 0;
	int rc;

	switch (rate) {
	case 44100:  tmp[0] = CAIAQ_SR_44100;  break;
	case 48000:  tmp[0] = CAIAQ_SR_48000;  break;
	case 88200:  tmp[0] = CAIAQ_SR_88200;  break;
	case 96000:  tmp[0] = CAIAQ_SR_96000;  break;
	case 192000: tmp[0] = CAIAQ_SR_192000; break;
	default:
		fprintf(stderr, "caiaq_set_audio_params: bad rate %d\n", rate);
		return -1;
	}

	switch (depth) {
	case 16: tmp[1] = CAIAQ_DEPTH_16; break;
	case 24: tmp[1] = CAIAQ_DEPTH_24; break;
	default:
		fprintf(stderr, "caiaq_set_audio_params: bad depth %d\n", depth);
		return -1;
	}

	tmp[2] = bpp & 0xff;
	tmp[3] = (bpp >> 8) & 0xff;
	tmp[4] = 1; /* packets per microframe */

	rc = caiaq_send_command(u, EP1_CMD_AUDIO_PARAMS, tmp, sizeof(tmp));
	if (rc != 0)
		return rc;

	rc = read_reply(u, EP1_CMD_AUDIO_PARAMS, buf, sizeof(buf), &len);
	if (rc != 0) {
		fprintf(stderr, "caiaq_set_audio_params: no ack\n");
		return -1;
	}

	/* buf[1] is the answer byte; 1 == accepted (device.c:303). */
	if (len < 2 || buf[1] != 1) {
		fprintf(stderr, "caiaq_set_audio_params: device rejected "
			"(answer=%d)\n", len >= 2 ? buf[1] : -1);
		return -1;
	}

	return 0;
}

int caiaq_set_direct_mon(struct rk3_usb *u, uint8_t mode)
{
	/* setDirectMonitoringMode (NI kext): EP1 0x0e, one-byte payload. */
	return caiaq_send_command(u, EP1_CMD_DIRECT_MON, &mode, 1);
}

int caiaq_get_direct_mon(struct rk3_usb *u, uint8_t *mode_out)
{
	uint8_t buf[EP1_BUFSIZE];
	size_t len = 0;
	int rc;

	rc = caiaq_send_command(u, EP1_CMD_GET_DIRECT_MON, NULL, 0);
	if (rc != 0)
		return rc;

	rc = read_reply(u, EP1_CMD_GET_DIRECT_MON, buf, sizeof(buf), &len);
	if (rc != 0) {
		fprintf(stderr, "caiaq_get_direct_mon: no reply\n");
		return -1;
	}

	/* buf[0] echoes the cmd; buf[1] is the current mode byte. */
	if (len < 2)
		return -1;
	if (mode_out)
		*mode_out = buf[1];
	return 0;
}
