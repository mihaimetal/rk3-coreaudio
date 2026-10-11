/*
 * codectest.c — unit test for the mode-2 packet codec (no hardware needed).
 *
 * Verifies:
 *   1. capture deinterleave recovers data bytes in order, skips check bytes,
 *      and validates correct check bytes with no panic;
 *   2. a corrupted check byte raises input_panic (and zeroes audio);
 *   3. playback places audio bytes in order and writes correct check bytes
 *      at the expected positions.
 */

#include "../codec_mode2.h"
#include <stdio.h>
#include <string.h>

#define BPP 88   /* 48 kHz / 24-bit, n_streams=1 (matches handshake output) */

static int failures;
#define CHECK(cond, msg) do { \
	if (!(cond)) { printf("  FAIL: %s\n", msg); failures++; } \
} while (0)

static uint8_t checkbyte0(int i) { return (uint8_t)((~(i >> 2)) & 1); }

/* Build a capture-layout packet: [chk][d0][d1][d2] per 4-byte group.
 * Data bytes carry a known counter; returns number of data bytes placed. */
static int build_capture_packet(uint8_t *usb, int len)
{
	int data_count = 0;
	for (int i = 0; i < len; i++) {
		if ((i & 3) == 0)
			usb[i] = checkbyte0(i);            /* valid check byte */
		else
			usb[i] = (uint8_t)(0x10 + data_count++); /* known data */
	}
	return data_count;
}

static void test_capture_roundtrip(void)
{
	printf("test_capture_roundtrip\n");
	uint8_t usb[BPP];
	int ndata = build_capture_packet(usb, BPP);

	uint8_t pcm[4096];
	int pos = 0;
	struct caiaq_codec c;
	caiaq_codec_reset(&c);
	c.first_packet = 0; /* validate immediately */

	caiaq_capture_mode2(&c, usb, BPP, pcm, &pos, sizeof(pcm));

	CHECK(c.input_panic == 0, "no input panic on valid packet");
	CHECK(c.output_panic == 0, "no output panic on valid packet");
	CHECK(pos == ndata, "all data bytes deinterleaved");

	int ok = 1;
	for (int k = 0; k < ndata; k++)
		if (pcm[k] != (uint8_t)(0x10 + k)) { ok = 0; break; }
	CHECK(ok, "data bytes recovered in order");
	printf("  (%d data bytes from %d-byte packet)\n", pos, BPP);
}

static void test_capture_panic(void)
{
	printf("test_capture_panic\n");
	uint8_t usb[BPP];
	build_capture_packet(usb, BPP);
	usb[8] ^= 0x01; /* corrupt a check byte (group at i=8) */

	uint8_t pcm[4096];
	int pos = 0;
	struct caiaq_codec c;
	caiaq_codec_reset(&c);
	c.first_packet = 0;

	caiaq_capture_mode2(&c, usb, BPP, pcm, &pos, sizeof(pcm));
	CHECK(c.input_panic == 1, "corrupted check byte raises input_panic");
	CHECK(c.input_errs == 1, "and counts one bad packet");
	int silent = 1;
	for (int k = 0; k < pos; k++) if (pcm[k]) silent = 0;
	CHECK(silent, "the bad packet is delivered as silence");

	/* the mute must NOT latch: the next good packet decodes normally */
	build_capture_packet(usb, BPP);
	int start = pos, nonzero = 0;
	caiaq_capture_mode2(&c, usb, BPP, pcm, &pos, sizeof(pcm));
	for (int k = start; k < pos; k++) if (pcm[k]) nonzero = 1;
	CHECK(nonzero, "a good packet after a bad one is not muted");
	CHECK(c.input_errs == 1, "and is not counted");
}

static void test_capture_grace(void)
{
	printf("test_capture_grace\n");
	uint8_t usb[BPP], pcm[65536];
	int pos = 0;
	struct caiaq_codec c;
	caiaq_codec_reset(&c);

	/* bad packets inside the start-up grace are muted but not flagged, and the
	 * grace is counted in packets, not bytes */
	for (int n = 0; n < CAIAQ_GRACE_PACKETS; n++) {
		build_capture_packet(usb, BPP);
		usb[4] ^= 0x01;
		caiaq_capture_mode2(&c, usb, BPP, pcm, &pos, sizeof(pcm));
	}
	CHECK(c.input_panic == 0 && c.input_errs == 0,
	      "bad packets during the grace period do not raise input_panic");
	build_capture_packet(usb, BPP);
	usb[4] ^= 0x01;
	caiaq_capture_mode2(&c, usb, BPP, pcm, &pos, sizeof(pcm));
	CHECK(c.input_panic == 1, "the first bad packet after the grace does");
}

static void test_playback_layout(void)
{
	printf("test_playback_layout\n");
	uint8_t pcm[4096];
	for (int k = 0; k < (int)sizeof(pcm); k++)
		pcm[k] = (uint8_t)(0x80 + (k & 0x3f));

	uint8_t usb[BPP];
	memset(usb, 0xAA, sizeof(usb));
	int pos = 0;
	struct caiaq_codec c;
	caiaq_codec_reset(&c);

	caiaq_playback_mode2(&c, usb, BPP, pcm, &pos, sizeof(pcm));

	/* Re-walk the same index pattern to classify each byte. */
	int i = 0, data_idx = 0, ok_data = 1, ok_check = 1, ndata = 0;
	while (i < BPP) {
		/* data byte */
		if (usb[i] != (uint8_t)(0x80 + (data_idx & 0x3f))) ok_data = 0;
		data_idx++; ndata++; i++;
		if (i < BPP && (i & 3) == 2) {
			if (usb[i] != checkbyte0(i)) ok_check = 0;
			i++;
		}
	}
	CHECK(ok_data, "playback audio bytes placed in order");
	CHECK(ok_check, "playback check bytes correct");
	CHECK(pos == ndata, "pcm read position advanced by data byte count");
	printf("  (%d audio bytes into %d-byte packet)\n", pos, BPP);
}

int main(void)
{
	printf("=== mode-2 codec unit tests ===\n");
	test_capture_roundtrip();
	test_capture_panic();
	test_capture_grace();
	test_playback_layout();

	if (failures == 0)
		printf("\nALL TESTS PASSED\n");
	else
		printf("\n%d FAILURE(S)\n", failures);
	return failures ? 1 : 0;
}
