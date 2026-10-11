/*
 * codec_mode2.c — see codec_mode2.h
 *
 * n_streams is fixed at 1 for the RK3, which lets us specialise the Linux
 * loops. The check-byte value for stream 0 (MAKE_CHECKBYTE, device.c) is:
 *     (stream << 1) | (~(i / (n_streams * BYTES_PER_SAMPLE_USB)) & 1)
 * with stream=0, n_streams=1, BYTES_PER_SAMPLE_USB=4  ->  (~(i >> 2)) & 1.
 */

#include "codec_mode2.h"

/* check byte value for stream 0 at packet-local byte index i */
static inline uint8_t checkbyte0(int i)
{
	return (uint8_t)((~(i >> 2)) & 1);
}

void caiaq_capture_mode2(struct caiaq_codec *c,
			 const uint8_t *usb, int usb_len,
			 uint8_t *pcm, int *pcm_pos, int pcm_size)
{
	/* Validate the whole packet first: check byte at the start of every 4-byte
	 * group (read_in_urb_mode2). */
	int bad = 0, out_panic = 0;
	for (int i = 0; i < usb_len; i += 4) {
		if ((usb[i] & 0x3f) != checkbyte0(i))
			bad = 1;
		if (usb[i] & 0x80)
			out_panic = 1;
	}
	if (c->first_packet) {
		c->first_packet--;            /* start-up grace: mute, don't flag */
	} else {
		if (bad) {
			c->input_panic = 1;
			c->input_errs++;
		}
		if (out_panic)
			c->output_panic = 1;
	}

	for (int i = 0; i < usb_len; i++) {
		if ((i & 3) == 0)
			continue;                 /* check byte */
		if (pcm) {
			pcm[*pcm_pos] = bad ? 0 : usb[i];
			*pcm_pos += 1;
			if (*pcm_pos == pcm_size)
				*pcm_pos = 0;
		}
	}
}

void caiaq_playback_mode2(struct caiaq_codec *c,
			  uint8_t *usb, int usb_len,
			  const uint8_t *pcm, int *pcm_pos, int pcm_size)
{
	int i = 0;
	int chk = c->pb_chk & 3;   /* check-byte slot within each 4-byte group */

	while (i < usb_len) {
		/* check byte at the configured slot (Linux mode_0: slot 2) */
		if ((i & 3) == chk) {
			usb[i] = checkbyte0(i);
			i++;
			continue;
		}
		/* one audio data byte (fill_out_urb_mode_0) */
		if (pcm) {
			usb[i] = pcm[*pcm_pos];
			*pcm_pos += 1;
			if (*pcm_pos == pcm_size)
				*pcm_pos = 0;
		} else {
			usb[i] = 0;
		}
		i++;
	}
}
