/*
 * codec_mode2.h — RigKontrol 3 audio packet (de)interleave, data_alignment==2.
 *
 * The RK3 reports data_alignment=2 (confirmed via the handshake tool), so we
 * port exactly two byte-loops from the Linux caiaq driver:
 *   capture  -> read_in_urb_mode2   (audio.c:405)
 *   playback -> fill_out_urb_mode_0 (audio.c:540, used for alignment 0 and 2)
 *
 * The RK3 has 1 stereo stream (n_streams=1). On the wire, audio samples are
 * 24-bit big-endian (S24_3BE = 3 bytes/sample) carried alongside per-group
 * check bytes that validate stream/channel sync:
 *
 *   capture  packet, per 4-byte group:  [check][d0][d1][d2]
 *   playback packet, per 4-byte group:  [d0][d1][check][d2]   (mode_0 layout)
 *
 * The PCM side is a stereo-interleaved S24_3BE ring: L0 L1 L2 R0 R1 R2 ...
 * Functions advance *pcm_pos modulo pcm_size, mirroring the ALSA dma ring.
 */

#ifndef CODEC_MODE2_H
#define CODEC_MODE2_H

#include <stdint.h>

#define CAIAQ_GRACE_PACKETS 32

struct caiaq_codec {
	int first_packet;   /* capture PACKETS left in the start-up grace period */
	int input_panic;    /* a check byte mismatched after the grace period (sticky,
	                     * diagnostic only — it no longer mutes the stream) */
	unsigned input_errs; /* packets muted for a bad check byte after the grace */
	int output_panic;   /* device signalled output underrun (bit 7) */
	int pb_chk;         /* playback check-byte position within each 4-byte group
	                     * (Linux fill_out_urb_mode_0 uses 2; experiment: 0). */
};

static inline void caiaq_codec_reset(struct caiaq_codec *c)
{
	/*
	 * Grace period, in PACKETS (Linux: first_packet = 4, decremented once per
	 * completed URB). The RK3's first one or two packets after a sample-rate
	 * change carry misaligned data — measured over 8 starts: every start that
	 * changed the rate to 192 kHz had 16-39 bad check bytes, all in packets
	 * 1-2, and none afterwards. This used to count down per audio BYTE, so the
	 * grace ended 4 bytes into the first packet, input_panic latched and the
	 * whole session captured silence.
	 */
	c->first_packet = CAIAQ_GRACE_PACKETS;
	c->input_panic = 0;
	c->input_errs = 0;
	c->output_panic = 0;
	c->pb_chk = 2;      /* Linux default */
}

/*
 * Deinterleave one captured isoc packet into the PCM ring.
 * usb/usb_len: raw bytes from EP 0x82. pcm/pcm_size: stereo S24_3BE ring.
 * A packet with a bad check byte is delivered as silence — that packet only:
 * the check sequence restarts in every packet, so one damaged packet says
 * nothing about the next. (Linux instead flags an xrun and ALSA restarts the
 * stream; latching the mute, as this port first did, silenced capture for the
 * rest of the session.) After the grace period such a packet also sets
 * input_panic and counts in input_errs.
 */
void caiaq_capture_mode2(struct caiaq_codec *c,
			 const uint8_t *usb, int usb_len,
			 uint8_t *pcm, int *pcm_pos, int pcm_size);

/*
 * Build one playback isoc packet (EP 0x06) from the PCM ring, inserting the
 * mode_0 check bytes. usb_len is the packet size to fill (bpp). If pcm is
 * NULL the audio bytes are filled with silence (check bytes still written).
 */
void caiaq_playback_mode2(struct caiaq_codec *c,
			  uint8_t *usb, int usb_len,
			  const uint8_t *pcm, int *pcm_pos, int pcm_size);

#endif /* CODEC_MODE2_H */
