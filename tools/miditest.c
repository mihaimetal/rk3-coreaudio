/*
 * miditest.c — unit test for the pedal -> MIDI scaling in rk3_midi.c (no
 * hardware, no CoreMIDI traffic). Includes rk3_midi.c directly so the static
 * decoder is reachable, and captures what it would have sent.
 */
#include <CoreMIDI/CoreMIDI.h>
#include <stdio.h>
#include <string.h>

static unsigned char g_cap[4096];
static int g_cap_n;
static OSStatus capture(MIDIEndpointRef src, const MIDIPacketList *l)
{
	(void)src;
	const MIDIPacket *p = &l->packet[0];
	for (UInt32 i = 0; i < l->numPackets; i++, p = MIDIPacketNext(p))
		for (int k = 0; k < p->length && g_cap_n < (int)sizeof(g_cap); k++)
			g_cap[g_cap_n++] = p->data[k];
	return noErr;
}
#define MIDIReceived capture
#include "../rk3_midi.c"

static int failures;
#define CHECK(c, msg) do { if (!(c)) { printf("  FAIL: %s\n", msg); failures++; } } while (0)

/* feed one analog report: exp1, exp2, treadle (big-endian 16-bit each) */
static void feed(struct rk3_midi *m, int exp1, int exp2, int treadle)
{
	uint8_t p[6] = { (uint8_t)(exp1 >> 8), (uint8_t)exp1, (uint8_t)(exp2 >> 8),
			 (uint8_t)exp2, (uint8_t)(treadle >> 8), (uint8_t)treadle };
	handle_analog(m, p, sizeof(p), 0);
}

int main(void)
{
	static struct rk3_midi m;

	printf("test_lores_default\n");
	memset(&m, 0, sizeof(m));
	cfg_defaults(&m.cfg);
	m.calib_axis = -1;
	g_cap_n = 0;
	feed(&m, 0, 0, 40);
	feed(&m, 0, 0, 270);
	/* first report sends all three axes; last 3 bytes are the treadle at max */
	CHECK(g_cap_n >= 3 && g_cap[g_cap_n - 3] == 0xB0 && g_cap[g_cap_n - 2] == 11 &&
	      g_cap[g_cap_n - 1] == 127, "7-bit treadle reaches CC11 = 127");

	printf("test_hires_range_and_pairing\n");
	memset(&m, 0, sizeof(m));
	cfg_defaults(&m.cfg);
	m.calib_axis = -1;
	m.cfg.axis[2].hires = 1;              /* treadle, cc 11 -> LSB on 43 */
	feed(&m, 0, 0, 40);
	g_cap_n = 0;
	int distinct = 0, last = -1, monotonic = 1, paired = 1;
	for (int raw = 40; raw <= 270; raw++) {
		int before = g_cap_n;
		feed(&m, 0, 0, raw);
		if (g_cap_n == before)
			continue;
		if (g_cap_n - before != 6 || g_cap[before] != 0xB0 ||
		    g_cap[before + 1] != 11 || g_cap[before + 3] != 0xB0 ||
		    g_cap[before + 4] != 43)
			paired = 0;
		int v = (g_cap[before + 2] << 7) | g_cap[before + 5];
		if (v <= last) monotonic = 0;
		last = v;
		distinct++;
	}
	printf("  %d distinct values over the treadle's travel, top = %d\n", distinct, last);
	CHECK(paired, "each update is MSB on CC11 then LSB on CC43, in one packet");
	CHECK(monotonic, "values rise monotonically");
	CHECK(distinct > 200, "more than 200 steps (7-bit gives 128)");
	CHECK(last == 16383, "full toe reaches 16383");

	printf("test_hires_dither_and_heel\n");
	int before = g_cap_n;
	for (int k = 0; k < 20; k++)
		feed(&m, 0, 0, 270 - (k & 1));    /* +-1 count dither at the toe */
	CHECK(g_cap_n == before, "one count of ADC dither sends nothing");
	for (int raw = 270; raw >= 40; raw--)
		feed(&m, 0, 0, raw);
	int v = (g_cap[g_cap_n - 4] << 7) | g_cap[g_cap_n - 1];
	CHECK(v == 0, "full heel returns to 0");

	printf("test_hires_switch_still_works\n");
	memset(&m, 0, sizeof(m));
	cfg_defaults(&m.cfg);
	m.calib_axis = -1;
	m.cfg.axis[2].hires = 1;
	m.cfg.axis[2].sw_cc = 9;
	feed(&m, 0, 0, 40);
	g_cap_n = 0;
	feed(&m, 0, 0, 150);
	int saw_sw = 0;
	for (int k = 0; k + 2 < g_cap_n; k += 3)
		if (g_cap[k] == 0xB0 && g_cap[k + 1] == 9 && g_cap[k + 2] == 127)
			saw_sw = 1;
	CHECK(saw_sw, "derived switch CC fires in hires mode");

	if (failures == 0)
		printf("\nALL MIDI TESTS PASSED\n");
	else
		printf("\n%d FAILURE(S)\n", failures);
	return failures ? 1 : 0;
}
