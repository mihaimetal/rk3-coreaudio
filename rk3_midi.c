/* rk3_midi.c — see rk3_midi.h */

#include "rk3_midi.h"
#include "caiaq_protocol.h"   /* EP1_CMD_READ_IO / EP1_CMD_READ_ANALOG */

#include <CoreMIDI/CoreMIDI.h>
#include <CoreFoundation/CoreFoundation.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>          /* strcasecmp */
#include <unistd.h>
#include <time.h>             /* clock_gettime (hold timers) */
#include <limits.h>           /* INT_MAX/INT_MIN (calibration) */
#include <sys/stat.h>         /* mkdir (config dir) */
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

/* ---- configurable mapping ------------------------------------------- */
/*
 * The defaults below reproduce the old hardcoded mapping. They can be
 * overridden at startup by ~/.config/rk3/midi.conf (rk3_midi_load_config);
 * a missing file just keeps these. See cfg_defaults() / rk3_midi_load_config().
 */

#define RK3_N_BUTTONS  9             /* keycode_rk3 has 9 (input.c:33) */
#define RK3_N_ANALOG   3             /* ABS_X/Y/Z (input.c:226) */
#define RK3_CC_OFF     (-1)          /* sentinel: control disabled (CC 0 is valid) */
#define RK3_HOLD_MS    350           /* default long-press threshold (ms) */

/* Button behaviour (config keyword in parentheses):
 *   MOMENTARY (momentary)  press -> cc 127, release -> cc 0       [the default]
 *   TOGGLE    (toggle)     press flips an internal latch 127<->0; release ignored
 *   HOLD      (hold)       momentary on `cc`; if held past hold_ms ALSO fire `cc2`
 *                          127, and release sends cc 0 (+ cc2 0 if it fired) —
 *                          i.e. an extra CC on long press.
 *   TAP       (tap)        tap-vs-hold, mutually exclusive, RELEASE-triggered tap:
 *                          released before hold_ms -> pulse `cc`; held past hold_ms
 *                          -> pulse `cc2` (the later release then does nothing).
 *                          A "pulse" is 127 immediately followed by 0. */
enum rk3_btn_mode { RK3_MOMENTARY = 0, RK3_TOGGLE, RK3_HOLD, RK3_TAP };

struct rk3_btn {
	int mode;        /* enum rk3_btn_mode */
	int cc;          /* primary CC; RK3_CC_OFF = button disabled */
	int cc2;         /* secondary (HOLD extra / TAP long); RK3_CC_OFF = none */
	int hold_ms;     /* HOLD/TAP threshold in ms */
};

struct rk3_axis {
	int cc;          /* continuous CC; RK3_CC_OFF = no continuous output */
	int min, max;    /* raw ADC values mapped to CC 0 / 127 */
	int sw_cc;       /* derived on/off switch CC; RK3_CC_OFF = none */
	int sw_on;       /* switch -> ON  when value >= sw_on  (0..127) */
	int sw_off;      /* switch -> OFF when value <  sw_off (hysteresis gap) */
	int hires;       /* 1 = 14-bit CC: `cc` is the MSB controller (0..31) and
	                  * cc+32 carries the LSB, per the MIDI 1.0 pairing */
	int smooth;      /* hires only: backlash in raw ADC counts that absorbs
	                  * the converter's +-1 dither at rest (0 = none) */
};

#define RK3_HIRES_SMOOTH 1   /* default backlash for a hires pedal */

struct rk3_cfg {
	int channel;                          /* 0..15 (MIDI channel 1..16 minus 1) */
	struct rk3_btn  btn[RK3_N_BUTTONS];   /* indexed by firmware bit */
	struct rk3_axis axis[RK3_N_ANALOG];   /* indexed by axis */
};

/* Physical silkscreen label P1..P9 -> firmware IO bit (0-based). The RK3 reports
 * its switches scrambled vs the labels (hardware-confirmed): physical P1..P9 fire
 * bits 4,5,6,7,0,1,2,3,8. Config addresses buttons by their printed label; this
 * maps label N (1..9) to the bit index the decoder works in (and the inverse). */
static const int kLabelToBit[RK3_N_BUTTONS] = { 4, 5, 6, 7, 0, 1, 2, 3, 8 };
static const int kBitToLabel[RK3_N_BUTTONS] = { 5, 6, 7, 8, 1, 2, 3, 4, 9 };

/* Axis index <-> config pedal name. axis0/1 = rear EXP jacks, axis2 = treadle. */
static const char *kAxisName[RK3_N_ANALOG] = { "exp1", "exp2", "treadle" };

/* Built-in defaults = the previously hardcoded mapping (all buttons momentary).
 *   buttons: bit b -> CC(19 + kBitToLabel[b]), i.e. physical Pn -> CC(19+n),
 *            press 127 / release 0, channel 1.
 *   analog:  axis2 = built-in treadle -> CC11, raw 40..270 (measured; does NOT
 *            swing the full 0..1023); axis0/1 = rear EXP jacks -> CC12/CC13,
 *            full 0..1023 (calibrate per pedal). No switch by default. */
static void cfg_defaults(struct rk3_cfg *c)
{
	static const int dflt_bit_cc[RK3_N_BUTTONS] =
		{ 24, 25, 26, 27, 20, 21, 22, 23, 28 };
	static const int acc[RK3_N_ANALOG]  = { 12, 13, 11 };
	static const int amin[RK3_N_ANALOG] = {  0,  0, 40 };
	static const int amax[RK3_N_ANALOG] = { 1023, 1023, 270 };

	memset(c, 0, sizeof(*c));
	c->channel = 0;
	for (int i = 0; i < RK3_N_BUTTONS; i++) {
		c->btn[i].mode    = RK3_MOMENTARY;
		c->btn[i].cc      = dflt_bit_cc[i];
		c->btn[i].cc2     = RK3_CC_OFF;
		c->btn[i].hold_ms = RK3_HOLD_MS;
	}
	for (int a = 0; a < RK3_N_ANALOG; a++) {
		c->axis[a].cc     = acc[a];
		c->axis[a].min    = amin[a];
		c->axis[a].max    = amax[a];
		c->axis[a].sw_cc  = RK3_CC_OFF;
		c->axis[a].sw_on  = 5;
		c->axis[a].sw_off = 3;
		c->axis[a].hires  = 0;
		c->axis[a].smooth = RK3_HIRES_SMOOTH;
	}
}

/* ---- state ---------------------------------------------------------- */

struct rk3_midi {
	MIDIClientRef   client;
	MIDIEndpointRef source;

	struct rk3_cfg cfg;                    /* mapping (defaults + config file) */

	int      have_buttons;
	uint8_t  button_state[RK3_N_BUTTONS];  /* last raw bit 0/1 */
	uint8_t  toggle_on[RK3_N_BUTTONS];     /* TOGGLE latch state */
	uint8_t  hold_pending[RK3_N_BUTTONS];  /* HOLD/TAP: down, timer running */
	uint8_t  hold_fired[RK3_N_BUTTONS];    /* HOLD/TAP: long action already sent */
	uint64_t press_ms[RK3_N_BUTTONS];      /* HOLD/TAP: press timestamp (ms) */

	int     have_analog;
	uint8_t analog_cc_val[RK3_N_ANALOG];   /* last sent 0..127 */
	int     analog_val14[RK3_N_ANALOG];    /* hires: last sent 0..16383 */
	int     analog_filt[RK3_N_ANALOG];     /* hires: backlash-filtered raw */
	int     analog_raw[RK3_N_ANALOG];      /* last raw, for -v on-change print */
	uint8_t analog_sw[RK3_N_ANALOG];       /* derived switch state 0/1 */

	/* live calibration capture (rk3_midi_calib_begin/finish) */
	int     calib_axis;                    /* axis under capture, -1 = none */
	int     calib_lo, calib_hi, calib_n;

	/* MIDI Beat Clock -> display decimal-point flash (rk3_midi_enable_clock_sync) */
	MIDIEndpointRef dest;                  /* virtual destination the DAW clocks  */
	int      led_fd;                       /* connected UDP -> engine LED port; -1 */
	unsigned clk;                          /* 0xF8 pulses since last downbeat reset */
};

/* ---- CoreMIDI send --------------------------------------------------- */

static void send_cc(struct rk3_midi *m, uint8_t cc, uint8_t val)
{
	Byte msg[3] = { (Byte)(0xB0 | (m->cfg.channel & 0x0f)),
			(Byte)(cc & 0x7f), (Byte)(val & 0x7f) };

	Byte storage[64];
	MIDIPacketList *list = (MIDIPacketList *)storage;
	MIDIPacket *pkt = MIDIPacketListInit(list);
	/* timestamp 0 => deliver now */
	pkt = MIDIPacketListAdd(list, sizeof(storage), pkt, 0, sizeof(msg), msg);
	if (pkt)
		MIDIReceived(m->source, list);
}

/* 14-bit controller: MSB on `cc`, LSB on cc+32, sent together (MSB first) so a
 * receiver never pairs a new MSB with a stale LSB. val is 0..16383. */
static void send_cc14(struct rk3_midi *m, uint8_t cc, int val)
{
	Byte st = (Byte)(0xB0 | (m->cfg.channel & 0x0f));
	Byte msg[6] = { st, (Byte)(cc & 0x7f), (Byte)((val >> 7) & 0x7f),
			st, (Byte)((cc + 32) & 0x7f), (Byte)(val & 0x7f) };

	Byte storage[64];
	MIDIPacketList *list = (MIDIPacketList *)storage;
	MIDIPacket *pkt = MIDIPacketListInit(list);
	pkt = MIDIPacketListAdd(list, sizeof(storage), pkt, 0, sizeof(msg), msg);
	if (pkt)
		MIDIReceived(m->source, list);
}

/* 127 immediately followed by 0 — a momentary "click" for TAP/HOLD triggers. */
static void send_pulse(struct rk3_midi *m, uint8_t cc)
{
	send_cc(m, cc, 127);
	send_cc(m, cc, 0);
}

static uint64_t now_ms(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000 + (uint64_t)ts.tv_nsec / 1000000;
}

/* ---- decoders (ported from Linux input.c) --------------------------- */

/* One button edge through its configured mode. pressed=1 press, 0 release. */
static void btn_edge(struct rk3_midi *m, int i, int pressed, int verbose)
{
	struct rk3_btn *b = &m->cfg.btn[i];
	int lbl = kBitToLabel[i];

	if (b->cc == RK3_CC_OFF) {                       /* button disabled */
		if (verbose)
			printf("  button P%d %-3s -> (disabled)\n",
			       lbl, pressed ? "ON" : "off");
		return;
	}

	switch (b->mode) {
	case RK3_TOGGLE:
		if (pressed) {
			m->toggle_on[i] ^= 1;
			send_cc(m, (uint8_t)b->cc, m->toggle_on[i] ? 127 : 0);
			if (verbose)
				printf("  button P%d toggle -> CC%d %d\n",
				       lbl, b->cc, m->toggle_on[i] ? 127 : 0);
		}
		break;

	case RK3_HOLD:                                   /* momentary + extra-on-hold */
		if (pressed) {
			send_cc(m, (uint8_t)b->cc, 127);
			m->press_ms[i]     = now_ms();
			m->hold_pending[i] = 1;
			m->hold_fired[i]   = 0;
			if (verbose)
				printf("  button P%d hold -> CC%d 127\n", lbl, b->cc);
		} else {
			send_cc(m, (uint8_t)b->cc, 0);
			if (m->hold_fired[i] && b->cc2 != RK3_CC_OFF)
				send_cc(m, (uint8_t)b->cc2, 0);
			m->hold_pending[i] = 0;
			m->hold_fired[i]   = 0;
			if (verbose)
				printf("  button P%d hold -> CC%d 0\n", lbl, b->cc);
		}
		break;

	case RK3_TAP:                                    /* tap vs hold, release-fired tap */
		if (pressed) {
			m->press_ms[i]     = now_ms();
			m->hold_pending[i] = 1;
			m->hold_fired[i]   = 0;
		} else {
			if (m->hold_pending[i] && !m->hold_fired[i]) {
				send_pulse(m, (uint8_t)b->cc);   /* it was a tap */
				if (verbose)
					printf("  button P%d tap -> CC%d\n", lbl, b->cc);
			}
			m->hold_pending[i] = 0;
			m->hold_fired[i]   = 0;
		}
		break;

	case RK3_MOMENTARY:
	default:
		send_cc(m, (uint8_t)b->cc, pressed ? 127 : 0);
		if (verbose)
			printf("  button P%d %-3s -> CC%d %d\n", lbl,
			       pressed ? "ON" : "off", b->cc, pressed ? 127 : 0);
		break;
	}
}

/* payload = EP1 buffer without the leading command byte. */
static void handle_io(struct rk3_midi *m, const uint8_t *p, int len,
		      int verbose)
{
	for (int i = 0; i < RK3_N_BUTTONS; i++) {
		int byte = i / 8;
		if (byte >= len)
			break;
		uint8_t bit = (p[byte] >> (i % 8)) & 1;   /* input.c:317 */

		if (!m->have_buttons) {           /* first report: seed, fire nothing */
			m->button_state[i] = bit;
			continue;
		}
		if (bit == m->button_state[i])
			continue;                 /* no edge */
		m->button_state[i] = bit;
		btn_edge(m, i, bit, verbose);
	}
	m->have_buttons = 1;
}

/* Fire HOLD/TAP long actions once a held button crosses its threshold. The
 * caller must invoke this regularly (the device only reports on change, so a
 * held button generates no packets to drive the timer otherwise). */
void rk3_midi_tick(struct rk3_midi *m, int verbose)
{
	if (!m || !m->have_buttons)
		return;
	uint64_t t = now_ms();
	for (int i = 0; i < RK3_N_BUTTONS; i++) {
		struct rk3_btn *b = &m->cfg.btn[i];
		if (!m->hold_pending[i] || m->hold_fired[i])
			continue;
		if (b->mode != RK3_HOLD && b->mode != RK3_TAP)
			continue;
		if (t - m->press_ms[i] < (uint64_t)b->hold_ms)
			continue;

		m->hold_fired[i] = 1;             /* threshold crossed, still held */
		if (b->cc2 == RK3_CC_OFF)
			continue;
		if (b->mode == RK3_HOLD) {
			send_cc(m, (uint8_t)b->cc2, 127);
			if (verbose)
				printf("  button P%d hold-extra -> CC%d 127\n",
				       kBitToLabel[i], b->cc2);
		} else {                         /* RK3_TAP */
			send_pulse(m, (uint8_t)b->cc2);
			if (verbose)
				printf("  button P%d hold -> CC%d\n",
				       kBitToLabel[i], b->cc2);
		}
	}
}

static void handle_analog(struct rk3_midi *m, const uint8_t *p, int len,
			  int verbose)
{
	for (int a = 0; a < RK3_N_ANALOG; a++) {
		int off = a * 2;                          /* input.c:208 */
		if (off + 1 >= len)
			break;
		int raw = (p[off] << 8) | p[off + 1];     /* big-endian */
		struct rk3_axis *ax = &m->cfg.axis[a];

		if (m->calib_axis == a) {                 /* live calibration capture */
			if (raw < m->calib_lo) m->calib_lo = raw;
			if (raw > m->calib_hi) m->calib_hi = raw;
			m->calib_n++;
		}

		int mapped = (ax->cc != RK3_CC_OFF || ax->sw_cc != RK3_CC_OFF);

		/* Print every axis on raw change (even unmapped ones) so the pedal
		 * jacks can be probed/calibrated with -v. Threshold drops ADC jitter. */
		if (verbose && (!m->have_analog ||
				abs(raw - m->analog_raw[a]) > 3)) {
			printf("  analog%d raw=%4d%s\n", a, raw,
			       mapped ? "" : "  (axis disabled, no CC)");
			m->analog_raw[a] = raw;
		}

		if (!mapped)
			continue;

		int lo = ax->min, hi = ax->max, clamped = raw;
		uint8_t val;

		if (ax->hires && ax->cc != RK3_CC_OFF) {
			/*
			 * 14-bit path. Every raw ADC count becomes its own value, so
			 * the converter's dither at rest would chatter; a backlash of
			 * `smooth` counts absorbs it without costing resolution while
			 * the pedal moves (the output trails the input by `smooth`
			 * counts in the direction of travel). The calibrated range is
			 * narrowed by the same amount at each end so 0 and 16383 stay
			 * reachable.
			 */
			int h = ax->smooth, f = m->have_analog ? m->analog_filt[a] : raw;
			if (raw > f + h) f = raw - h;
			else if (raw < f - h) f = raw + h;
			m->analog_filt[a] = f;

			int flo = lo + h, fhi = hi - h;
			if (fhi <= flo) { flo = lo; fhi = hi; }
			clamped = f;
			if (clamped < flo) clamped = flo;
			if (clamped > fhi) clamped = fhi;
			int v14 = (fhi > flo)
				? (int)((long)(clamped - flo) * 16383 / (fhi - flo)) : 0;
			val = (uint8_t)(v14 >> 7);       /* 7-bit view for the switch */
			if (!m->have_analog || v14 != m->analog_val14[a]) {
				m->analog_val14[a] = v14;
				m->analog_cc_val[a] = val;
				send_cc14(m, (uint8_t)ax->cc, v14);
				if (verbose)
					printf("    analog%d -> CC%d/%d %d (14-bit)\n",
					       a, ax->cc, ax->cc + 32, v14);
			}
		} else {
			if (clamped < lo) clamped = lo;
			if (clamped > hi) clamped = hi;
			val = (hi > lo)
				? (uint8_t)((clamped - lo) * 127 / (hi - lo)) : 0;
		}

		/* continuous CC (7-bit) */
		if (!ax->hires && ax->cc != RK3_CC_OFF &&
		    (!m->have_analog || val != m->analog_cc_val[a])) {
			m->analog_cc_val[a] = val;
			send_cc(m, (uint8_t)ax->cc, val);
			if (verbose)
				printf("    analog%d -> CC%d %d\n", a, ax->cc, val);
		}

		/* derived on/off switch (hysteresis: ON at >=sw_on, OFF at <sw_off) */
		if (ax->sw_cc != RK3_CC_OFF) {
			uint8_t sw = m->analog_sw[a];
			if (!sw && val >= ax->sw_on)       sw = 1;
			else if (sw && val <  ax->sw_off)  sw = 0;
			if (!m->have_analog || sw != m->analog_sw[a]) {
				m->analog_sw[a] = sw;
				send_cc(m, (uint8_t)ax->sw_cc, sw ? 127 : 0);
				if (verbose)
					printf("    analog%d switch -> CC%d %d\n",
					       a, ax->sw_cc, sw ? 127 : 0);
			}
		}
	}
	m->have_analog = 1;
}

/* ---- public --------------------------------------------------------- */

void rk3_midi_dispatch(struct rk3_midi *m, const uint8_t *buf, int len,
		       int verbose)
{
	if (!m || len < 1)
		return;

	switch (buf[0]) {
	case EP1_CMD_READ_IO:
		handle_io(m, buf + 1, len - 1, verbose);
		break;
	case EP1_CMD_READ_ANALOG:
		handle_analog(m, buf + 1, len - 1, verbose);
		break;
	default:
		break;   /* device-info / midi / audio replies: not ours */
	}
}

/* ---- config file ---------------------------------------------------- */

/* Resolve the config path: $RK3_MIDI_CONFIG, else ~/.config/rk3/midi.conf.
 * Returns 0 and fills buf, or -1 if neither is available. */
static int cfg_path(char *buf, size_t n)
{
	const char *env = getenv("RK3_MIDI_CONFIG");
	if (env && *env) {
		snprintf(buf, n, "%s", env);
		return 0;
	}
	const char *home = getenv("HOME");
	if (!home || !*home)
		return -1;
	snprintf(buf, n, "%s/.config/rk3/midi.conf", home);
	return 0;
}

/* Parse a base-10 int from `s` (whole token, no trailing junk). 0 ok, -1 bad. */
static int parse_int(const char *s, int *out)
{
	char *end;
	long v = strtol(s, &end, 10);
	if (end == s || *end != '\0')
		return -1;
	*out = (int)v;
	return 0;
}

/* Consume "<kw> <int>" at tok[*i]; advances *i past the value. Returns the int
 * via *out (validated to [lo,hi]) or -1 on a missing/out-of-range value. */
static int take_val(char **tok, int n, int *i, int lo, int hi, int *out)
{
	if (*i + 1 >= n || parse_int(tok[++(*i)], out) || *out < lo || *out > hi)
		return -1;
	return 0;
}

/* axis name (treadle/exp1/exp2) -> axis index, or -1. */
static int axis_from_name(const char *name)
{
	for (int a = 0; a < RK3_N_ANALOG; a++)
		if (strcasecmp(name, kAxisName[a]) == 0)
			return a;
	return -1;
}

int rk3_midi_load_config(struct rk3_midi *m, const char *path)
{
	if (!m)
		return -1;

	/* Reset to defaults first so a reload drops settings removed from the
	 * file (and a re-specified line fully redefines that control). */
	cfg_defaults(&m->cfg);

	char pathbuf[1024];
	if (!path) {
		if (cfg_path(pathbuf, sizeof(pathbuf)))
			return 0;                  /* no $HOME/override: keep defaults */
		path = pathbuf;
	}

	FILE *f = fopen(path, "r");
	if (!f)
		return 0;                          /* missing file: keep defaults */

	fprintf(stderr, "rk3_midi: loading config %s\n", path);

	char line[512];
	int lineno = 0, errors = 0;
	while (fgets(line, sizeof(line), f)) {
		lineno++;

		char *hash = strchr(line, '#');    /* strip comment */
		if (hash)
			*hash = '\0';

		char *tok[32];                     /* tokenize on whitespace and '=' */
		int n = 0;
		for (char *p = strtok(line, " \t\r\n=");
		     p && n < 32; p = strtok(NULL, " \t\r\n="))
			tok[n++] = p;
		if (n == 0)
			continue;

		if (strcasecmp(tok[0], "channel") == 0) {
			int ch, i = 0;
			if (take_val(tok, n, &i, 1, 16, &ch)) {
				fprintf(stderr, "  line %d: bad channel "
					"(want 1..16)\n", lineno);
				errors++;
				continue;
			}
			m->cfg.channel = ch - 1;

		} else if (strcasecmp(tok[0], "button") == 0) {
			int label;
			if (n < 2 || parse_int(tok[1], &label) ||
			    label < 1 || label > RK3_N_BUTTONS) {
				fprintf(stderr, "  line %d: bad button label "
					"(want 1..9)\n", lineno);
				errors++;
				continue;
			}
			struct rk3_btn *b = &m->cfg.btn[kLabelToBit[label - 1]];
			int ok = 1, v;
			for (int i = 2; i < n; i++) {
				if (strcasecmp(tok[i], "momentary") == 0)
					b->mode = RK3_MOMENTARY;
				else if (strcasecmp(tok[i], "toggle") == 0)
					b->mode = RK3_TOGGLE;
				else if (strcasecmp(tok[i], "hold") == 0)
					b->mode = RK3_HOLD;
				else if (strcasecmp(tok[i], "tap") == 0)
					b->mode = RK3_TAP;
				else if (strcasecmp(tok[i], "off") == 0)
					b->cc = RK3_CC_OFF;
				else if (strcasecmp(tok[i], "cc") == 0 &&
					 !take_val(tok, n, &i, 0, 127, &v))
					b->cc = v;
				else if (strcasecmp(tok[i], "long") == 0 &&
					 !take_val(tok, n, &i, 0, 127, &v))
					b->cc2 = v;
				else if (strcasecmp(tok[i], "time") == 0 &&
					 !take_val(tok, n, &i, 1, 10000, &v))
					b->hold_ms = v;
				else {
					fprintf(stderr, "  line %d: button %d: bad "
						"token near '%s'\n", lineno, label,
						tok[i]);
					ok = 0;
					break;
				}
			}
			if (!ok)
				errors++;

		} else if (strcasecmp(tok[0], "pedal") == 0) {
			int axis = (n >= 2) ? axis_from_name(tok[1]) : -1;
			if (axis < 0) {
				fprintf(stderr, "  line %d: bad pedal name "
					"(want treadle|exp1|exp2)\n", lineno);
				errors++;
				continue;
			}
			struct rk3_axis *ax = &m->cfg.axis[axis];
			int ok = 1, v;
			for (int i = 2; i < n; i++) {
				if (strcasecmp(tok[i], "off") == 0)
					ax->cc = RK3_CC_OFF;
				else if (strcasecmp(tok[i], "cc") == 0 &&
					 !take_val(tok, n, &i, 0, 127, &v))
					ax->cc = v;
				else if (strcasecmp(tok[i], "hires") == 0)
					ax->hires = 1;
				else if (strcasecmp(tok[i], "lores") == 0)
					ax->hires = 0;
				else if (strcasecmp(tok[i], "smooth") == 0 &&
					 !take_val(tok, n, &i, 0, 16, &v))
					ax->smooth = v;
				else if (strcasecmp(tok[i], "calibmin") == 0 &&
					 !take_val(tok, n, &i, 0, 65535, &v))
					ax->min = v;
				else if (strcasecmp(tok[i], "calibmax") == 0 &&
					 !take_val(tok, n, &i, 0, 65535, &v))
					ax->max = v;
				else if (strcasecmp(tok[i], "switch") == 0 &&
					 !take_val(tok, n, &i, 0, 127, &v))
					ax->sw_cc = v;
				else if (strcasecmp(tok[i], "son") == 0 &&
					 !take_val(tok, n, &i, 0, 127, &v))
					ax->sw_on = v;
				else if (strcasecmp(tok[i], "soff") == 0 &&
					 !take_val(tok, n, &i, 0, 127, &v))
					ax->sw_off = v;
				else {
					fprintf(stderr, "  line %d: pedal %s: bad "
						"token near '%s'\n", lineno,
						tok[1], tok[i]);
					ok = 0;
					break;
				}
			}
			if (!ok)
				errors++;

		} else {
			fprintf(stderr, "  line %d: unknown directive '%s'\n",
				lineno, tok[0]);
			errors++;
		}
	}
	fclose(f);

	/* Per-axis sanity: a calibration range with min >= max would clamp every
	 * value to 0; revert that axis. And keep switch hysteresis sane (off<=on). */
	struct rk3_cfg d;
	cfg_defaults(&d);
	for (int a = 0; a < RK3_N_ANALOG; a++) {
		struct rk3_axis *ax = &m->cfg.axis[a];
		if (ax->min >= ax->max) {
			fprintf(stderr, "  %s: calibMin >= calibMax (%d >= %d); "
				"reverting to default %d..%d\n", kAxisName[a],
				ax->min, ax->max, d.axis[a].min, d.axis[a].max);
			ax->min = d.axis[a].min;
			ax->max = d.axis[a].max;
			errors++;
		}
		if (ax->hires && ax->cc != RK3_CC_OFF && ax->cc > 31) {
			fprintf(stderr, "  %s: hires needs cc 0..31 (the LSB goes out "
				"on cc+32); cc %d stays 7-bit\n", kAxisName[a], ax->cc);
			ax->hires = 0;
			errors++;
		}
		if (ax->sw_off > ax->sw_on) {
			fprintf(stderr, "  %s: soff > son (%d > %d); clamping "
				"soff to son\n", kAxisName[a], ax->sw_off, ax->sw_on);
			ax->sw_off = ax->sw_on;
			errors++;
		}
	}

	return errors ? 1 : 0;
}

/* ---- live calibration ------------------------------------------------ */

/* Start capturing raw min/max for `axis`; handle_analog records while active. */
void rk3_midi_calib_begin(struct rk3_midi *m, int axis)
{
	if (!m || axis < 0 || axis >= RK3_N_ANALOG)
		return;
	m->calib_lo   = INT_MAX;
	m->calib_hi   = INT_MIN;
	m->calib_n    = 0;
	m->calib_axis = axis;            /* set last: handle_analog reads it live */
}

/* Build the canonical "pedal <name> = ..." line for `axis` from cfg. */
static void fmt_pedal_line(char *buf, size_t n, const struct rk3_axis *ax,
			   const char *name)
{
	int k = snprintf(buf, n, "pedal %s =", name);
	if (ax->cc == RK3_CC_OFF)
		k += snprintf(buf + k, n - k, " off");
	else
		k += snprintf(buf + k, n - k, " cc %d", ax->cc);
	k += snprintf(buf + k, n - k, " calibMin %d calibMax %d", ax->min, ax->max);
	if (ax->hires) {
		k += snprintf(buf + k, n - k, " hires");
		if (ax->smooth != RK3_HIRES_SMOOTH)
			k += snprintf(buf + k, n - k, " smooth %d", ax->smooth);
	}
	if (ax->sw_cc != RK3_CC_OFF)
		k += snprintf(buf + k, n - k, " switch %d son %d soff %d",
			      ax->sw_cc, ax->sw_on, ax->sw_off);
	snprintf(buf + k, n - k, "\n");
}

/* Rewrite (or create) the config file, replacing just `axis`'s pedal line with
 * the current cfg values. Other lines + comments are preserved verbatim. */
static void cfg_save_pedal(struct rk3_midi *m, int axis)
{
	const char *name = kAxisName[axis];
	char path[1024];
	if (cfg_path(path, sizeof(path))) {
		fprintf(stderr, "rk3_midi: no config path; not saving\n");
		return;
	}

	char newline[256];
	fmt_pedal_line(newline, sizeof(newline), &m->cfg.axis[axis], name);

	char *lines[512];
	int nlines = 0, replaced = 0, had_file = 0;
	FILE *f = fopen(path, "r");
	if (f) {
		had_file = 1;
		char buf[512];
		while (nlines < 512 && fgets(buf, sizeof(buf), f)) {
			char tmp[512];
			snprintf(tmp, sizeof(tmp), "%s", buf);
			char *hash = strchr(tmp, '#');
			if (hash)
				*hash = '\0';
			char *t0 = strtok(tmp, " \t\r\n=");
			char *t1 = t0 ? strtok(NULL, " \t\r\n=") : NULL;
			if (!replaced && t0 && t1 &&
			    strcasecmp(t0, "pedal") == 0 &&
			    strcasecmp(t1, name) == 0) {
				lines[nlines++] = strdup(newline);
				replaced = 1;
			} else {
				lines[nlines++] = strdup(buf);
			}
		}
		fclose(f);
	} else {
		/* first write: make sure ~/.config/rk3 exists (default path only) */
		const char *home = getenv("HOME");
		if (home && !getenv("RK3_MIDI_CONFIG")) {
			char d[1024];
			snprintf(d, sizeof(d), "%s/.config", home);
			mkdir(d, 0755);
			snprintf(d, sizeof(d), "%s/.config/rk3", home);
			mkdir(d, 0755);
		}
	}

	FILE *w = fopen(path, "w");
	if (!w) {
		fprintf(stderr, "rk3_midi: cannot write %s\n", path);
		for (int i = 0; i < nlines; i++)
			free(lines[i]);
		return;
	}
	if (!had_file)
		fputs("# RigKontrol 3 MIDI config "
		      "(auto-updated by rk3-calibrate)\n", w);
	for (int i = 0; i < nlines; i++) {
		fputs(lines[i], w);
		free(lines[i]);
	}
	if (!replaced)
		fputs(newline, w);
	fclose(w);
	fprintf(stderr, "rk3_midi: saved %s calibration to %s\n", name, path);
}

/* Stop capturing for `axis`, apply the captured range live AND persist it to
 * the config file. Returns 0 (filling out_lo and out_hi if non-NULL) or -1 if
 * the pedal wasn't swept enough (too few samples / no movement). */
int rk3_midi_calib_finish(struct rk3_midi *m, int axis, int *out_lo, int *out_hi)
{
	if (!m || axis < 0 || axis >= RK3_N_ANALOG)
		return -1;
	int n = m->calib_n, lo = m->calib_lo, hi = m->calib_hi;
	m->calib_axis = -1;                      /* stop capturing */
	if (n < 2 || lo >= hi) {
		fprintf(stderr, "rk3_midi: %s calibration: not enough movement "
			"(samples=%d)\n", kAxisName[axis], n);
		return -1;
	}
	m->cfg.axis[axis].min = lo;              /* apply live */
	m->cfg.axis[axis].max = hi;
	if (out_lo) *out_lo = lo;
	if (out_hi) *out_hi = hi;
	fprintf(stderr, "rk3_midi: %s calibrated: calibMin %d calibMax %d\n",
		kAxisName[axis], lo, hi);
	cfg_save_pedal(m, axis);                 /* persist */
	return 0;
}

/* ---- MIDI Beat Clock -> display decimal-point flash ------------------ */

/* "Con" glyphs (see rk3_engine.c): C=0x39, o=0x5C, n=0x54; bit7 = decimal pt.
 * Send a 3-byte control_state to the engine's LED port; the engine zero-pads to
 * a full frame and relays it to WRITE_IO. `on` lights the dot after the 'n'. */
static void led_flash_dot(struct rk3_midi *m, int on)
{
	if (m->led_fd < 0)
		return;
	uint8_t f[3] = { 0x39, 0x5C, (uint8_t)(on ? (0x54 | 0x80) : 0x54) };
	(void)send(m->led_fd, f, sizeof(f), 0);   /* best-effort; engine may drop */
}

/* CoreMIDI delivers clock on its own thread. System Real-Time bytes (>=0xF8)
 * are single-byte and never collide with data bytes (<0x80), so a flat scan of
 * every byte is safe. 24 clocks = one beat: dot ON at the beat, OFF ~2 clocks
 * later (a brief flash whose length tracks tempo but stays short). */
static void clock_read(const MIDIPacketList *pl, void *refCon, void *srcRef)
{
	(void)srcRef;
	struct rk3_midi *m = refCon;
	const MIDIPacket *p = &pl->packet[0];
	for (unsigned i = 0; i < pl->numPackets; i++) {
		for (UInt16 j = 0; j < p->length; j++) {
			switch (p->data[j]) {
			case 0xF8: {                       /* timing clock */
				unsigned phase = m->clk % 24;
				if (phase == 0)
					led_flash_dot(m, 1);
				else if (phase == 2)
					led_flash_dot(m, 0);
				m->clk++;
				break;
			}
			case 0xFA:                         /* start    */
			case 0xFB:                         /* continue */
				m->clk = 0;                /* align the downbeat */
				break;
			case 0xFC:                         /* stop     */
				m->clk = 0;
				led_flash_dot(m, 0);       /* leave a clean "Con" */
				break;
			default:
				break;
			}
		}
		p = MIDIPacketNext(p);
	}
}

int rk3_midi_enable_clock_sync(struct rk3_midi *m, const char *dest_name)
{
	if (!m)
		return -1;

	/* Connected UDP socket so led_flash_dot() can just send(). */
	m->led_fd = socket(AF_INET, SOCK_DGRAM, 0);
	if (m->led_fd >= 0) {
		struct sockaddr_in a;
		memset(&a, 0, sizeof(a));
		a.sin_family = AF_INET;
		a.sin_port = htons(RK3_LED_UDP_PORT);
		a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
		if (connect(m->led_fd, (struct sockaddr *)&a, sizeof(a)) != 0) {
			close(m->led_fd);
			m->led_fd = -1;
		}
	}

	CFStringRef cn = CFStringCreateWithCString(NULL, dest_name,
						   kCFStringEncodingUTF8);
	OSStatus st = MIDIDestinationCreate(m->client, cn, clock_read, m, &m->dest);
	if (cn)
		CFRelease(cn);
	if (st != noErr) {
		fprintf(stderr, "rk3_midi_enable_clock_sync: "
			"MIDIDestinationCreate failed (%d)\n", (int)st);
		return -1;
	}
	return 0;
}

struct rk3_midi *rk3_midi_open(const char *name)
{
	struct rk3_midi *m = calloc(1, sizeof(*m));
	if (!m)
		return NULL;
	m->led_fd = -1;                  /* calloc leaves 0 (= stdin); mark unset */
	m->calib_axis = -1;              /* not calibrating */
	cfg_defaults(&m->cfg);           /* override later via rk3_midi_load_config */

	CFStringRef cfname = CFStringCreateWithCString(
		NULL, name, kCFStringEncodingUTF8);

	OSStatus st = MIDIClientCreate(cfname, NULL, NULL, &m->client);
	if (st != noErr) {
		fprintf(stderr, "rk3_midi_open: MIDIClientCreate failed (%d)\n",
			(int)st);
		goto fail;
	}

	st = MIDISourceCreate(m->client, cfname, &m->source);
	if (st != noErr) {
		fprintf(stderr, "rk3_midi_open: MIDISourceCreate failed (%d)\n",
			(int)st);
		goto fail;
	}

	CFRelease(cfname);
	return m;

fail:
	if (cfname)
		CFRelease(cfname);
	rk3_midi_close(m);
	return NULL;
}

void rk3_midi_close(struct rk3_midi *m)
{
	if (!m)
		return;
	if (m->led_fd >= 0)
		close(m->led_fd);
	if (m->dest)
		MIDIEndpointDispose(m->dest);
	if (m->source)
		MIDIEndpointDispose(m->source);
	if (m->client)
		MIDIClientDispose(m->client);
	free(m);
}
