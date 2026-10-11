/*
 * midimon.c — watch the "RigKontrol 3" CoreMIDI source and summarise a pedal's
 * controller stream: how many messages, how many DISTINCT values, the range,
 * and whether it stays quiet when the pedal is held still. Understands a
 * 14-bit pair (MSB on cc, LSB on cc+32) as well as a plain 7-bit CC.
 *
 * Usage: ./build/midimon [seconds] [cc]      (default 30 s, CC 11)
 */
#include <CoreMIDI/CoreMIDI.h>
#include <CoreFoundation/CoreFoundation.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int g_cc = 11;
static int g_msb = -1, g_lsb = -1;      /* latest halves */
static unsigned g_n_msb, g_n_lsb, g_n_other;
static unsigned char g_seen14[16384], g_seen7[128];
static int g_min = 1 << 30, g_max = -1, g_cur = -1;
static unsigned g_sec_msgs;

static void on_midi(const MIDIPacketList *l, void *a, void *b)
{
	(void)a; (void)b;
	const MIDIPacket *p = &l->packet[0];
	for (UInt32 i = 0; i < l->numPackets; i++, p = MIDIPacketNext(p)) {
		for (int k = 0; k + 2 < p->length + 0 && k + 2 <= p->length - 1; k += 3) {
			if ((p->data[k] & 0xF0) != 0xB0) { g_n_other++; continue; }
			int cc = p->data[k + 1], v = p->data[k + 2];
			if (cc == g_cc) { g_msb = v; g_n_msb++; g_seen7[v] = 1; }
			else if (cc == g_cc + 32) { g_lsb = v; g_n_lsb++; }
			else { g_n_other++; continue; }
			g_sec_msgs++;
			if (cc == g_cc + 32 && g_msb >= 0) {       /* pair complete */
				g_cur = (g_msb << 7) | g_lsb;
				g_seen14[g_cur] = 1;
				if (g_cur < g_min) g_min = g_cur;
				if (g_cur > g_max) g_max = g_cur;
			}
		}
	}
}

static void tick(CFRunLoopTimerRef t, void *info)
{
	(void)t;
	int *left = info;
	printf("  %2ds left: %4u msgs/s  value=%d\n", *left, g_sec_msgs,
	       g_cur >= 0 ? g_cur : g_msb);
	fflush(stdout);
	g_sec_msgs = 0;
	if (--*left <= 0)
		CFRunLoopStop(CFRunLoopGetCurrent());
}

int main(int argc, char **argv)
{
	int seconds = argc > 1 ? atoi(argv[1]) : 30;
	if (argc > 2) g_cc = atoi(argv[2]);

	MIDIClientRef client = 0;
	MIDIPortRef port = 0;
	MIDIClientCreate(CFSTR("rk3 midimon"), NULL, NULL, &client);
	MIDIInputPortCreate(client, CFSTR("in"), on_midi, NULL, &port);
	int found = 0;
	for (ItemCount i = 0; i < MIDIGetNumberOfSources(); i++) {
		MIDIEndpointRef src = MIDIGetSource(i);
		CFStringRef name = NULL;
		MIDIObjectGetStringProperty(src, kMIDIPropertyName, &name);
		if (name && CFStringCompare(name, CFSTR("RigKontrol 3"), 0) == kCFCompareEqualTo) {
			MIDIPortConnectSource(port, src, NULL);
			found = 1;
		}
		if (name) CFRelease(name);
	}
	if (!found) { fprintf(stderr, "no 'RigKontrol 3' MIDI source (helper running?)\n"); return 1; }

	printf("listening on CC%d (+CC%d) for %d s — move the pedal\n", g_cc, g_cc + 32, seconds);
	CFRunLoopTimerContext ctx = { 0, &seconds, NULL, NULL, NULL };
	CFRunLoopTimerRef tm = CFRunLoopTimerCreate(NULL, CFAbsoluteTimeGetCurrent() + 1,
			1, 0, 0, tick, &ctx);
	CFRunLoopAddTimer(CFRunLoopGetCurrent(), tm, kCFRunLoopDefaultMode);
	CFRunLoopRun();

	int d14 = 0, d7 = 0;
	for (int i = 0; i < 16384; i++) d14 += g_seen14[i];
	for (int i = 0; i < 128; i++) d7 += g_seen7[i];
	printf("CC%d (MSB): %u messages, %d distinct 7-bit values\n", g_cc, g_n_msb, d7);
	printf("CC%d (LSB): %u messages\n", g_cc + 32, g_n_lsb);
	if (g_n_lsb)
		printf("14-bit: %d distinct values, range %d..%d\n", d14, g_min, g_max);
	else
		printf("no LSB seen: this pedal is sending plain 7-bit CC\n");
	printf("other messages ignored: %u\n", g_n_other);
	return 0;
}
