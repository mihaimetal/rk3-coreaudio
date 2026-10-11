/*
 * iotest.c — drive the RigKontrol 3 AudioServerPlugin device for end-to-end test.
 *
 * Binds an AUHAL output unit to OUR device (matched by UID), and (optionally) an
 * input AUHAL, to exercise the plugin's StartIO / DoIOOperation path and the USB
 * engine inside coreaudiod — without disturbing the system default device.
 *
 *   ./build/iotest [seconds] [--in]
 *      default: play a 440 Hz tone out the RK3 for `seconds` (listen on RK3 out).
 *      --in:    also capture from the RK3 and write iotest_cap.wav.
 *      --rate N:  switch the device to N Hz first (default: its current rate).
 *      --buf N:   IO buffer frames for --latency (default 64).
 *      --thresh X: --latency detection threshold, linear (default 0.05; lower
 *                 it if the loopback level is weak).
 *      --latency: LOOPBACK round-trip latency measurement. Patch a cable from an
 *                 RK3 OUTPUT jack to an RK3 INPUT jack first. Emits one impulse
 *                 per second on output, detects its arrival on input, and reports
 *                 the round-trip in samples / ms using the shared device clock
 *                 (mSampleTime). This is the GROUND TRUTH; the reported
 *                 kAudioDevicePropertyLatency is only an estimate.
 *
 * Watch the engine with:  make driver-log
 */

#include <AudioToolbox/AudioToolbox.h>
#include <CoreAudio/CoreAudio.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define RK3_UID "RigKontrol3:17cc:1940"
/* Stream at the device's current nominal rate (read in main) so the AUHAL never
 * resamples; --rate N switches the device first. */
static double g_rate = 48000.0;
#define RATE g_rate

static AudioDeviceID find_device(void)
{
	AudioObjectPropertyAddress da = { kAudioHardwarePropertyDevices,
		kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain };
	UInt32 sz = 0;
	if (AudioObjectGetPropertyDataSize(kAudioObjectSystemObject, &da,
					   0, NULL, &sz) != noErr || sz == 0)
		return kAudioObjectUnknown;
	int n = sz / sizeof(AudioDeviceID);
	AudioDeviceID *ids = malloc(sz);
	if (AudioObjectGetPropertyData(kAudioObjectSystemObject, &da,
				       0, NULL, &sz, ids) != noErr) {
		free(ids); return kAudioObjectUnknown;
	}
	AudioDeviceID found = kAudioObjectUnknown;
	for (int i = 0; i < n && found == kAudioObjectUnknown; i++) {
		AudioObjectPropertyAddress ua = { kAudioDevicePropertyDeviceUID,
			kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain };
		CFStringRef uid = NULL; UInt32 usz = sizeof(uid);
		if (AudioObjectGetPropertyData(ids[i], &ua, 0, NULL, &usz, &uid) == noErr
		    && uid) {
			if (CFStringCompare(uid, CFSTR(RK3_UID), 0) == kCFCompareEqualTo)
				found = ids[i];
			CFRelease(uid);
		}
	}
	free(ids);
	return found;
}

static double g_phase = 0.0;
static OSStatus render_cb(void *ref, AudioUnitRenderActionFlags *flags,
			  const AudioTimeStamp *ts, UInt32 bus, UInt32 nframes,
			  AudioBufferList *io)
{
	(void)ref; (void)flags; (void)ts; (void)bus;
	float *out = (float *)io->mBuffers[0].mData;   /* interleaved stereo */
	double step = 2.0 * M_PI * 440.0 / RATE;
	for (UInt32 i = 0; i < nframes; i++) {
		float v = (float)(0.2 * sin(g_phase));
		g_phase += step;
		if (g_phase > 2.0 * M_PI) g_phase -= 2.0 * M_PI;
		out[i * 2 + 0] = v;
		out[i * 2 + 1] = v;
	}
	return noErr;
}

/* ---- WAV writer (Float32 in -> S24 LE) ------------------------------ */

static void put_le32(FILE *f, uint32_t v) {
	fputc(v, f); fputc(v >> 8, f); fputc(v >> 16, f); fputc(v >> 24, f);
}
static void put_le16(FILE *f, uint16_t v) { fputc(v, f); fputc(v >> 8, f); }

/* ---- input capture mode --------------------------------------------- */

static AudioUnit g_in_au;
static FILE *g_wav;
static AudioBufferList *g_iolist;
static UInt32 g_frames_written;

static OSStatus input_cb(void *ref, AudioUnitRenderActionFlags *flags,
			 const AudioTimeStamp *ts, UInt32 bus, UInt32 nframes,
			 AudioBufferList *unused)
{
	(void)ref; (void)unused;
	g_iolist->mBuffers[0].mDataByteSize = nframes * 8;
	OSStatus s = AudioUnitRender(g_in_au, flags, ts, bus, nframes, g_iolist);
	if (s != noErr) return s;
	const float *in = (const float *)g_iolist->mBuffers[0].mData;
	for (UInt32 i = 0; i < nframes * 2; i++) {
		float f = in[i];
		if (f > 1.0f) f = 1.0f; if (f < -1.0f) f = -1.0f;
		int32_t v = (int32_t)(f * 8388607.0f);
		fputc(v & 0xff, g_wav); fputc((v >> 8) & 0xff, g_wav);
		fputc((v >> 16) & 0xff, g_wav);
	}
	g_frames_written += nframes;
	return noErr;
}

static int run_input(AudioDeviceID dev, double seconds)
{
	AudioComponentDescription desc = {
		kAudioUnitType_Output, kAudioUnitSubType_HALOutput,
		kAudioUnitManufacturer_Apple, 0, 0 };
	AudioComponent comp = AudioComponentFindNext(NULL, &desc);
	if (AudioComponentInstanceNew(comp, &g_in_au) != noErr) return 1;

	UInt32 one = 1, zero = 0; OSStatus s;
	s = AudioUnitSetProperty(g_in_au, kAudioOutputUnitProperty_EnableIO,
			kAudioUnitScope_Input, 1, &one, sizeof(one));
	if (s) { fprintf(stderr, "EnableIO(in): %d\n", s); return 1; }
	s = AudioUnitSetProperty(g_in_au, kAudioOutputUnitProperty_EnableIO,
			kAudioUnitScope_Output, 0, &zero, sizeof(zero));
	if (s) { fprintf(stderr, "DisableIO(out): %d\n", s); return 1; }
	s = AudioUnitSetProperty(g_in_au, kAudioOutputUnitProperty_CurrentDevice,
			kAudioUnitScope_Global, 0, &dev, sizeof(dev));
	if (s) { fprintf(stderr, "set CurrentDevice: %d\n", s); return 1; }

	AudioStreamBasicDescription fmt = {
		.mSampleRate = RATE, .mFormatID = kAudioFormatLinearPCM,
		.mFormatFlags = kAudioFormatFlagIsFloat | kAudioFormatFlagIsPacked,
		.mBytesPerPacket = 8, .mFramesPerPacket = 1, .mBytesPerFrame = 8,
		.mChannelsPerFrame = 2, .mBitsPerChannel = 32 };
	s = AudioUnitSetProperty(g_in_au, kAudioUnitProperty_StreamFormat,
			kAudioUnitScope_Output, 1, &fmt, sizeof(fmt));
	if (s) { fprintf(stderr, "set in StreamFormat: %d\n", s); return 1; }

	AURenderCallbackStruct cb = { input_cb, NULL };
	s = AudioUnitSetProperty(g_in_au, kAudioOutputUnitProperty_SetInputCallback,
			kAudioUnitScope_Global, 0, &cb, sizeof(cb));
	if (s) { fprintf(stderr, "set InputCallback: %d\n", s); return 1; }

	g_iolist = calloc(1, sizeof(AudioBufferList) + sizeof(AudioBuffer));
	g_iolist->mNumberBuffers = 1;
	g_iolist->mBuffers[0].mNumberChannels = 2;
	g_iolist->mBuffers[0].mData = malloc(8 * 4096);

	/* WAV header (24-bit stereo); patch sizes at the end */
	g_wav = fopen("iotest_cap.wav", "wb");
	fwrite("RIFF", 1, 4, g_wav); put_le32(g_wav, 0);
	fwrite("WAVE", 1, 4, g_wav); fwrite("fmt ", 1, 4, g_wav);
	put_le32(g_wav, 16); put_le16(g_wav, 1); put_le16(g_wav, 2);
	put_le32(g_wav, (uint32_t)RATE); put_le32(g_wav, (uint32_t)RATE * 6);
	put_le16(g_wav, 6); put_le16(g_wav, 24);
	fwrite("data", 1, 4, g_wav); put_le32(g_wav, 0);

	if ((s = AudioUnitInitialize(g_in_au)) != noErr) {
		fprintf(stderr, "AudioUnitInitialize: %d\n", s); return 1; }
	if ((s = AudioOutputUnitStart(g_in_au)) != noErr) {
		fprintf(stderr, "AudioOutputUnitStart: %d\n", s); return 1; }

	usleep((useconds_t)(seconds * 1e6));

	AudioOutputUnitStop(g_in_au);
	AudioUnitUninitialize(g_in_au);
	AudioComponentInstanceDispose(g_in_au);

	uint32_t data_bytes = g_frames_written * 6;
	fseek(g_wav, 4, SEEK_SET);  put_le32(g_wav, 36 + data_bytes);
	fseek(g_wav, 40, SEEK_SET); put_le32(g_wav, data_bytes);
	fclose(g_wav);
	printf("captured %u frames (%.2fs) -> iotest_cap.wav\n",
	       g_frames_written, (double)g_frames_written / RATE);
	return 0;
}

/* ---- loopback latency mode ------------------------------------------ */
/*
 * Output emits one impulse/sec; input watches for it. A SINGLE full-duplex AUHAL
 * does both (two separate units on one device fail to start), so ts->mSampleTime
 * shares one clock domain and the in-minus-out delta is the true round-trip
 * (output presentation + analog loopback + input).
 */
#define LAT_IMP_LEN     ((int)(RATE / 1000))  /* impulse width: 1 ms at any rate */
#define LAT_IMP_AMP     0.95f      /* near full-scale for a strong loopback */
static float g_lat_thresh = 0.05f; /* input detection threshold (--thresh X) */
#define LAT_THRESH      g_lat_thresh
#define LAT_PERIOD      RATE      /* one impulse per second */
#define LAT_TIMEOUT     0.5       /* give up on a pending impulse after this */
#define LAT_MAX         64

enum { LAT_IDLE, LAT_PENDING };
static volatile int    g_lat_state = LAT_IDLE;
static volatile double g_lat_next_target = -1.0;  /* sampletime of next impulse */
static volatile int    g_lat_imp_remaining = 0;
static volatile double g_lat_out_time = 0.0;       /* sampletime of impulse start */
static double g_lat_meas[LAT_MAX];
static volatile int    g_lat_n = 0;
static volatile int    g_lat_emitted = 0;          /* impulses sent on output */
static volatile float  g_lat_inpeak = 0.0f;        /* max |input| seen (any chan) */

static UInt32 g_lat_bufsize = 64;   /* IO buffer frames (Reaper-like; --buf N) */
static AudioUnit g_lat_au;          /* single full-duplex unit */
static AudioBufferList *g_lat_inlist;

static OSStatus lat_out_cb(void *ref, AudioUnitRenderActionFlags *flags,
			   const AudioTimeStamp *ts, UInt32 bus, UInt32 nframes,
			   AudioBufferList *io)
{
	(void)ref; (void)flags; (void)bus;
	float *out = (float *)io->mBuffers[0].mData;
	double t0 = ts->mSampleTime;
	if (g_lat_next_target < 0.0)
		g_lat_next_target = t0 + LAT_PERIOD;  /* 1s warmup */

	for (UInt32 i = 0; i < nframes; i++) {
		double t = t0 + i;
		float v = 0.0f;
		if (g_lat_state == LAT_IDLE && g_lat_imp_remaining == 0 &&
		    t >= g_lat_next_target) {
			g_lat_imp_remaining = LAT_IMP_LEN;
			g_lat_out_time = t;
			g_lat_state = LAT_PENDING;
			g_lat_emitted++;
		}
		if (g_lat_imp_remaining > 0) { v = LAT_IMP_AMP; g_lat_imp_remaining--; }
		out[i * 2 + 0] = v;
		out[i * 2 + 1] = v;
	}
	/* time out a pending impulse that was never detected (e.g. no cable) */
	if (g_lat_state == LAT_PENDING &&
	    (t0 + nframes) - g_lat_out_time > LAT_TIMEOUT * RATE) {
		g_lat_state = LAT_IDLE;
		g_lat_next_target += LAT_PERIOD;
	}
	return noErr;
}

static OSStatus lat_in_cb(void *ref, AudioUnitRenderActionFlags *flags,
			  const AudioTimeStamp *ts, UInt32 bus, UInt32 nframes,
			  AudioBufferList *unused)
{
	(void)ref; (void)unused;
	g_lat_inlist->mBuffers[0].mDataByteSize = nframes * 8;
	if (AudioUnitRender(g_lat_au, flags, ts, bus, nframes, g_lat_inlist) != noErr)
		return noErr;
	const float *in = (const float *)g_lat_inlist->mBuffers[0].mData;
	for (UInt32 i = 0; i < nframes; i++) {
		float l = in[i * 2], r = in[i * 2 + 1];
		float m = fabsf(l) > fabsf(r) ? fabsf(l) : fabsf(r);
		if (m > g_lat_inpeak) g_lat_inpeak = m;   /* always track level */
		if (g_lat_state == LAT_PENDING && m > LAT_THRESH) {
			double in_time = ts->mSampleTime + i;
			double lat = in_time - g_lat_out_time;
			if (lat > 0 && g_lat_n < LAT_MAX)
				g_lat_meas[g_lat_n++] = lat;
			g_lat_state = LAT_IDLE;
			g_lat_next_target += LAT_PERIOD;
		}
	}
	return noErr;
}

static int cmp_d(const void *a, const void *b) {
	double x = *(const double *)a, y = *(const double *)b;
	return (x > y) - (x < y);
}

static int run_latency(AudioDeviceID dev, double seconds)
{
	OSStatus s;
	AudioComponentDescription desc = {
		kAudioUnitType_Output, kAudioUnitSubType_HALOutput,
		kAudioUnitManufacturer_Apple, 0, 0 };
	AudioStreamBasicDescription fmt = {
		.mSampleRate = RATE, .mFormatID = kAudioFormatLinearPCM,
		.mFormatFlags = kAudioFormatFlagIsFloat | kAudioFormatFlagIsPacked,
		.mBytesPerPacket = 8, .mFramesPerPacket = 1, .mBytesPerFrame = 8,
		.mChannelsPerFrame = 2, .mBitsPerChannel = 32 };

	/* --- single full-duplex unit (input elem 1 + output elem 0) --- */
	AudioComponent comp = AudioComponentFindNext(NULL, &desc);
	if (AudioComponentInstanceNew(comp, &g_lat_au) != noErr) return 1;
	UInt32 one = 1;
	AudioUnitSetProperty(g_lat_au, kAudioOutputUnitProperty_EnableIO,
			kAudioUnitScope_Input, 1, &one, sizeof(one));
	/* output (elem 0) is enabled by default on a HALOutput unit */
	AudioUnitSetProperty(g_lat_au, kAudioOutputUnitProperty_CurrentDevice,
			kAudioUnitScope_Global, 0, &dev, sizeof(dev));
	/* Match Reaper's small IO buffer (cushion floor ~= buffer size). */
	AudioObjectPropertyAddress bfa = { kAudioDevicePropertyBufferFrameSize,
			kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain };
	if (AudioObjectSetPropertyData(dev, &bfa, 0, NULL,
			sizeof(g_lat_bufsize), &g_lat_bufsize) != noErr)
		fprintf(stderr, "warn: could not set buffer frame size %u\n", g_lat_bufsize);
	UInt32 actual = 0, asz = sizeof(actual);
	AudioObjectGetPropertyData(dev, &bfa, 0, NULL, &asz, &actual);
	printf("IO buffer = %u frames\n", actual);
	/* output side: format we feed (input scope, elem 0) */
	AudioUnitSetProperty(g_lat_au, kAudioUnitProperty_StreamFormat,
			kAudioUnitScope_Input, 0, &fmt, sizeof(fmt));
	/* input side: format we receive (output scope, elem 1) */
	AudioUnitSetProperty(g_lat_au, kAudioUnitProperty_StreamFormat,
			kAudioUnitScope_Output, 1, &fmt, sizeof(fmt));
	AURenderCallbackStruct ocb = { lat_out_cb, NULL };
	AudioUnitSetProperty(g_lat_au, kAudioUnitProperty_SetRenderCallback,
			kAudioUnitScope_Input, 0, &ocb, sizeof(ocb));
	AURenderCallbackStruct icb = { lat_in_cb, NULL };
	AudioUnitSetProperty(g_lat_au, kAudioOutputUnitProperty_SetInputCallback,
			kAudioUnitScope_Global, 0, &icb, sizeof(icb));
	g_lat_inlist = calloc(1, sizeof(AudioBufferList) + sizeof(AudioBuffer));
	g_lat_inlist->mNumberBuffers = 1;
	g_lat_inlist->mBuffers[0].mNumberChannels = 2;
	g_lat_inlist->mBuffers[0].mData = malloc(8 * 4096);

	if ((s = AudioUnitInitialize(g_lat_au)) != noErr) {
		fprintf(stderr, "Initialize: %d\n", s);
		AudioComponentInstanceDispose(g_lat_au);
		return 1;
	}
	if ((s = AudioOutputUnitStart(g_lat_au)) != noErr) {
		fprintf(stderr, "Start: %d\n", s);
		AudioUnitUninitialize(g_lat_au);
		AudioComponentInstanceDispose(g_lat_au);
		return 1;
	}

	printf("measuring loopback latency for %.0fs "
	       "(patch RK3 output -> input)...\n", seconds);
	usleep((useconds_t)(seconds * 1e6));

	AudioOutputUnitStop(g_lat_au);
	AudioUnitUninitialize(g_lat_au);
	AudioComponentInstanceDispose(g_lat_au);

	int n = g_lat_n;
	printf("emitted %d impulses; peak input level seen = %.4f (%.1f dBFS)\n",
	       g_lat_emitted, g_lat_inpeak,
	       g_lat_inpeak > 0 ? 20.0 * log10(g_lat_inpeak) : -99.0);
	if (n == 0) {
		printf("NO impulses detected (threshold %.3f).\n", (double)LAT_THRESH);
		if (g_lat_inpeak < LAT_THRESH)
			printf("  -> input peak is below threshold: signal too weak or "
			       "not reaching the input.\n     Check the cable jacks and "
			       "raise the RK3 input gain, then retry.\n");
		else
			printf("  -> input HAS signal above threshold but it never lined "
			       "up with an emitted impulse;\n     unexpected, tell the "
			       "developer.\n");
		return 1;
	}
	printf("  raw:");
	for (int i = 0; i < n; i++) printf(" %.0f", g_lat_meas[i]);
	printf("\n");
	qsort(g_lat_meas, n, sizeof(double), cmp_d);
	double sum = 0; for (int i = 0; i < n; i++) sum += g_lat_meas[i];
	double med = g_lat_meas[n / 2];
	printf("round-trip latency over %d impulses:\n", n);
	printf("  min    %8.1f spls  %6.2f ms\n", g_lat_meas[0],   g_lat_meas[0]   / RATE * 1000);
	printf("  median %8.1f spls  %6.2f ms\n", med,             med             / RATE * 1000);
	printf("  mean   %8.1f spls  %6.2f ms\n", sum / n,         sum / n         / RATE * 1000);
	printf("  max    %8.1f spls  %6.2f ms\n", g_lat_meas[n-1], g_lat_meas[n-1] / RATE * 1000);
	return 0;
}

int main(int argc, char **argv)
{
	double seconds = 4.0;
	int do_input = 0, do_latency = 0;
	double want_rate = 0.0;
	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "--in") == 0) do_input = 1;
		else if (strcmp(argv[i], "--latency") == 0) do_latency = 1;
		else if (strcmp(argv[i], "--buf") == 0 && i + 1 < argc)
			g_lat_bufsize = (UInt32)atoi(argv[++i]);
		else if (strcmp(argv[i], "--thresh") == 0 && i + 1 < argc)
			g_lat_thresh = (float)atof(argv[++i]);
		else if (strcmp(argv[i], "--rate") == 0 && i + 1 < argc)
			want_rate = atof(argv[++i]);
		else seconds = atof(argv[i]);
	}

	AudioDeviceID dev = find_device();
	if (dev == kAudioObjectUnknown) {
		fprintf(stderr, "RigKontrol 3 device not found (driver installed?)\n");
		return 1;
	}

	AudioObjectPropertyAddress ra = { kAudioDevicePropertyNominalSampleRate,
		kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain };
	Float64 cur_rate = 0; UInt32 rsz = sizeof(cur_rate);
	AudioObjectGetPropertyData(dev, &ra, 0, NULL, &rsz, &cur_rate);
	if (want_rate > 0 && want_rate != cur_rate) {
		if (AudioObjectSetPropertyData(dev, &ra, 0, NULL,
				sizeof(want_rate), &want_rate) != noErr)
			fprintf(stderr, "warn: could not set rate %.0f\n", want_rate);
		sleep(2);                     /* let the config change land */
		rsz = sizeof(cur_rate);
		AudioObjectGetPropertyData(dev, &ra, 0, NULL, &rsz, &cur_rate);
	}
	if (cur_rate > 0) g_rate = cur_rate;
	printf("device rate = %.0f Hz\n", g_rate);

	if (do_latency) {
		if (seconds < 3.0) seconds = 6.0;  /* need a few impulses */
		printf("RigKontrol 3 = AudioDeviceID %u\n", dev);
		return run_latency(dev, seconds);
	}

	if (do_input) {
		printf("RigKontrol 3 = AudioDeviceID %u; capturing for %.1fs\n",
		       dev, seconds);
		return run_input(dev, seconds);
	}

	printf("RigKontrol 3 = AudioDeviceID %u; playing 440 Hz for %.1fs\n",
	       dev, seconds);

	AudioComponentDescription desc = {
		kAudioUnitType_Output, kAudioUnitSubType_HALOutput,
		kAudioUnitManufacturer_Apple, 0, 0 };
	AudioComponent comp = AudioComponentFindNext(NULL, &desc);
	AudioUnit au = NULL;
	if (AudioComponentInstanceNew(comp, &au) != noErr) {
		fprintf(stderr, "AudioComponentInstanceNew failed\n"); return 1;
	}

	OSStatus s;
	s = AudioUnitSetProperty(au, kAudioOutputUnitProperty_CurrentDevice,
			kAudioUnitScope_Global, 0, &dev, sizeof(dev));
	if (s != noErr) { fprintf(stderr, "set CurrentDevice: %d\n", s); return 1; }

	AudioStreamBasicDescription fmt = {
		.mSampleRate = RATE,
		.mFormatID = kAudioFormatLinearPCM,
		.mFormatFlags = kAudioFormatFlagIsFloat | kAudioFormatFlagIsPacked,
		.mBytesPerPacket = 8, .mFramesPerPacket = 1, .mBytesPerFrame = 8,
		.mChannelsPerFrame = 2, .mBitsPerChannel = 32 };
	s = AudioUnitSetProperty(au, kAudioUnitProperty_StreamFormat,
			kAudioUnitScope_Input, 0, &fmt, sizeof(fmt));
	if (s != noErr) { fprintf(stderr, "set StreamFormat: %d\n", s); return 1; }

	AURenderCallbackStruct cb = { render_cb, NULL };
	s = AudioUnitSetProperty(au, kAudioUnitProperty_SetRenderCallback,
			kAudioUnitScope_Input, 0, &cb, sizeof(cb));
	if (s != noErr) { fprintf(stderr, "set RenderCallback: %d\n", s); return 1; }

	if ((s = AudioUnitInitialize(au)) != noErr) {
		fprintf(stderr, "AudioUnitInitialize: %d\n", s); return 1;
	}
	if ((s = AudioOutputUnitStart(au)) != noErr) {
		fprintf(stderr, "AudioOutputUnitStart: %d\n", s); return 1;
	}

	usleep((useconds_t)(seconds * 1e6));

	AudioOutputUnitStop(au);
	AudioUnitUninitialize(au);
	AudioComponentInstanceDispose(au);
	printf("done.\n");
	return 0;
}
