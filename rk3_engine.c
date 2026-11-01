/* rk3_engine.c — see rk3_engine.h */

#include "rk3_engine.h"
#include "runloop_sync.h"
#include "rk3_midi.h"          /* RK3_MIDI_UDP_PORT */

#include <os/log.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <mach/mach_time.h>
#include <mach/mach.h>
#include <mach/thread_policy.h>
#include <mach/thread_act.h>

static os_log_t eng_log(void)
{
	static os_log_t l;
	static dispatch_once_t once;
	dispatch_once(&once, ^{ l = os_log_create("com.mihai.rk3", "engine"); });
	return l;
}
#define ELOG(fmt, ...) os_log(eng_log(), "[rk3] " fmt, ##__VA_ARGS__)

/* ---- ring helpers (single producer + single consumer per ring) ------- */

static inline unsigned ring_avail(unsigned w, unsigned r)
{
	return (w - r + RK3_RING_BYTES) % RK3_RING_BYTES;   /* bytes readable */
}
static inline unsigned ring_space(unsigned w, unsigned r)
{
	/* keep one byte free so full != empty */
	return RK3_RING_BYTES - 1 - ring_avail(w, r);
}

/* ---- cadence mirror (engine-thread only) ----------------------------- */

static int pat_count(struct rk3_engine *e) { return e->pat_head - e->pat_tail; }

static void pat_push(struct rk3_engine *e, struct rk3_iso_xfer *x)
{
	if (pat_count(e) >= RK3_PAT_RING)
		e->pat_tail++;
	int *slot = e->pat[e->pat_head % RK3_PAT_RING];
	for (int f = 0; f < RK3_FRAMES_PER_XFER; f++)
		slot[f] = x->frames[f].frActCount;
	e->pat_head++;
}

/* Synthesize the device's bursty 88/0 cadence for one transfer (startup). */
static void pat_synth(struct rk3_engine *e, int *lens)
{
	int pkt_frames = e->bpp / 8;
	int need = e->rate / 8000;
	int rem  = e->rate % 8000;
	for (int f = 0; f < RK3_FRAMES_PER_XFER; f++) {
		e->out_acc += need * 8000 + rem;
		if (e->out_acc >= pkt_frames * 8000) {
			e->out_acc -= pkt_frames * 8000;
			lens[f] = e->bpp;
		} else {
			lens[f] = 0;
		}
	}
}

static void pat_pop(struct rk3_engine *e, int *lens)
{
	if (pat_count(e) > 0) {
		int *slot = e->pat[e->pat_tail % RK3_PAT_RING];
		for (int f = 0; f < RK3_FRAMES_PER_XFER; f++)
			lens[f] = slot[f];
		e->pat_tail++;
	} else {
		pat_synth(e, lens);
	}
}

/*
 * Pop one OUT transfer's worth (RK3_OUT_FRAMES_PER_XFER microframes) of cadence,
 * refilling the IN-sized staging block as it drains. RK3_OUT_FRAMES_PER_XFER
 * divides RK3_FRAMES_PER_XFER, so each captured IN block feeds an exact integer
 * number of OUT transfers — the OUT chain stays rate-locked to the device clock
 * even though it ships smaller transfers than capture.
 */
static void pat_pop_out(struct rk3_engine *e, int *lens)
{
	if (e->pat_stage_off >= RK3_FRAMES_PER_XFER) {
		pat_pop(e, e->pat_stage);
		e->pat_stage_off = 0;
	}
	for (int f = 0; f < RK3_OUT_FRAMES_PER_XFER; f++)
		lens[f] = e->pat_stage[e->pat_stage_off + f];
	e->pat_stage_off += RK3_OUT_FRAMES_PER_XFER;
}

/* ---- isoc completion callbacks (ported from stream.c) ---------------- */

static void resubmit_in(struct rk3_iso_xfer *x);
static int  submit_out(struct rk3_iso_xfer *x);
static void fill_out(struct rk3_engine *e, struct rk3_iso_xfer *x);
static void out_done(void *refcon, IOReturn result, void *arg0);
static void in_done(void *refcon, IOReturn result, void *arg0);

/*
 * Isoc submit wrappers — the ONLY place the basic vs low-latency API divergence
 * lives. The callback bodies (in_done/fill_out/out_done/...) access x->buf and
 * x->frames[f] identically either way; only the submit call name + the extra
 * updateFrequency arg (and the kernel-shared buffer allocation) differ. See
 * RK3_USE_LOWLAT in rk3_engine.h.
 */
static IOReturn rk3_read_isoc(struct rk3_engine *e, struct rk3_iso_xfer *x,
			      UInt32 nframes)
{
#if RK3_USE_LOWLAT
	return (*e->usb.intf)->LowLatencyReadIsochPipeAsync(e->usb.intf,
			e->usb.pipe_iso_in, x->buf, x->frame_start, nframes,
			RK3_LL_UPDATE_FREQ, x->frames, in_done, x);
#else
	return (*e->usb.intf)->ReadIsochPipeAsync(e->usb.intf,
			e->usb.pipe_iso_in, x->buf, x->frame_start, nframes,
			x->frames, in_done, x);
#endif
}

static IOReturn rk3_write_isoc(struct rk3_engine *e, struct rk3_iso_xfer *x,
			       UInt32 nframes)
{
#if RK3_USE_LOWLAT
	return (*e->usb.intf)->LowLatencyWriteIsochPipeAsync(e->usb.intf,
			e->usb.pipe_iso_out, x->buf, x->frame_start, nframes,
			RK3_LL_UPDATE_FREQ, x->frames, out_done, x);
#else
	return (*e->usb.intf)->WriteIsochPipeAsync(e->usb.intf,
			e->usb.pipe_iso_out, x->buf, x->frame_start, nframes,
			x->frames, out_done, x);
#endif
}

#if RK3_USE_LOWLAT
/*
 * Allocate one transfer's kernel-shared data + frame-list buffers. read=1 for
 * the capture pipe, 0 for playback. bufsz/nframes are direction-specific (IN is
 * full size, OUT the smaller OUT_ values). Returns 0 on success. The basic path
 * uses inline arrays and needs no allocation, so these are LL-only.
 */
static int ll_alloc_xfer(struct rk3_engine *e, struct rk3_iso_xfer *x,
			 size_t bufsz, int nframes, int read)
{
	IOReturn kr = (*e->usb.intf)->LowLatencyCreateBuffer(e->usb.intf,
			(void **)&x->buf, bufsz,
			read ? kUSBLowLatencyReadBuffer : kUSBLowLatencyWriteBuffer);
	if (kr != kIOReturnSuccess || !x->buf) {
		ELOG("LowLatencyCreateBuffer(data) 0x%08x", kr);
		x->buf = NULL;
		return -1;
	}
	kr = (*e->usb.intf)->LowLatencyCreateBuffer(e->usb.intf,
			(void **)&x->frames,
			(IOByteCount)(nframes * sizeof(rk3_isoc_frame_t)),
			kUSBLowLatencyFrameListBuffer);
	if (kr != kIOReturnSuccess || !x->frames) {
		ELOG("LowLatencyCreateBuffer(framelist) 0x%08x", kr);
		x->frames = NULL;
		return -1;
	}
	memset(x->buf, 0, bufsz);
	return 0;
}

static void ll_free_xfer(struct rk3_engine *e, struct rk3_iso_xfer *x)
{
	if (x->buf) {
		(*e->usb.intf)->LowLatencyDestroyBuffer(e->usb.intf, x->buf);
		x->buf = NULL;
	}
	if (x->frames) {
		(*e->usb.intf)->LowLatencyDestroyBuffer(e->usb.intf, x->frames);
		x->frames = NULL;
	}
}

/* Allocate all isoc buffers for a stream start; frees partial on failure. */
static int ll_alloc_all(struct rk3_engine *e)
{
	for (int i = 0; i < RK3_N_XFERS; i++)
		if (ll_alloc_xfer(e, &e->in[i], RK3_XFER_BUF,
				  RK3_FRAMES_PER_XFER, 1) != 0)
			return -1;
	for (int i = 0; i < RK3_N_OUT_XFERS; i++)
		if (ll_alloc_xfer(e, &e->out[i], RK3_OUT_XFER_BUF,
				  RK3_OUT_FRAMES_PER_XFER, 0) != 0)
			return -1;
	return 0;
}

static void ll_free_all(struct rk3_engine *e)
{
	for (int i = 0; i < RK3_N_XFERS; i++)
		ll_free_xfer(e, &e->in[i]);
	for (int i = 0; i < RK3_N_OUT_XFERS; i++)
		ll_free_xfer(e, &e->out[i]);
}
#endif /* RK3_USE_LOWLAT */

#define SCHED_MARGIN 4
static UInt64 schedule_frame(struct rk3_engine *e, UInt64 want)
{
	UInt64 cur = 0;
	AbsoluteTime at;
	(*e->usb.intf)->GetBusFrameNumber(e->usb.intf, &cur, &at);
	if (want < cur + SCHED_MARGIN)
		want = cur + SCHED_MARGIN;
	return want;
}

static void in_done(void *refcon, IOReturn result, void *arg0)
{
	struct rk3_iso_xfer *x = refcon;
	struct rk3_engine *e = x->e;
	(void)result; (void)arg0;

	if (!atomic_load_explicit(&e->audio_running, memory_order_acquire))
		return;

	/* Deinterleave each microframe into a scratch then push to the cap ring. */
	uint8_t scratch[RK3_XFER_BUF];
	int pos = 0;
	for (int f = 0; f < RK3_FRAMES_PER_XFER; f++) {
		int act = x->frames[f].frActCount;
		if (act <= 0)
			continue;
		caiaq_capture_mode2(&e->codec, x->buf + f * RK3_FRAME_BYTES, act,
				    scratch, &pos, sizeof(scratch));
	}
	if (pos > 0) {
		/* device clock: count frames actually produced, even if the ring drops them */
		atomic_fetch_add_explicit(&e->dev_frames,
				(unsigned)(pos / RK3_BYTES_PER_FRAME), memory_order_relaxed);
		unsigned w = atomic_load_explicit(&e->cap_w, memory_order_relaxed);
		unsigned r = atomic_load_explicit(&e->cap_r, memory_order_acquire);
		if (ring_space(w, r) < (unsigned)pos) {
			atomic_fetch_add_explicit(&e->cap_overruns, 1,
						  memory_order_relaxed);
		} else {
			unsigned first = RK3_RING_BYTES - w;
			if (first > (unsigned)pos) first = pos;
			memcpy(e->cap_buf + w, scratch, first);
			if ((unsigned)pos > first)
				memcpy(e->cap_buf, scratch + first, pos - first);
			atomic_store_explicit(&e->cap_w,
				(w + pos) % RK3_RING_BYTES, memory_order_release);
		}
	}

	/* mirror this capture's per-microframe cadence for playback */
	pat_push(e, x);

	resubmit_in(x);
}

/*
 * Fill one OUT transfer from the playback ring. Pops the captured cadence
 * (rate-locked to the device), guarantees a non-zero trailing frame, and packs
 * the audio CONTIGUOUSLY by frReqCount via caiaq_playback_mode2.
 *
 * Startup gate: until CoreAudio has buffered RK3_OUT_CUSHION (pb_started, set in
 * write_output), the engine sends digital silence and does NOT touch pb_r. Once
 * started, pb_r is read from the ring at RK3_OUT_PHASE (set in write_output) and
 * advances continuously, preserving the proven 4-byte OUT phase across packets.
 */
static void fill_out(struct rk3_engine *e, struct rk3_iso_xfer *x)
{
	int lens[RK3_OUT_FRAMES_PER_XFER];
	pat_pop_out(e, lens);

	if (lens[RK3_OUT_FRAMES_PER_XFER - 1] == 0) {
		int j = RK3_OUT_FRAMES_PER_XFER - 2;
		while (j >= 0 && lens[j] == 0)
			j--;
		if (j >= 0) {
			lens[RK3_OUT_FRAMES_PER_XFER - 1] = lens[j];
			lens[j] = 0;
		}
	}

	int started = atomic_load_explicit(&e->pb_started, memory_order_acquire);
	unsigned r = atomic_load_explicit(&e->pb_r, memory_order_relaxed);
	unsigned w = atomic_load_explicit(&e->pb_w, memory_order_acquire);
	unsigned avail = started ? ring_avail(w, r) : 0;

	int off = 0;
	for (int f = 0; f < RK3_OUT_FRAMES_PER_XFER; f++) {
		int len = lens[f];
		if (len > RK3_FRAME_BYTES) len = RK3_FRAME_BYTES;
		x->frames[f].frReqCount = len;
		x->frames[f].frActCount = 0;
		x->frames[f].frStatus = 0;
		if (len > 0) {
			int audio = len - (len + 1) / 4;   /* audio bytes in pkt */
			if (avail < (unsigned)audio) {
				/* not started yet, or underrun: emit silence */
				if (started)
					atomic_fetch_add_explicit(&e->pb_underruns,
							1, memory_order_relaxed);
				caiaq_playback_mode2(&e->codec, x->buf + off, len,
						     NULL, NULL, 0);
			} else {
				caiaq_playback_mode2(&e->codec, x->buf + off, len,
						     e->pb_buf, (int *)&r,
						     RK3_RING_BYTES);
				avail -= audio;
			}
		}
		off += len;
	}
	if (started) {
		atomic_store_explicit(&e->pb_r, r, memory_order_release);
		/* windowed-min out_fill telemetry: post-consume trough (this is the
		 * sample right after the OUT bite, i.e. closest to underrun). */
		unsigned wf = atomic_load_explicit(&e->pb_w, memory_order_relaxed);
		unsigned of = ring_avail(wf, r) / RK3_BYTES_PER_FRAME;
		if (of < e->out_fill_min) e->out_fill_min = of;
	}
}

static int submit_out(struct rk3_iso_xfer *x)
{
	struct rk3_engine *e = x->e;
	for (int tries = 0; tries < 8; tries++) {
		IOReturn kr = rk3_write_isoc(e, x, RK3_OUT_FRAMES_PER_XFER);
		if (kr == kIOReturnSuccess)
			return 0;
		e->out_errs++;
		UInt64 cur = 0;
		AbsoluteTime at;
		(*e->usb.intf)->GetBusFrameNumber(e->usb.intf, &cur, &at);
		UInt64 floor = cur + RK3_OUT_MIN_LEAD;
		x->frame_start = (x->frame_start < floor)
				 ? floor : x->frame_start + RK3_OUT_FRAME_STRIDE;
	}
	e->out_drops++;
	return -1;
}

static void out_done(void *refcon, IOReturn result, void *arg0)
{
	struct rk3_iso_xfer *x = refcon;
	struct rk3_engine *e = x->e;
	(void)result; (void)arg0;
	if (!atomic_load_explicit(&e->audio_running, memory_order_acquire))
		return;
	x->frame_start += (UInt64)RK3_N_OUT_XFERS * RK3_OUT_FRAME_STRIDE;
	fill_out(e, x);
	submit_out(x);

	/*
	 * Drift-vs-jitter probe. out_done fires once per OUT transfer (every
	 * RK3_OUT_FRAME_STRIDE ms-frame = 8000/RK3_OUT_FRAMES_PER_XFER per sec)
	 * regardless of N_OUT_XFERS, so tick every (8000/OUT_FPX) calls ≈ 1 second. Log
	 * the OUT ring fill, the CAPTURE ring fill (so we catch a silent input-latency
	 * balloon — the IN ring growing if CoreAudio drains it slower than the device
	 * fills it), and the CUMULATIVE measured device rate from a FIXED anchor
	 * (converges as the run lengthens). Monotonic out_fill decline => drift; noisy
	 * around a stable mean with underruns on dips => jitter/margin.
	 */
	if (e->dbg_host0 == 0) {
		e->dbg_host0 = mach_absolute_time();
		e->dbg_devframes0 = atomic_load_explicit(&e->dev_frames, memory_order_relaxed);
	}
	if (++e->dbg_ctr >= (unsigned)(8000 / RK3_OUT_FRAMES_PER_XFER)) {
		e->dbg_ctr = 0;
		static mach_timebase_info_data_t tb;
		if (tb.denom == 0) mach_timebase_info(&tb);
		UInt64 now = mach_absolute_time();
		double elapsed_s = (double)(now - e->dbg_host0) * tb.numer / tb.denom / 1e9;
		unsigned w = atomic_load_explicit(&e->pb_w, memory_order_acquire);
		unsigned r = atomic_load_explicit(&e->pb_r, memory_order_relaxed);
		unsigned fill = ring_avail(w, r) / RK3_BYTES_PER_FRAME;
		unsigned cw = atomic_load_explicit(&e->cap_w, memory_order_acquire);
		unsigned cr = atomic_load_explicit(&e->cap_r, memory_order_relaxed);
		unsigned capfill = ring_avail(cw, cr) / RK3_BYTES_PER_FRAME;
		unsigned long long df = atomic_load_explicit(&e->dev_frames, memory_order_relaxed);
		double rate = elapsed_s > 0 ?
			(double)(df - e->dbg_devframes0) / elapsed_s : 0.0;
		unsigned ofmin = e->out_fill_min, cfmin = e->cap_fill_min;
		e->out_fill_min = ~0u;
		e->cap_fill_min = ~0u;
		ELOG("clk: out_fill=%u (min %u) cap_fill=%u (min %u)  dev_rate=%.2f Hz (nominal %d)  underruns=%u",
		     fill, ofmin == ~0u ? 0 : ofmin, capfill, cfmin == ~0u ? 0 : cfmin,
		     rate, e->rate, atomic_load(&e->pb_underruns));
	}
}

static void resubmit_in(struct rk3_iso_xfer *x)
{
	struct rk3_engine *e = x->e;
	x->frame_start = schedule_frame(e, x->frame_start + RK3_N_XFERS * RK3_FRAME_STRIDE);
	for (int f = 0; f < RK3_FRAMES_PER_XFER; f++) {
		x->frames[f].frReqCount = RK3_FRAME_BYTES;
		x->frames[f].frActCount = 0;
		x->frames[f].frStatus = 0;
	}
	IOReturn kr = rk3_read_isoc(e, x, RK3_FRAMES_PER_XFER);
	if (kr != kIOReturnSuccess &&
	    atomic_load_explicit(&e->audio_running, memory_order_acquire)) {
		e->in_errs++;
		x->frame_start = schedule_frame(e, 0);
		rk3_read_isoc(e, x, RK3_FRAMES_PER_XFER);
	}
}

/* ---- engine thread --------------------------------------------------- */

static int prime_chains(struct rk3_engine *e)
{
#if RK3_USE_LOWLAT
	/* Allocate this start's kernel-shared isoc buffers (freed in audio_stop). */
	if (ll_alloc_all(e) != 0) {
		ELOG("prime: LowLatency buffer allocation failed");
		ll_free_all(e);
		return -1;
	}
#endif

	UInt64 frame = 0;
	AbsoluteTime atTime;
	(*e->usb.intf)->GetBusFrameNumber(e->usb.intf, &frame, &atTime);
	UInt64 start = frame + 10;

	for (int i = 0; i < RK3_N_XFERS; i++) {
		e->in[i].e = e;
		e->in[i].index = i;
		e->in[i].frame_start = start + (UInt64)i * RK3_FRAME_STRIDE;
		for (int f = 0; f < RK3_FRAMES_PER_XFER; f++)
			e->in[i].frames[f].frReqCount = RK3_FRAME_BYTES;
		IOReturn kr = rk3_read_isoc(e, &e->in[i], RK3_FRAMES_PER_XFER);
		if (kr != kIOReturnSuccess)
			ELOG("prime IN %d: 0x%08x", i, kr);
	}

	for (int i = 0; i < RK3_N_OUT_XFERS; i++) {
		e->out[i].e = e;
		e->out[i].index = i;
		e->out[i].frame_start = start + (UInt64)i * RK3_OUT_FRAME_STRIDE;
		fill_out(e, &e->out[i]);
		if (submit_out(&e->out[i]) != 0)
			ELOG("prime OUT %d failed", i);
	}
	return 0;
}

/*
 * Promote the engine's USB/CFRunLoop thread to time-constraint (real-time)
 * scheduling so isoc completion callbacks are serviced on time even under DAW
 * load. Insurance for the smaller OUT transfers (higher completion rate): the
 * storm risk is a default-priority thread falling behind the completion stream.
 * preemptible=1 so we still yield to CoreAudio's own real-time IO thread.
 * Best-effort; a failure here is non-fatal (logged, engine runs default-sched).
 */
static void engine_set_realtime(void)
{
	mach_timebase_info_data_t tb;
	mach_timebase_info(&tb);
	double abs_per_ms = 1e6 * (double)tb.denom / (double)tb.numer;
	thread_time_constraint_policy_data_t pol;
	pol.period      = 0;                              /* aperiodic (event-driven) */
	pol.computation = (uint32_t)(0.5 * abs_per_ms);  /* ~0.5 ms to service a cb */
	pol.constraint  = (uint32_t)(2.0 * abs_per_ms);  /* deadline ~2 ms */
	pol.preemptible = 1;
	kern_return_t kr = thread_policy_set(mach_thread_self(),
			THREAD_TIME_CONSTRAINT_POLICY, (thread_policy_t)&pol,
			THREAD_TIME_CONSTRAINT_POLICY_COUNT);
	if (kr != KERN_SUCCESS)
		ELOG("thread_policy_set RT failed: 0x%x", kr);
}

/* ---- MIDI relay: EP 0x81 control reports -> loopback UDP -> helper --- */

static void midi_arm_read(struct rk3_engine *e);
static void led_write_done(void *refcon, IOReturn result, void *arg0);

/* Open a UDP socket connected to the helper's loopback port. Best-effort. */
static void midi_relay_open(struct rk3_engine *e)
{
	e->midi_fd = socket(AF_INET, SOCK_DGRAM, 0);
	if (e->midi_fd < 0) {
		ELOG("midi relay: socket failed errno=%d (no MIDI this session)", errno);
		return;
	}
	struct sockaddr_in to;
	memset(&to, 0, sizeof(to));
	to.sin_family = AF_INET;
	to.sin_port = htons(RK3_MIDI_UDP_PORT);
	to.sin_addr.s_addr = htonl(INADDR_LOOPBACK);   /* 127.0.0.1 */
	/* connect() a datagram socket so later sends need no address and a dead
	 * peer surfaces as a send error we can ignore rather than a routing cost. */
	if (connect(e->midi_fd, (struct sockaddr *)&to, sizeof(to)) != 0) {
		ELOG("midi relay: connect failed errno=%d (no MIDI this session)", errno);
		close(e->midi_fd);
		e->midi_fd = -1;
		return;
	}
	ELOG("midi relay: forwarding EP1 control reports to 127.0.0.1:%d", RK3_MIDI_UDP_PORT);
}

/*
 * Storm de-risk. The relay is now always-on (no sentinel), so the old
 * "disable + latch off" guard would have killed MIDI for the whole session with
 * no way back. Instead we THROTTLE the re-arm: midi_arm_read stamps midi_last_arm
 * and midi_read_done re-arms inline only when >= RK3_MIDI_REARM_MIN_MS has passed,
 * otherwise it defers via midi_rearm_timer. This bounds the EP 0x81 re-arm rate to
 * <= ~1 kHz regardless of how fast completions fire, so a runaway physically can't
 * peg the CFRunLoop / storm coreaudiod. Real input (~100/s, >=10 ms apart) always
 * clears the floor and re-arms inline => zero added latency. RK3_MIDI_CB_WINDOW is
 * kept only for a rate-limited WARNING log (telemetry), it no longer disables.
 */
#define RK3_MIDI_REARM_MIN_MS  1.0
#define RK3_MIDI_CB_WINDOW     1024
#define RK3_MIDI_CB_WARN_MS    50.0

/* Milliseconds elapsed since mach time t0 (0 if t0 is 0). */
static double mach_ms_since(UInt64 t0)
{
	static mach_timebase_info_data_t tb;
	if (tb.denom == 0)
		mach_timebase_info(&tb);
	if (!t0)
		return 1e9;
	return (double)(mach_absolute_time() - t0) * tb.numer / tb.denom / 1e6;
}

/* Async completion for the EP 0x81 read: forward the report, then re-arm. */
static void midi_read_done(void *refcon, IOReturn result, void *arg0)
{
	struct rk3_engine *e = refcon;
	if (!atomic_load_explicit(&e->midi_running, memory_order_acquire))
		return;
	if (result == kIOReturnAborted)
		return;                       /* teardown: stop the chain */

	/*
	 * Device-gone detection. When the RK3 is unplugged, the armed read
	 * completes with a not-attached/not-responding code. Mark the engine USB
	 * dead and stop the chain (no re-arm) so the plugin supervisor can close
	 * and reopen the engine on the next attach. Not a storm-latch: a fresh
	 * open should re-arm normally.
	 */
	if (result == kIOReturnNoDevice || result == kIOReturnNotResponding ||
	    result == kIOReturnNotAttached) {
		ELOG("midi relay: device gone (result=0x%08x) — USB dead, stopping relay",
		     result);
		atomic_store_explicit(&e->usb_dead, 1, memory_order_release);
		atomic_store_explicit(&e->midi_running, 0, memory_order_release);
		return;
	}

	if (result == kIOReturnSuccess) {
		UInt32 n = (UInt32)(uintptr_t)arg0;   /* bytes read */
		if (n > 0 && e->midi_fd >= 0) {
			if (send(e->midi_fd, e->midi_buf, n, 0) < 0 &&
			    e->midi_send_errs < 4) {
				ELOG("midi relay: send errno=%d (helper not running?)", errno);
				e->midi_send_errs++;
			}
		}
	}

	/*
	 * Rate-limited WARNING only (telemetry) — the throttle below, not this, is
	 * what bounds the load now. If a full window of completions arrives faster
	 * than a sane control rate it means the device/pipe is misbehaving; log it
	 * once per window so it's visible in driver-log, but keep relaying.
	 */
	if (++e->midi_cb_ctr >= RK3_MIDI_CB_WINDOW) {
		double ms = mach_ms_since(e->midi_cb_t0);
		if (ms < RK3_MIDI_CB_WARN_MS)
			ELOG("midi relay: fast completions (%d cbs in %.2f ms, last "
			     "result=0x%08x) — throttled to ~%g kHz, audio protected",
			     RK3_MIDI_CB_WINDOW, ms, result, 1.0 / RK3_MIDI_REARM_MIN_MS);
		e->midi_cb_t0 = mach_absolute_time();
		e->midi_cb_ctr = 0;
	}

	/*
	 * THROTTLED RE-ARM (storm de-risk). Re-arm inline only if enough time has
	 * passed since the last arm; otherwise defer to midi_rearm_timer so the run
	 * loop can breathe. Caps the re-arm rate to ~1 kHz no matter how fast
	 * completions fire, so a runaway can't peg the CFRunLoop. Real input clears
	 * the floor every time, so it re-arms inline with no added latency.
	 */
	if (mach_ms_since(e->midi_last_arm) >= RK3_MIDI_REARM_MIN_MS) {
		midi_arm_read(e);
	} else if (e->midi_rearm_timer) {
		double wait_ms = RK3_MIDI_REARM_MIN_MS - mach_ms_since(e->midi_last_arm);
		if (wait_ms < 0)
			wait_ms = 0;
		CFRunLoopTimerSetNextFireDate(e->midi_rearm_timer,
				CFAbsoluteTimeGetCurrent() + wait_ms / 1000.0);
	}
}

static void midi_arm_read(struct rk3_engine *e)
{
	e->midi_last_arm = mach_absolute_time();
	IOReturn kr = (*e->usb.intf)->ReadPipeAsync(e->usb.intf, e->usb.pipe_cmd_in,
			e->midi_buf, sizeof(e->midi_buf), midi_read_done, e);
	if (kr != kIOReturnSuccess)
		ELOG("midi relay: ReadPipeAsync arm failed 0x%08x", kr);
}

/* Deferred re-arm: fires on the engine run loop after the throttle interval. */
static void midi_rearm_tick(CFRunLoopTimerRef timer, void *info)
{
	(void)timer;
	struct rk3_engine *e = info;
	if (atomic_load_explicit(&e->midi_running, memory_order_acquire))
		midi_arm_read(e);
}

/*
 * Show "CON" on the 4-digit 7-segment display (cosmetic; mirrors NI's "MIDI
 * mode" indicator). The RK3 LED/display is a control_state byte array sent via
 * EP1_CMD_WRITE_IO: digit N = byte N, and within a digit bit0..6 = segments
 * a,b,c,d,e,f,g (bit7 = decimal point). Confirmed bit6 = the middle bar (g).
 *   'C' = a,d,e,f          = 0x39
 *   'o' = c,d,e,g           = 0x5C
 *   'n' = c,e,g             = 0x54
 * Left-justified on digits 1-3; digit 4 + the LED bytes left dark. Async write
 * on the engine run loop (same as led_recv); best-effort, never disturbs audio.
 */
/* 7-seg glyphs for 0-9 (segments a..g = bits 0..6; bit7 = decimal point). */
static const uint8_t kSegDigit[10] = {
	0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F,
};

/* Issue the current e->led_wbuf payload (digits/LEDs in bytes 1..) as an async
 * WRITE_IO. Best-effort: skipped if a prior write is still in flight. Shared by
 * the boot animation and midi_show_con; the caller fills bytes 1.. first. */
static void led_write_frame(struct rk3_engine *e)
{
	if (e->led_busy)
		return;                       /* a WRITE_IO is already in flight; skip */
	e->led_wbuf[0] = EP1_CMD_WRITE_IO;
	e->led_busy = 1;
	IOReturn kr = (*e->usb.intf)->WritePipeAsync(e->usb.intf, e->usb.pipe_cmd_out,
			e->led_wbuf, (UInt32)EP1_BUFSIZE, led_write_done, e);
	if (kr != kIOReturnSuccess) {
		e->led_busy = 0;
		ELOG("led: display WRITE_IO WritePipeAsync failed 0x%08x", kr);
	}
}

static void midi_show_con(struct rk3_engine *e)
{
	memset(e->led_wbuf + 1, 0, EP1_BUFSIZE - 1);
	e->led_wbuf[1] = 0x39;            /* digit 1: C */
	e->led_wbuf[2] = 0x5C;            /* digit 2: o */
	e->led_wbuf[3] = 0x54;            /* digit 3: n */
	led_write_frame(e);
}

/* Render the live sample rate in kHz (44/48/96/192) left-justified on the
 * 7-seg display: most-significant digit on the leftmost digit. */
static void boot_set_rate(struct rk3_engine *e)
{
	memset(e->led_wbuf + 1, 0, EP1_BUFSIZE - 1);
	int khz = e->rate / 1000;         /* 44, 48, 96, 192 */
	if (khz <= 0)
		khz = 48;
	uint8_t digs[4];
	int n = 0;
	while (khz > 0 && n < 4) { digs[n++] = (uint8_t)(khz % 10); khz /= 10; }
	for (int i = 0; i < n; i++)        /* digs is LSB-first; reverse onto digits */
		e->led_wbuf[1 + i] = kSegDigit[digs[n - 1 - i]];
}

/*
 * Power-on display sequence, stepped once per boot_timer tick (~90 ms) on the
 * engine run loop: the sample rate flashes twice, then it settles on "Con" and
 * the timer self-invalidates. Purely cosmetic and best-effort (uses the same
 * drop-if-busy WRITE_IO path), so it never touches the audio path.
 */
static void boot_anim_tick(CFRunLoopTimerRef t, void *info)
{
	(void)t;
	struct rk3_engine *e = info;
	int s = e->boot_step++;

	switch (s) {
	case 0:                                            /* rate on */
	case 2:
		boot_set_rate(e);
		led_write_frame(e);
		break;
	case 1:                                            /* blank (blink) */
		memset(e->led_wbuf + 1, 0, EP1_BUFSIZE - 1);
		led_write_frame(e);
		break;
	default:                                           /* settle + stop */
		midi_show_con(e);
		if (e->boot_timer) {
			CFRunLoopTimerInvalidate(e->boot_timer);
			CFRelease(e->boot_timer);
			e->boot_timer = NULL;
		}
		break;
	}
}

static void boot_anim_start(struct rk3_engine *e)
{
	e->boot_step = 0;
	CFRunLoopTimerContext tctx = { 0, e, NULL, NULL, NULL };
	e->boot_timer = CFRunLoopTimerCreate(NULL,
			CFAbsoluteTimeGetCurrent(),   /* first frame immediately */
			0.09,                          /* ~90 ms per frame */
			0, 0, boot_anim_tick, &tctx);
	if (e->boot_timer)
		CFRunLoopAddTimer(CFRunLoopGetCurrent(), e->boot_timer,
				  kCFRunLoopDefaultMode);
	else
		midi_show_con(e);                 /* no timer: just show Con */
}

/* Bring the relay up on the engine thread: open the socket, arm the EP1 read,
 * then tell the device to auto-report buttons/pedal. Arm BEFORE AUTO_MSG so we
 * never miss the first report. All steps are non-fatal to audio. Called
 * unconditionally once at engine open (always-on MIDI; no opt-in sentinel). */
static void midi_relay_start(struct rk3_engine *e)
{
	midi_relay_open(e);
	e->midi_cb_ctr = 0;
	e->midi_cb_t0 = mach_absolute_time();
	e->midi_send_errs = 0;
	atomic_store_explicit(&e->midi_running, 1, memory_order_release);
	midi_arm_read(e);

	/* snd_usb_caiaq_set_auto_msg: RK3 = (digital=1, analog=10, erp=0). */
	uint8_t am[3] = { 1, 10, 0 };
	if (caiaq_send_command(&e->usb, EP1_CMD_AUTO_MSG, am, sizeof(am)) != 0)
		ELOG("midi relay: AUTO_MSG failed (controls won't auto-report)");

	boot_anim_start(e);               /* sweep + rate flash, settling on "Con" */
}

static void midi_relay_stop(struct rk3_engine *e)
{
	int was_on = atomic_load_explicit(&e->midi_running, memory_order_acquire);
	atomic_store_explicit(&e->midi_running, 0, memory_order_release);
	if (was_on)
		(*e->usb.intf)->AbortPipe(e->usb.intf, e->usb.pipe_cmd_in);
	if (e->midi_fd >= 0) {
		close(e->midi_fd);
		e->midi_fd = -1;
	}
}

/* ---- LED / 7-segment listener: loopback UDP -> EP1_CMD_WRITE_IO ------- */

/* Async WRITE_IO completion: clear the in-flight flag, count / log the result. */
static void led_write_done(void *refcon, IOReturn result, void *arg0)
{
	(void)arg0;
	struct rk3_engine *e = refcon;
	e->led_busy = 0;
	if (result != kIOReturnSuccess) {
		if (e->led_errs < 4) {
			ELOG("led: WRITE_IO async failed 0x%08x (errs=%d)",
			     result, e->led_errs + 1);
			e->led_errs++;
		}
	} else {
		e->led_writes++;
	}
}

/*
 * One datagram arrived on the LED port: it IS a control_state buffer (the RK3
 * protocol always sends the whole buffer). Forward it verbatim as a WRITE_IO on
 * bulk EP 0x01 OUT. Runs on the engine run loop, so it is strictly sequential
 * with the isoc completions and the MIDI read — no locking, no concurrent USB
 * access. recv once per callback; CFSocket re-fires while datagrams remain queued.
 *
 * The write MUST be asynchronous: a synchronous WritePipeTO here blocks this run
 * loop, freezing the isoc servicing and melting the audio stream (observed
 * 2026-06-21 — OUT ring ballooned to full, dev_rate collapsed). WritePipeAsync
 * returns immediately and completes via led_write_done. Best-effort throughout:
 * never disturbs audio. led_wbuf must persist for the duration of the async call.
 */
static void led_recv(CFSocketRef s, CFSocketCallBackType type,
		     CFDataRef address, const void *data, void *info)
{
	(void)type; (void)address; (void)data;
	struct rk3_engine *e = info;
	uint8_t buf[EP1_BUFSIZE];
	ssize_t n = recv(CFSocketGetNative(s), buf, sizeof(buf), 0);
	if (n <= 0)
		return;
	if (e->led_busy)
		return;                       /* a write is still in flight; drop */
	int len = (int)n;
	if (len > EP1_BUFSIZE - 1)
		len = EP1_BUFSIZE - 1;        /* control_state can't exceed the cmd payload */

	/* Always emit a full EP1_BUFSIZE frame (cmd + 63 zero-padded control_state),
	 * matching Linux control.c which sends sizeof(control_state) capped to 63.
	 * The RK3 firmware ignores short frames (a 7-byte write returns success but
	 * lights nothing), so pad the unused tail with zeros. */
	e->led_wbuf[0] = EP1_CMD_WRITE_IO;
	memset(e->led_wbuf + 1, 0, EP1_BUFSIZE - 1);
	memcpy(e->led_wbuf + 1, buf, len);
	e->led_busy = 1;
	IOReturn kr = (*e->usb.intf)->WritePipeAsync(e->usb.intf, e->usb.pipe_cmd_out,
			e->led_wbuf, (UInt32)EP1_BUFSIZE, led_write_done, e);
	if (kr != kIOReturnSuccess) {
		e->led_busy = 0;
		if (e->led_errs < 4) {
			ELOG("led: WritePipeAsync arm failed 0x%08x", kr);
			e->led_errs++;
		}
	}
}

/*
 * Bind the loopback UDP LED port and add it to the engine run loop. Non-fatal:
 * if the coreaudiod sandbox blocks inbound UDP (only outbound is proven), we log
 * and continue with audio + MIDI unaffected — LED control is simply unavailable
 * this session. Whether bind() is permitted is the open question this answers.
 */
static void led_listen_start(struct rk3_engine *e)
{
	CFSocketContext ctx = { 0, e, NULL, NULL, NULL };
	e->led_sock = CFSocketCreate(NULL, AF_INET, SOCK_DGRAM, IPPROTO_UDP,
			kCFSocketReadCallBack, led_recv, &ctx);
	if (!e->led_sock) {
		ELOG("led: CFSocketCreate failed — LED control unavailable this session");
		return;
	}
	int fd = CFSocketGetNative(e->led_sock);
	int yes = 1;
	setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

	struct sockaddr_in a;
	memset(&a, 0, sizeof(a));
	a.sin_family = AF_INET;
	a.sin_port = htons(RK3_LED_UDP_PORT);
	a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);   /* 127.0.0.1 only */
	if (bind(fd, (struct sockaddr *)&a, sizeof(a)) != 0) {
		ELOG("led: bind 127.0.0.1:%d failed errno=%d — LED control unavailable "
		     "(coreaudiod sandbox may block inbound UDP)", RK3_LED_UDP_PORT, errno);
		CFRelease(e->led_sock);     /* closes the native fd (close-on-invalidate) */
		e->led_sock = NULL;
		return;
	}

	e->led_src = CFSocketCreateRunLoopSource(NULL, e->led_sock, 0);
	if (!e->led_src) {
		ELOG("led: CFSocketCreateRunLoopSource failed — LED control unavailable");
		CFRelease(e->led_sock);
		e->led_sock = NULL;
		return;
	}
	CFRunLoopAddSource(CFRunLoopGetCurrent(), e->led_src, kCFRunLoopDefaultMode);
	ELOG("led: listening on 127.0.0.1:%d (control_state -> WRITE_IO)", RK3_LED_UDP_PORT);
}

static void led_listen_stop(struct rk3_engine *e)
{
	if (e->led_src) {
		CFRunLoopRemoveSource(CFRunLoopGetCurrent(), e->led_src,
				      kCFRunLoopDefaultMode);
		CFRelease(e->led_src);
		e->led_src = NULL;
	}
	if (e->led_sock) {
		CFSocketInvalidate(e->led_sock);   /* closes the native fd */
		CFRelease(e->led_sock);
		e->led_sock = NULL;
	}
}

static void *engine_thread(void *arg)
{
	struct rk3_engine *e = arg;
	e->runloop = CFRunLoopGetCurrent();
	engine_set_realtime();

	CFRunLoopSourceRef src = NULL;
	IOReturn kr = (*e->usb.intf)->CreateInterfaceAsyncEventSource(e->usb.intf, &src);
	if (kr != kIOReturnSuccess || !src) {
		ELOG("CreateInterfaceAsyncEventSource 0x%08x", kr);
		e->start_rc = -1;
		atomic_store_explicit(&e->thread_ready, 1, memory_order_release);
		return NULL;
	}
	CFRunLoopAddSource(CFRunLoopGetCurrent(), src, kCFRunLoopDefaultMode);

	/* One-shot timer used by the throttled re-arm: it never fires on its own
	 * (seeded far in the future), midi_read_done bumps its fire date when it has
	 * to defer a re-arm. Created here so it lives on this run loop. */
	CFRunLoopTimerContext tctx = { 0, e, NULL, NULL, NULL };
	e->midi_rearm_timer = CFRunLoopTimerCreate(NULL,
			CFAbsoluteTimeGetCurrent() + 1e9, 1e9, 0, 0,
			midi_rearm_tick, &tctx);
	if (e->midi_rearm_timer)
		CFRunLoopAddTimer(CFRunLoopGetCurrent(), e->midi_rearm_timer,
				kCFRunLoopDefaultMode);

	/* LED / 7-segment control listener. Passive (acts only on inbound datagrams),
	 * no callback-rate risk, so it runs unconditionally. */
	led_listen_start(e);

	atomic_store_explicit(&e->running, 1, memory_order_release);
	e->start_rc = 0;
	atomic_store_explicit(&e->thread_ready, 1, memory_order_release);

	/* Always-on MIDI: arm the foot-controls relay the moment USB is claimed, so
	 * the RK3 is a system MIDI device whether or not audio is streaming. */
	midi_relay_start(e);

	ELOG("engine USB up (rate=%d bpp=%d) — MIDI relay armed (always-on); audio idle",
	     e->rate, e->bpp);
	CFRunLoopRun();   /* until rk3_engine_close() calls CFRunLoopStop */

	/* teardown on the engine thread (audio already stopped by close) */
	led_listen_stop(e);
	midi_relay_stop(e);
	if (e->boot_timer) {              /* boot animation still running at close */
		CFRunLoopTimerInvalidate(e->boot_timer);
		CFRelease(e->boot_timer);
		e->boot_timer = NULL;
	}
	if (e->midi_rearm_timer) {
		CFRunLoopRemoveTimer(CFRunLoopGetCurrent(), e->midi_rearm_timer,
				     kCFRunLoopDefaultMode);
		CFRelease(e->midi_rearm_timer);
		e->midi_rearm_timer = NULL;
	}
	(*e->usb.intf)->AbortPipe(e->usb.intf, e->usb.pipe_iso_in);
	(*e->usb.intf)->AbortPipe(e->usb.intf, e->usb.pipe_iso_out);
	CFRunLoopRemoveSource(CFRunLoopGetCurrent(), src, kCFRunLoopDefaultMode);
	CFRelease(src);
	ELOG("engine stopped (cap_overruns=%u pb_underruns=%u pb_overruns=%u "
	     "in_errs=%d out_errs=%d)",
	     atomic_load(&e->cap_overruns), atomic_load(&e->pb_underruns),
	     atomic_load(&e->pb_overruns), e->in_errs, e->out_errs);
	return NULL;
}

/* Serialize USB work with the engine. A queue timeout cancels the work;
 * an already executing operation is joined before the caller can tear down. */
static bool engine_sync(struct rk3_engine *e, int max_ms, void (^work)(void))
{
    if (!e->runloop || !atomic_load_explicit(&e->running, memory_order_acquire))
        return false;
    bool ok = rk3_runloop_sync(e->runloop, max_ms, work);
    if (!ok)
        ELOG("engine_sync: cancelled queued work after %d ms", max_ms);
    return ok;
}

/* Reset all audio-streaming state (rings, cadence mirror, diagnostics, clock
 * telemetry) for a fresh audio_start. Does NOT touch USB / thread / MIDI. */
static void reset_audio_state(struct rk3_engine *e)
{
	e->pat_head = e->pat_tail = e->out_acc = 0;
	e->pat_stage_off = RK3_FRAMES_PER_XFER;   /* force a pop on first OUT fill */
	e->in_errs = e->out_errs = e->out_drops = 0;

	/*
	 * Capture ring empty. Playback ring empty + ungated: the engine sends
	 * silence on OUT until write_output has buffered RK3_OUT_CUSHION, then it
	 * sets pb_r = RK3_OUT_PHASE and pb_started (preserving the 4-byte phase).
	 */
	atomic_store(&e->cap_w, 0);
	atomic_store(&e->cap_r, 0);
	atomic_store(&e->cap_primed, 0);
	e->cap_settle_left = e->cap_settle_init;
	e->cap_measure_left = e->cap_measure_init;
	e->cap_meas_min = ~0u;
	e->out_fill_min = ~0u;
	e->cap_fill_min = ~0u;
	atomic_store(&e->pb_w, 0);
	atomic_store(&e->pb_r, 0);
	atomic_store(&e->pb_started, 0);
	atomic_store(&e->cap_overruns, 0);
	atomic_store(&e->pb_underruns, 0);
	atomic_store(&e->pb_overruns, 0);

	atomic_store(&e->dev_frames, 0);
	e->dbg_ctr = 0;
	e->dbg_host0 = 0;
	e->dbg_devframes0 = 0;
}

/* ---- public API ------------------------------------------------------ */

/* Recompute the rate-scaled fill constants from e->rate. The reference values are
 * tuned at 48 kHz and describe fixed amounts of time / producer-consumer bite, so
 * they scale linearly with the rate. Called at open and on every set_rate. */
static void engine_apply_rate_scale(struct rk3_engine *e)
{
	e->out_cushion       = RK3_SCALE_TO_RATE(RK3_OUT_CUSHION, e->rate);
	e->out_cushion      -= e->out_cushion % RK3_BYTES_PER_FRAME;  /* keep frame-aligned */
	e->out_fill_setpoint = RK3_SCALE_TO_RATE(RK3_OUT_FILL_SETPOINT, e->rate);
	e->cap_fill_floor    = RK3_SCALE_TO_RATE(RK3_CAP_FILL_FLOOR, e->rate);
	e->cap_settle_init   = RK3_SCALE_TO_RATE(RK3_CAP_SETTLE_FRAMES, e->rate);
	e->cap_measure_init  = RK3_SCALE_TO_RATE(RK3_CAP_MEASURE_FRAMES, e->rate);
}

/* Rates the caiaq protocol + this engine accept (caiaq_set_audio_params codes). */
static int rate_supported(int rate)
{
	return rate == 44100 || rate == 48000 || rate == 88200 ||
	       rate == 96000 || rate == 192000;
}

int rk3_engine_set_rate(struct rk3_engine *e, int rate)
{
	if (!rate_supported(rate)) {
		ELOG("set_rate: unsupported rate %d", rate);
		return -1;
	}
	if (atomic_load_explicit(&e->audio_running, memory_order_acquire)) {
		ELOG("set_rate: refused while audio running");
		return -1;
	}
	e->rate = rate;
	e->bpp = caiaq_compute_bpp(rate, e->data_alignment, e->n_streams);
	engine_apply_rate_scale(e);
	ELOG("set_rate: rate=%d bpp=%d (takes effect at next audio_start)", e->rate, e->bpp);
	return 0;
}

int rk3_engine_out_fill_setpoint(const struct rk3_engine *e)
{
	return e->out_fill_setpoint;
}

int rk3_engine_open(struct rk3_engine *e, int rate)
{
	struct caiaq_device_spec spec;

	atomic_store(&e->running, 0);
	atomic_store(&e->audio_running, 0);
	atomic_store(&e->usb_dead, 0);
	atomic_store(&e->thread_ready, 0);
	e->runloop = NULL;
	e->rate = rate;
	e->midi_fd = -1;
	e->midi_send_errs = 0;
	e->midi_cb_ctr = 0;
	e->midi_cb_t0 = 0;
	e->midi_storm_tripped = 0;
	atomic_store(&e->midi_running, 0);
	e->led_sock = NULL;
	e->led_src = NULL;
	e->led_writes = 0;
	e->led_errs = 0;
	e->led_busy = 0;
	e->boot_timer = NULL;
	e->boot_step = 0;

	if (rk3_usb_open(&e->usb) != 0)
		return -1;

	/* Handshake: identifies the device and yields data_alignment for bpp.
	 * AUDIO_PARAMS is deferred to audio_start (no audio format set while idle,
	 * mirroring the proven standalone MIDI path). */
	if (caiaq_get_device_info(&e->usb, &spec) != 0) {
		rk3_usb_close(&e->usb);
		return -1;
	}
	int n_streams = spec.num_analog_audio_in / 2;
	if (n_streams < 1) n_streams = 1;
	e->data_alignment = spec.data_alignment;
	e->n_streams = n_streams;
	e->bpp = caiaq_compute_bpp(rate, spec.data_alignment, n_streams);
	engine_apply_rate_scale(e);

	if (pthread_create(&e->thread, NULL, engine_thread, e) != 0) {
		rk3_usb_close(&e->usb);
		return -1;
	}

	/* wait for the thread to come up (or fail to create its event source) */
	while (!atomic_load_explicit(&e->thread_ready, memory_order_acquire))
		usleep(1000);

	if (e->start_rc != 0) {
		rk3_engine_close(e);
		return -1;
	}
	return 0;
}

int rk3_engine_audio_start(struct rk3_engine *e)
{
	if (!atomic_load_explicit(&e->running, memory_order_acquire)) {
		ELOG("audio_start: engine not open");
		return -1;
	}
	if (atomic_load_explicit(&e->audio_running, memory_order_acquire))
		return 0;                         /* already streaming */

	/* Rings/diagnostics are reset here (no consumer yet: StartIO hasn't returned
	 * so no DoIOOperation runs). The USB command I/O (AUDIO_PARAMS, which reads a
	 * reply on EP 0x81) and the isoc prime are done on the engine thread so they
	 * never run concurrently with the engine's own USB traffic. */
	reset_audio_state(e);
	atomic_store_explicit(&e->audio_running, 1, memory_order_release);

	__block int params_rc = -1;
	bool ok = engine_sync(e, 9000, ^{
		/*
		 * Quiesce the MIDI relay's async EP 0x81 read first: AUDIO_PARAMS reads
		 * its ack on the SAME bulk-in pipe, and two readers on one pipe conflict.
		 * We're on the engine thread, so this is strictly sequential; the aborted
		 * read's stale completion is harmless (its midi_running guard returns it).
		 */
		int relay_was_on = atomic_load_explicit(&e->midi_running, memory_order_acquire);
		if (relay_was_on) {
			atomic_store_explicit(&e->midi_running, 0, memory_order_release);
			(*e->usb.intf)->AbortPipe(e->usb.intf, e->usb.pipe_cmd_in);
		}

		/*
		 * AUDIO_PARAMS reads its ack on EP 0x81 (pipe_cmd_in) — the SAME pipe the
		 * MIDI relay's async read was just aborted on. AbortPipe cancels the
		 * in-flight transfer but does NOT resync the endpoint data toggle, so the
		 * device's ack can arrive on a toggle the host no longer expects, the read
		 * times out, and params_rc comes back -1. That's the recurring
		 * "audio_start: failed (params_rc=-1)" that previously needed a reboot
		 * (device_info never hits it — it runs before the relay is ever armed).
		 *
		 * Recover deterministically: ClearPipeStallBothEnds resyncs host+device
		 * toggles for the pipe, and we retry a few times before declaring failure.
		 */
		for (int attempt = 0; attempt < 3; attempt++) {
			(*e->usb.intf)->ClearPipeStallBothEnds(e->usb.intf, e->usb.pipe_cmd_in);
			params_rc = caiaq_set_audio_params(&e->usb, e->rate, 24, e->bpp);
			if (params_rc == 0)
				break;
			ELOG("audio_start: AUDIO_PARAMS attempt %d failed; retrying after pipe resync",
			     attempt + 1);
		}
		if (params_rc == 0) {
			caiaq_codec_reset(&e->codec);
			if (prime_chains(e) != 0)
				params_rc = -1;   /* propagate to the failure check below */
		}

		/*
		 * NOTE: hardware direct monitoring (EP1 0x0e) was investigated here and
		 * confirmed INERT on the RK3 — the firmware ACKs the command but there is
		 * no analog/DSP monitor mixer behind it (the RK3 has a single stereo out,
		 * no Monitor channel pair, unlike NI's larger interfaces). Monitoring on
		 * this device must go through the host. See caiaq_set_direct_mon() (kept
		 * as documentation) and the project memory for the full negative result.
		 */

		/* Re-arm MIDI so the controls keep working alongside audio (or, on a
		 * params failure, while idle — MIDI is independent of audio). */
		if (relay_was_on) {
			atomic_store_explicit(&e->midi_running, 1, memory_order_release);
			midi_arm_read(e);
		}
	});

	if (!ok || params_rc != 0) {
		atomic_store_explicit(&e->audio_running, 0, memory_order_release);
		ELOG("audio_start: failed (sync_ok=%d params_rc=%d)", ok, params_rc);
		return -1;
	}
	ELOG("audio_start: isoc chains primed (rate=%d bpp=%d)", e->rate, e->bpp);
	return 0;
}

void rk3_engine_audio_stop(struct rk3_engine *e)
{
	if (!atomic_load_explicit(&e->running, memory_order_acquire))
		return;
	if (!atomic_load_explicit(&e->audio_running, memory_order_acquire))
		return;
	/* stop resubmits first, then abort in-flight transfers on the engine thread */
	atomic_store_explicit(&e->audio_running, 0, memory_order_release);
	engine_sync(e, 1000, ^{
		(*e->usb.intf)->AbortPipe(e->usb.intf, e->usb.pipe_iso_in);
		(*e->usb.intf)->AbortPipe(e->usb.intf, e->usb.pipe_iso_out);
#if RK3_USE_LOWLAT
		/* free the kernel-shared buffers now the chains are aborted (on the
		 * engine thread, so no completion can still touch them). */
		ll_free_all(e);
#endif
	});
	ELOG("audio_stop: isoc chains aborted (USB + MIDI stay up)");
}

void rk3_engine_close(struct rk3_engine *e)
{
	rk3_engine_audio_stop(e);            /* no-op if not streaming */
	if (!atomic_load(&e->running) && !e->runloop)
		return;
	atomic_store_explicit(&e->running, 0, memory_order_release);
	if (e->runloop) {
		CFRunLoopStop(e->runloop);
		pthread_join(e->thread, NULL);
		e->runloop = NULL;
	}
	rk3_usb_close(&e->usb);
}

int rk3_engine_usb_dead(struct rk3_engine *e)
{
	return atomic_load_explicit(&e->usb_dead, memory_order_acquire);
}

/* ---- composites (preserve the original one-shot API for the tools) --- */

int rk3_engine_start(struct rk3_engine *e, int rate)
{
	int rc = rk3_engine_open(e, rate);
	if (rc != 0)
		return rc;
	rc = rk3_engine_audio_start(e);
	if (rc != 0) {
		rk3_engine_close(e);
		return rc;
	}
	return 0;
}

void rk3_engine_stop(struct rk3_engine *e)
{
	rk3_engine_audio_stop(e);
	rk3_engine_close(e);
}

/* ---- Float32 <-> S24_3BE bridges (CoreAudio IO thread) --------------- */

static inline float s24be_to_f32(const uint8_t *p)
{
	int32_t v = (int32_t)((uint32_t)p[0] << 24 |
			      (uint32_t)p[1] << 16 |
			      (uint32_t)p[2] << 8);   /* sign in MSB */
	return (float)(v >> 8) / 8388608.0f;
}
static inline void f32_to_s24be(float f, uint8_t *p)
{
	if (f > 1.0f) f = 1.0f;
	if (f < -1.0f) f = -1.0f;
	int32_t v = (int32_t)(f * 8388607.0f);
	p[0] = (uint8_t)(v >> 16);
	p[1] = (uint8_t)(v >> 8);
	p[2] = (uint8_t)v;
}

void rk3_engine_read_input(struct rk3_engine *e, float *dst, int nframes)
{
	int need = nframes * RK3_BYTES_PER_FRAME;
	unsigned r = atomic_load_explicit(&e->cap_r, memory_order_relaxed);
	unsigned w = atomic_load_explicit(&e->cap_w, memory_order_acquire);

	/*
	 * One-time startup capture drain. The capture standing fill is an uncontrolled
	 * startup offset (~490 frames ≈ 10 ms of hidden input latency) that the device's
	 * initial clock transient builds up over the first ~1 s — so we cannot drain at
	 * pb_started (too early, fill not yet established). Wait RK3_CAP_SETTLE_FRAMES
	 * past pb_started, then collapse the fill to RK3_CAP_FILL_SETPOINT once. The
	 * dropped amount is a frame multiple, so cap_r stays frame-aligned. Runs exactly
	 * once (cap_primed). read_input is the sole cap_r writer, so this is SPSC-safe.
	 */
	if (!atomic_load_explicit(&e->cap_primed, memory_order_relaxed) &&
	    atomic_load_explicit(&e->pb_started, memory_order_acquire)) {
		if (e->cap_settle_left > 0) {
			e->cap_settle_left -= nframes;   /* wait out the IN startup transient */
		} else if (e->cap_measure_left > 0) {
			unsigned hf = ring_avail(w, r) / RK3_BYTES_PER_FRAME;
			if (hf < e->cap_meas_min) e->cap_meas_min = hf;  /* find the trough */
			e->cap_measure_left -= nframes;
		} else {
			if (e->cap_meas_min != ~0u &&
			    e->cap_meas_min > (unsigned)e->cap_fill_floor) {
				unsigned drop = (e->cap_meas_min - e->cap_fill_floor)
						* RK3_BYTES_PER_FRAME;
				r = (r + drop) % RK3_RING_BYTES;   /* trough -> floor */
			}
			atomic_store_explicit(&e->cap_primed, 1, memory_order_relaxed);
		}
	}

	unsigned avail = ring_avail(w, r);
	{                                   /* windowed-min cap_fill telemetry */
		unsigned af = avail / RK3_BYTES_PER_FRAME;
		if (af < e->cap_fill_min) e->cap_fill_min = af;
	}
	int got = (avail < (unsigned)need) ? (int)avail : need;
	got -= got % RK3_BYTES_PER_FRAME;

	int i = 0;
	for (; i < got / RK3_BYTES_PER_FRAME; i++) {
		/* The RK3 streams its two capture channels in the opposite order to
		 * its silkscreened input jacks, so swap them here: device sample 0 ->
		 * Reaper input 2, device sample 1 -> Reaper input 1. */
		float c0 = s24be_to_f32(&e->cap_buf[r]); r = (r + 3) % RK3_RING_BYTES;
		float c1 = s24be_to_f32(&e->cap_buf[r]); r = (r + 3) % RK3_RING_BYTES;
		dst[i * 2 + 0] = c1;
		dst[i * 2 + 1] = c0;
	}
	atomic_store_explicit(&e->cap_r, r, memory_order_release);
	for (; i < nframes; i++) {          /* zero-fill underrun */
		dst[i * 2 + 0] = 0.0f;
		dst[i * 2 + 1] = 0.0f;
	}
}

void rk3_engine_write_output(struct rk3_engine *e, const float *src, int nframes)
{
	int want = nframes * RK3_BYTES_PER_FRAME;
	unsigned w = atomic_load_explicit(&e->pb_w, memory_order_relaxed);
	unsigned r = atomic_load_explicit(&e->pb_r, memory_order_acquire);
	if (ring_space(w, r) < (unsigned)want) {
		atomic_fetch_add_explicit(&e->pb_overruns, 1, memory_order_relaxed);
		return;                      /* ring full: drop this block */
	}
	for (int i = 0; i < nframes; i++) {
		f32_to_s24be(src[i * 2 + 0], &e->pb_buf[w]); w = (w + 3) % RK3_RING_BYTES;
		f32_to_s24be(src[i * 2 + 1], &e->pb_buf[w]); w = (w + 3) % RK3_RING_BYTES;
	}
	atomic_store_explicit(&e->pb_w, w, memory_order_release);

	/*
	 * Startup gate: once the cushion is buffered, hold the read pointer
	 * RK3_OUT_PHASE bytes in (the proven 4-byte OUT rotation) and release the
	 * engine to start playout. Set pb_r before pb_started (release) so fill_out,
	 * which reads pb_started with acquire, sees the seeded pb_r.
	 */
	if (!atomic_load_explicit(&e->pb_started, memory_order_relaxed) &&
	    w >= (unsigned)e->out_cushion) {
		atomic_store_explicit(&e->pb_r, RK3_OUT_PHASE, memory_order_relaxed);
		atomic_store_explicit(&e->pb_started, 1, memory_order_release);
	}
}

int rk3_engine_out_fill_frames(struct rk3_engine *e)
{
	if (!atomic_load_explicit(&e->pb_started, memory_order_acquire))
		return -1;
	unsigned w = atomic_load_explicit(&e->pb_w, memory_order_acquire);
	unsigned r = atomic_load_explicit(&e->pb_r, memory_order_relaxed);
	return (int)(ring_avail(w, r) / RK3_BYTES_PER_FRAME);
}
