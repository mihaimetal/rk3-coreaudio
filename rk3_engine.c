/* rk3_engine.c — see rk3_engine.h */

#include "rk3_engine.h"
#include "rk3_midi.h"          /* RK3_MIDI_UDP_PORT */

#include <os/log.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
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
	for (int f = 0; f < e->in_fpx; f++)
		slot[f] = x->frames[f].frActCount;
	e->pat_head++;
}

/* Synthesize the device's bursty 88/0 cadence for one transfer (startup). */
static void pat_synth(struct rk3_engine *e, int *lens)
{
	int pkt_frames = e->bpp / 8;
	int need = e->rate / 8000;
	int rem  = e->rate % 8000;
	for (int f = 0; f < e->in_fpx; f++) {
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
		for (int f = 0; f < e->in_fpx; f++)
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
	if (e->pat_stage_off >= e->in_fpx) {
		pat_pop(e, e->pat_stage);
		e->pat_stage_off = 0;
	}
	for (int f = 0; f < e->out_fpx; f++)
		lens[f] = e->pat_stage[e->pat_stage_off + f];
	e->pat_stage_off += e->out_fpx;
}

/* ---- isoc completion callbacks (ported from stream.c) ---------------- */

static int  submit_in(struct rk3_iso_xfer *x);
static int  submit_out(struct rk3_iso_xfer *x);
static void fill_out(struct rk3_engine *e, struct rk3_iso_xfer *x);
static void out_done(void *refcon, IOReturn result, void *arg0);
static bool adapt_warm(struct rk3_engine *e, UInt64 now);
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

/* Allocate all isoc buffers for a stream start; frees partial on failure. Sized
 * for the largest runtime geometry (RK3_FRAMES_PER_XFER microframes), so any
 * tuning fits without reallocating. */
static int ll_alloc_all(struct rk3_engine *e)
{
	for (int i = 0; i < RK3_N_XFERS; i++)
		if (ll_alloc_xfer(e, &e->in[i], RK3_XFER_BUF,
				  RK3_FRAMES_PER_XFER, 1) != 0)
			return -1;
	for (int i = 0; i < RK3_MAX_OUT_XFERS; i++)
		if (ll_alloc_xfer(e, &e->out[i], RK3_XFER_BUF,
				  RK3_FRAMES_PER_XFER, 0) != 0)
			return -1;
	return 0;
}

static void ll_free_all(struct rk3_engine *e)
{
	for (int i = 0; i < RK3_N_XFERS; i++)
		ll_free_xfer(e, &e->in[i]);
	for (int i = 0; i < RK3_MAX_OUT_XFERS; i++)
		ll_free_xfer(e, &e->out[i]);
}
#endif /* RK3_USE_LOWLAT */

#define SCHED_MARGIN     4   /* IN: ms-frames ahead of the bus on a (re)sync */
#define OUT_SCHED_MARGIN 2   /* OUT: ms-frames of lead on a resync (1 gets rejected ~1 in 4) */

static UInt64 bus_frame(struct rk3_engine *e)
{
	UInt64 cur = 0;
	AbsoluteTime at;
	(*e->usb.intf)->GetBusFrameNumber(e->usb.intf, &cur, &at);
	return cur;
}

static double ticks_to_ms(UInt64 dt)
{
	static mach_timebase_info_data_t tb;
	if (tb.denom == 0) mach_timebase_info(&tb);
	return (double)dt * tb.numer / tb.denom / 1e6;
}

/* ---- circuit breaker (see RK3_BRK_* in rk3_engine.h) ------------------ */

static void brk_trip(struct rk3_engine *e, const char *why, IOReturn kr)
{
	int ok = 0;
	atomic_store_explicit(&e->audio_running, 0, memory_order_release);
	if (!atomic_compare_exchange_strong(&e->fault, &ok, 1))
		return;                       /* already tripped */
	e->fault_time = mach_absolute_time();
	ELOG("BREAKER TRIPPED: %{public}s (0x%08x) — isoc resubmission stopped "
	     "(in_fpx=%d out_fpx=%d n_out=%d in_errs=%d out_errs=%d restarts=%d)",
	     why, kr, e->in_fpx, e->out_fpx, e->n_out, e->in_errs, e->out_errs,
	     e->restarts);
}

static void res_note(struct rk3_res *tab, IOReturn code)
{
	for (int i = 0; i < RK3_BRK_RESULTS; i++) {
		if (tab[i].n == 0) tab[i].code = code;
		if (tab[i].code == code) { tab[i].n++; return; }
	}
}

static void res_format(const struct rk3_res *tab, char *dst, size_t cap)
{
	size_t off = 0;
	dst[0] = 0;
	for (int i = 0; i < RK3_BRK_RESULTS && tab[i].n && off < cap; i++)
		off += (size_t)snprintf(dst + off, cap - off, "%s%x:%u",
					i ? "," : "", tab[i].code, tab[i].n);
}

/* Called first thing in every isoc completion. Returns false when the chain
 * must NOT resubmit (audio stopped, or this completion tripped the breaker). */
static bool brk_completion(struct rk3_engine *e, struct rk3_res *tab, IOReturn result)
{
	if (!atomic_load_explicit(&e->audio_running, memory_order_acquire))
		return false;
	res_note(tab, result);

	UInt64 now = mach_absolute_time();
	if (e->brk_t0 == 0) {
		e->brk_t0 = now;
		e->brk_cb = 0;
	}
	if (++e->brk_cb >= RK3_BRK_WINDOW) {
		double ms = ticks_to_ms(now - e->brk_t0);
		double nominal_ms = RK3_BRK_WINDOW * 1000.0 /
			(8000.0 / e->in_fpx + 8000.0 / e->out_fpx);
		e->brk_cb = 0;
		e->brk_t0 = now;
		if (ms < nominal_ms / RK3_BRK_RATIO) {
			ELOG("breaker: %d completions in %.2f ms (nominal %.1f ms)",
			     RK3_BRK_WINDOW, ms, nominal_ms);
			brk_trip(e, "completion rate", result);
			return false;
		}
	}
	return true;
}

/* ---- isoc submit ------------------------------------------------------ */

/*
 * Queue one capture transfer at the IN cursor. The submit is optimistic: in
 * steady state the cursor is always ahead of the bus, so the bus frame number is
 * NOT read first (GetBusFrameNumber is a kernel round trip — ~27 us, and at 2000
 * transfers/s it was 10% of this thread's CPU). Only when the host rejects the
 * transfer (the cursor fell behind after a late callback) is the cursor pulled
 * forward to the bus frame, further ahead on each retry, so the chain resyncs as
 * a whole and transfers never overlap. If it still fails the chain would lose a
 * member for good, so trip the breaker.
 */
static int submit_in(struct rk3_iso_xfer *x)
{
	struct rk3_engine *e = x->e;
	IOReturn kr = kIOReturnSuccess;
	for (int tries = 0; tries < 4; tries++) {
		if (tries > 0) {
			e->in_next = bus_frame(e) + ((UInt64)SCHED_MARGIN << (tries - 1));
			e->in_resyncs++;
		}
		x->frame_start = e->in_next;
		for (int f = 0; f < e->in_fpx; f++) {
			x->frames[f].frReqCount = RK3_FRAME_BYTES;
			x->frames[f].frActCount = 0;
			x->frames[f].frStatus = 0;
		}
		kr = rk3_read_isoc(e, x, (UInt32)e->in_fpx);
		if (kr == kIOReturnSuccess) {
			e->in_next += (UInt64)(e->in_fpx / 8);
			return 0;
		}
		e->in_errs++;
	}
	brk_trip(e, "IN submit failed", kr);
	return -1;
}

/* Same for playback. The data was already packed by fill_out; a resync only
 * moves where on the bus it lands (a gap), never its order. */
static int submit_out(struct rk3_iso_xfer *x)
{
	struct rk3_engine *e = x->e;
	IOReturn kr = kIOReturnSuccess;
	for (int tries = 0; tries < 5; tries++) {
		if (tries > 0) {
			e->out_next = bus_frame(e) + ((UInt64)OUT_SCHED_MARGIN << (tries - 1));
			e->out_resyncs++;
		}
		x->frame_start = e->out_next;
		kr = rk3_write_isoc(e, x, (UInt32)e->out_fpx);
		if (kr == kIOReturnSuccess) {
			e->out_next += (UInt64)(e->out_fpx / 8);
			return 0;
		}
		e->out_errs++;
	}
	e->out_drops++;
	brk_trip(e, "OUT submit failed", kr);
	return -1;
}

/* ---- isoc completion callbacks ---------------------------------------- */

static void in_done(void *refcon, IOReturn result, void *arg0)
{
	struct rk3_iso_xfer *x = refcon;
	struct rk3_engine *e = x->e;
	(void)arg0;

	if (!brk_completion(e, e->in_res, result))
		return;

	/* Deinterleave each microframe into a scratch then push to the cap ring. */
	uint8_t scratch[RK3_XFER_BUF];
	int pos = 0;
	for (int f = 0; f < e->in_fpx; f++) {
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

	submit_in(x);
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
	const int nf = e->out_fpx;
	int lens[RK3_FRAMES_PER_XFER];
	pat_pop_out(e, lens);

	if (lens[nf - 1] == 0) {
		int j = nf - 2;
		while (j >= 0 && lens[j] == 0)
			j--;
		if (j >= 0) {
			lens[nf - 1] = lens[j];
			lens[j] = 0;
		}
	}

	int started = atomic_load_explicit(&e->pb_started, memory_order_acquire);
	unsigned r = atomic_load_explicit(&e->pb_r, memory_order_relaxed);
	unsigned w = atomic_load_explicit(&e->pb_w, memory_order_acquire);
	unsigned avail = started ? ring_avail(w, r) : 0;

	int off = 0;
	for (int f = 0; f < nf; f++) {
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
		unsigned tr = atomic_load_explicit(&e->out_trough, memory_order_relaxed);
		while (of < tr && !atomic_compare_exchange_weak_explicit(&e->out_trough,
				&tr, of, memory_order_relaxed, memory_order_relaxed))
			;
	}
}

static void out_done(void *refcon, IOReturn result, void *arg0)
{
	struct rk3_iso_xfer *x = refcon;
	struct rk3_engine *e = x->e;
	(void)arg0;
	if (!brk_completion(e, e->out_res, result))
		return;

	/* Adaptive pipeline depth (RK3_ADAPT_*): retire the last slot when asked. */
	if (e->out_shrink && x->index == e->n_out - 1) {
		e->out_shrink = 0;
		e->n_out--;
		return;
	}

	fill_out(e, x);
	if (submit_out(x) != 0)
		return;

	UInt64 now = mach_absolute_time();
	if (e->out_errs != e->out_errs_seen) {
		/* A submit was rejected: this callback ran later than the pipeline's
		 * lead. Add one transfer in flight so the next late one still lands. */
		e->out_errs_seen = e->out_errs;
		e->out_err_time = now;
		e->out_shrink = 0;
		if (adapt_warm(e, now) && e->n_out < RK3_MAX_OUT_XFERS &&
		    e->n_out < e->n_out_base + RK3_ADAPT_MAX_GROW &&
		    ticks_to_ms(now - e->out_grow_time) >= RK3_ADAPT_HOLD_S * 1000.0) {
			struct rk3_iso_xfer *y = &e->out[e->n_out];
			y->e = e;
			y->index = e->n_out;
			e->n_out++;
			e->out_grow_time = now;
			fill_out(e, y);
			if (submit_out(y) != 0)
				return;
			e->out_errs_seen = e->out_errs;
		}
	} else if (e->n_out > e->n_out_base && !e->out_shrink &&
		   ticks_to_ms(now - e->out_err_time) >= RK3_ADAPT_SHRINK_S * 1000.0) {
		e->out_shrink = 1;
		e->out_err_time = now;        /* one step per quiet period */
	}

	/*
	 * Drift-vs-jitter probe, about once a second (out_done fires 8000/out_fpx
	 * times per second regardless of n_out). Logs the OUT ring fill, the CAPTURE
	 * ring fill (so we catch a silent input-latency balloon — the IN ring growing
	 * if CoreAudio drains it slower than the device fills it), the CUMULATIVE
	 * measured device rate from a FIXED anchor (converges as the run lengthens),
	 * and the transport health counters. Monotonic out_fill decline => drift;
	 * noisy around a stable mean with underruns on dips => jitter/margin.
	 */
	if (e->dbg_host0 == 0) {
		e->dbg_host0 = mach_absolute_time();
		e->dbg_devframes0 = atomic_load_explicit(&e->dev_frames, memory_order_relaxed);
	}
	if (++e->dbg_ctr >= (unsigned)(8000 / e->out_fpx)) {
		e->dbg_ctr = 0;
		double elapsed_s = ticks_to_ms(mach_absolute_time() - e->dbg_host0) / 1e3;
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
		char ires[96], ores[96];
		res_format(e->in_res, ires, sizeof(ires));
		res_format(e->out_res, ores, sizeof(ores));
		ELOG("clk: out_fill=%u (min %u) cap_fill=%u (margin min %u)  dev_rate=%.2f Hz (nominal %d)  underruns=%u cap_under=%u"
		     "  xfer=%d/%dx%d errs=%d/%d resync=%u/%u drains=%u margin=+%d/+%d chk_errs=%u out_panic=%d res in[%{public}s] out[%{public}s]",
		     fill, ofmin == ~0u ? 0 : ofmin, capfill, cfmin == ~0u ? 0 : cfmin,
		     rate, e->rate, atomic_load(&e->pb_underruns),
		     atomic_load(&e->cap_underruns),
		     e->in_fpx, e->out_fpx, e->n_out, e->in_errs, e->out_errs,
		     e->in_resyncs, e->out_resyncs, e->cap_drains,
		     atomic_load(&e->out_extra), atomic_load(&e->cap_extra),
		     e->codec.input_errs, e->codec.output_panic, ires, ores);
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

	e->in_next = e->out_next = bus_frame(e) + 10;

	for (int i = 0; i < RK3_N_XFERS; i++) {
		e->in[i].e = e;
		e->in[i].index = i;
		if (submit_in(&e->in[i]) != 0)
			ELOG("prime IN %d failed", i);
	}

	for (int i = 0; i < e->n_out; i++) {
		e->out[i].e = e;
		e->out[i].index = i;
		fill_out(e, &e->out[i]);
		if (submit_out(&e->out[i]) != 0)
			ELOG("prime OUT %d failed", i);
	}
	return 0;
}

/*
 * Breaker recovery, on the engine run loop at 2 Hz. A trip leaves in-flight
 * transfers to complete without resubmitting; the first tick after it aborts
 * both pipes, and a later tick (>= 1 s after the trip, so every stale completion
 * has drained) re-primes the chains. Bounded by RK3_BRK_MAX_RESTARTS per
 * audio_start, so a persistent fault ends in silence, never in a retry loop.
 */
static void watchdog_tick(CFRunLoopTimerRef t, void *info)
{
	(void)t;
	struct rk3_engine *e = info;
	int f = atomic_load_explicit(&e->fault, memory_order_acquire);
	if (!f || !atomic_load_explicit(&e->audio_wanted, memory_order_acquire))
		return;
	if (f == 1) {
		(*e->usb.intf)->AbortPipe(e->usb.intf, e->usb.pipe_iso_in);
		(*e->usb.intf)->AbortPipe(e->usb.intf, e->usb.pipe_iso_out);
		atomic_store_explicit(&e->fault, 2, memory_order_release);
		return;
	}
	if (ticks_to_ms(mach_absolute_time() - e->fault_time) < 1000.0)
		return;
	if (e->restarts >= RK3_BRK_MAX_RESTARTS) {
		if (e->restarts == RK3_BRK_MAX_RESTARTS) {
			e->restarts++;
			ELOG("breaker: %d restarts used — audio stays down until the "
			     "next StartIO", RK3_BRK_MAX_RESTARTS);
		}
		return;
	}
	e->restarts++;

	e->pat_head = e->pat_tail = e->out_acc = 0;
	e->pat_stage_off = e->in_fpx;
	e->brk_cb = 0;
	e->brk_t0 = 0;
	caiaq_codec_reset(&e->codec);

	/* CoreAudio kept producing while the consumer was down; drop that backlog
	 * (whole frames, so the OUT phase is preserved) instead of playing it late. */
	if (atomic_load_explicit(&e->pb_started, memory_order_acquire)) {
		unsigned w = atomic_load_explicit(&e->pb_w, memory_order_acquire);
		unsigned r = atomic_load_explicit(&e->pb_r, memory_order_relaxed);
		unsigned fill = ring_avail(w, r) / RK3_BYTES_PER_FRAME;
		if (fill > (unsigned)e->out_fill_setpoint) {
			r = (r + (fill - e->out_fill_setpoint) * RK3_BYTES_PER_FRAME)
			    % RK3_RING_BYTES;
			atomic_store_explicit(&e->pb_r, r, memory_order_release);
		}
	}

	ELOG("breaker: re-priming isoc chains (restart %d/%d)",
	     e->restarts, RK3_BRK_MAX_RESTARTS);
	atomic_store_explicit(&e->fault, 0, memory_order_release);
	atomic_store_explicit(&e->audio_running, 1, memory_order_release);
	/* audio_stop may have raced the two stores above; it wins. */
	if (!atomic_load_explicit(&e->audio_wanted, memory_order_acquire)) {
		atomic_store_explicit(&e->audio_running, 0, memory_order_release);
		return;
	}
	prime_chains(e);
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
	/*
	 * kIOReturnAborted with midi_running still set is NOT our teardown (every
	 * abort of ours clears midi_running first, caught above): the system
	 * aborted the read — sleep/wake, or the device being unplugged, which
	 * reports Aborted rather than NoDevice here. Returning now, as this used
	 * to, left the relay dead and the engine holding a stale handle for good:
	 * no MIDI after a wake, and a re-plugged RK3 was never claimed again. So
	 * fall through to the throttled re-arm; if the device is really gone the
	 * plugin supervisor sees it via rk3_engine_present and recycles the engine.
	 */
	if (result == kIOReturnAborted && !rk3_usb_present(&e->usb)) {
		ELOG("midi relay: read aborted and the device is detached — USB dead");
		atomic_store_explicit(&e->usb_dead, 1, memory_order_release);
		atomic_store_explicit(&e->midi_running, 0, memory_order_release);
		return;
	}

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

/* Show n (0..999) on digits 2-4 with leading zeros; digit 1 stays dark. */
static void boot_set_number(struct rk3_engine *e, int n)
{
	memset(e->led_wbuf + 1, 0, EP1_BUFSIZE - 1);
	e->led_wbuf[2] = kSegDigit[(n / 100) % 10];
	e->led_wbuf[3] = kSegDigit[(n / 10) % 10];
	e->led_wbuf[4] = kSegDigit[n % 10];
}

/*
 * Power-on display sequence, stepped once per boot_timer tick (RK3_BOOT_TICK_S)
 * on the engine run loop:
 *   1. "rig kontrol 3" scrolls right-to-left across the four digits while a
 *      single LED bounces along the eight round LEDs and the pedal LED blinks;
 *   2. a counter races from 000 to 999, accelerating, with the round LEDs as a
 *      bar graph of its progress;
 *   3. "999" flashes with all LEDs, then a segment spinner runs around all four
 *      digits while the LEDs sparkle;
 *   4. it settles on "Con" with the LEDs off and the timer self-invalidates.
 * Purely cosmetic and best-effort (same drop-if-busy WRITE_IO path as the LED
 * socket), so it never touches the audio path; audio_start cuts it short
 * (boot_anim_cancel) because AUDIO_PARAMS needs the command pipe to itself.
 *
 * led_wbuf layout after the command byte: [1..4] = digits 1..4 (bit0..6 =
 * segments a..g, bit7 = decimal point), [5] = round LEDs 1..8, [6] = pedal LED.
 */
#define RK3_BOOT_TICK_S 0.03

/* 7-segment glyphs, all lowercase: r i g _ k o n t r o l _ 3, padded with four
 * blanks each side so the text enters from the right edge and leaves by the
 * left. (A 7-segment digit has no real capital R — the usual one is identical
 * to A — so the whole name is set in lowercase for consistency.) */
static const uint8_t kBootText[] = {
	0x00, 0x00, 0x00, 0x00,
	0x50, 0x04, 0x6F, 0x00,                         /* r i g   */
	0x75, 0x5C, 0x54, 0x78, 0x50, 0x5C, 0x30, 0x00, /* k o n t r o l */
	0x4F,                                           /* 3 */
	0x00, 0x00, 0x00, 0x00,
};
#define RK3_BOOT_SCROLL_DIV 4                              /* ticks per scroll step */
#define RK3_BOOT_SCROLL ((int)sizeof(kBootText) - 4 + 1)   /* window positions */
#define RK3_BOOT_COUNT  40                                 /* ticks 000 -> 999 */
#define RK3_BOOT_FLASH  12                                 /* ticks: 999 blinks */
#define RK3_BOOT_SPIN   24                                 /* ticks: spinner */

static void boot_anim_stop_timer(struct rk3_engine *e)
{
	if (e->boot_timer) {
		CFRunLoopTimerInvalidate(e->boot_timer);
		CFRelease(e->boot_timer);
		e->boot_timer = NULL;
	}
}

static void boot_anim_tick(CFRunLoopTimerRef t, void *info)
{
	(void)t;
	struct rk3_engine *e = info;
	int s = e->boot_step++;

	if (s < RK3_BOOT_SCROLL * RK3_BOOT_SCROLL_DIV) {   /* 1. scrolling title */
		if (s % RK3_BOOT_SCROLL_DIV)
			return;
		s /= RK3_BOOT_SCROLL_DIV;
		memset(e->led_wbuf + 1, 0, EP1_BUFSIZE - 1);
		memcpy(e->led_wbuf + 1, kBootText + s, 4);
		int pos = s % 14;                          /* 0..7..0 bounce */
		e->led_wbuf[5] = (uint8_t)(1u << (pos < 8 ? pos : 14 - pos));
		e->led_wbuf[6] = (uint8_t)(s & 1);
		led_write_frame(e);
		return;
	}
	s -= RK3_BOOT_SCROLL * RK3_BOOT_SCROLL_DIV;
	if (s < RK3_BOOT_COUNT) {                          /* 2. 000 -> 999, accelerating */
		int n = 999 * (s + 1) * (s + 1) / (RK3_BOOT_COUNT * RK3_BOOT_COUNT);
		boot_set_number(e, n);
		e->led_wbuf[5] = (uint8_t)((1u << (n * 8 / 999)) - 1);   /* bar graph */
		e->led_wbuf[6] = (uint8_t)(s & 1);
		led_write_frame(e);
		return;
	}
	s -= RK3_BOOT_COUNT;
	if (s < RK3_BOOT_FLASH) {                          /* 3a. 999 blinks */
		if ((s / 3) & 1) {
			memset(e->led_wbuf + 1, 0, EP1_BUFSIZE - 1);
		} else {
			boot_set_number(e, 999);
			e->led_wbuf[5] = 0xFF;
			e->led_wbuf[6] = 1;
		}
		led_write_frame(e);
		return;
	}
	s -= RK3_BOOT_FLASH;
	if (s < RK3_BOOT_SPIN) {                           /* 3b. spinner + sparkle */
		memset(e->led_wbuf + 1, 0, EP1_BUFSIZE - 1);
		for (int d = 0; d < 4; d++)                /* segments a..f in a ring,
		                                            * each digit one step behind */
			e->led_wbuf[1 + d] = (uint8_t)(1u << ((s + 6 - d) % 6));
		e->led_wbuf[5] = (uint8_t)((s * 0x9D + 0x35) ^ (s << 3));
		e->led_wbuf[6] = (uint8_t)((s >> 1) & 1);
		led_write_frame(e);
		return;
	}
	midi_show_con(e);                                  /* 4. settle + stop */
	boot_anim_stop_timer(e);
}

/* Cut the animation short (engine thread). Waits briefly for an in-flight
 * display write to complete so the command pipe is free, then leaves "Con" up. */
static void boot_anim_cancel(struct rk3_engine *e)
{
	if (!e->boot_timer)
		return;
	boot_anim_stop_timer(e);
	for (int i = 0; i < 20 && e->led_busy; i++)
		CFRunLoopRunInMode(kCFRunLoopDefaultMode, 0.005, true);
	midi_show_con(e);
	for (int i = 0; i < 20 && e->led_busy; i++)
		CFRunLoopRunInMode(kCFRunLoopDefaultMode, 0.005, true);
}

static void boot_anim_start(struct rk3_engine *e)
{
	e->boot_step = 0;
	CFRunLoopTimerContext tctx = { 0, e, NULL, NULL, NULL };
	e->boot_timer = CFRunLoopTimerCreate(NULL,
			CFAbsoluteTimeGetCurrent(),   /* first frame immediately */
			RK3_BOOT_TICK_S,
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
	/* Tuning datagram: "TUNE in_fpx out_fpx n_out setpoint48 floor48" (0 = keep).
	 * Validated by rk3_engine_set_tuning; applies at the next audio_start. */
	if (n > 5 && n < (ssize_t)sizeof(buf) && memcmp(buf, "TUNE ", 5) == 0) {
		int v[5] = { 0, 0, 0, 0, 0 };
		buf[n] = 0;
		sscanf((const char *)buf + 5, "%d %d %d %d %d",
		       &v[0], &v[1], &v[2], &v[3], &v[4]);
		rk3_engine_set_tuning(e, v[0], v[1], v[2], v[3], v[4]);
		return;
	}
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

	CFRunLoopTimerContext wctx = { 0, e, NULL, NULL, NULL };
	e->wd_timer = CFRunLoopTimerCreate(NULL, CFAbsoluteTimeGetCurrent() + 0.5,
			0.5, 0, 0, watchdog_tick, &wctx);
	if (e->wd_timer)
		CFRunLoopAddTimer(CFRunLoopGetCurrent(), e->wd_timer,
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
	boot_anim_stop_timer(e);          /* boot animation still running at close */
	if (e->wd_timer) {
		CFRunLoopTimerInvalidate(e->wd_timer);
		CFRelease(e->wd_timer);
		e->wd_timer = NULL;
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

/*
 * Run `work` on the engine thread's run loop and block (bounded) until it
 * completes. The isoc prime/abort must touch the IOUSBLib async machinery from
 * the run-loop thread, but StartIO/StopIO call in on CoreAudio's thread — this
 * marshals the work across. Bounded wait so a wedged engine can't hang StartIO.
 */
static bool engine_sync(struct rk3_engine *e, int max_ms, void (^work)(void))
{
	if (!e->runloop || !atomic_load_explicit(&e->running, memory_order_acquire))
		return false;
	atomic_int done;
	atomic_init(&done, 0);
	atomic_int *donep = &done;
	CFRunLoopPerformBlock(e->runloop, kCFRunLoopDefaultMode, ^{
		work();
		atomic_store_explicit(donep, 1, memory_order_release);
	});
	CFRunLoopWakeUp(e->runloop);
	for (int i = 0; i < max_ms * 2; i++) {    /* usleep(500) => 2 iters/ms */
		if (atomic_load_explicit(&done, memory_order_acquire))
			return true;
		usleep(500);
	}
	ELOG("engine_sync: timed out (%d ms) waiting for engine thread", max_ms);
	return false;
}

/* Reset all audio-streaming state (rings, cadence mirror, diagnostics, clock
 * telemetry) for a fresh audio_start. Does NOT touch USB / thread / MIDI. */
static void reset_audio_state(struct rk3_engine *e)
{
	e->pat_head = e->pat_tail = e->out_acc = 0;
	e->pat_stage_off = e->in_fpx;             /* force a pop on first OUT fill */
	e->in_errs = e->out_errs = e->out_drops = 0;
	e->in_resyncs = e->out_resyncs = 0;
	e->out_errs_seen = 0;
	e->out_shrink = 0;
	e->audio_t0 = e->out_decay_t = e->out_err_time = mach_absolute_time();
	e->out_grow_time = e->out_bump_t = 0;   /* no hold-off pending */
	e->cap_quiet = e->cap_holdoff = 0;
	e->brk_cb = 0;
	e->brk_t0 = 0;
	memset(e->in_res, 0, sizeof(e->in_res));
	memset(e->out_res, 0, sizeof(e->out_res));

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
	e->cap_drains = 0;
	e->out_fill_min = ~0u;
	e->cap_fill_min = ~0u;
	atomic_store(&e->out_trough, ~0u);
	e->servo_underruns = 0;
	atomic_store(&e->pb_w, 0);
	atomic_store(&e->pb_r, 0);
	atomic_store(&e->pb_started, 0);
	atomic_store(&e->cap_overruns, 0);
	atomic_store(&e->cap_underruns, 0);
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
	e->out_fill_setpoint = RK3_SCALE_TO_RATE(atomic_load(&e->tune_out_setpoint), e->rate);
	e->cap_fill_floor    = RK3_SCALE_TO_RATE(atomic_load(&e->tune_cap_floor), e->rate);
	e->cap_settle_init   = RK3_SCALE_TO_RATE(RK3_CAP_SETTLE_FRAMES, e->rate);
	e->cap_measure_init  = RK3_SCALE_TO_RATE(RK3_CAP_MEASURE_FRAMES, e->rate);
	e->cap_redrain_init  = RK3_SCALE_TO_RATE(RK3_CAP_REDRAIN_FRAMES, e->rate);
	e->cap_redrain_hyst  = RK3_SCALE_TO_RATE(RK3_CAP_REDRAIN_HYST, e->rate);
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
	/* The learned extras are frame counts at the old rate; carry them over as
	 * the same amount of TIME (unscaled, 15 ms learned at 192 kHz became 60 ms
	 * at 48 kHz). */
	if (e->rate > 0 && rate != e->rate) {
		atomic_store(&e->out_extra, (int)((long long)atomic_load(&e->out_extra)
						  * rate / e->rate));
		atomic_store(&e->cap_extra, (int)((long long)atomic_load(&e->cap_extra)
						  * rate / e->rate));
	}
	e->rate = rate;
	e->bpp = caiaq_compute_bpp(rate, e->data_alignment, e->n_streams);
	engine_apply_rate_scale(e);
	ELOG("set_rate: rate=%d bpp=%d (takes effect at next audio_start)", e->rate, e->bpp);
	return 0;
}

int rk3_engine_out_fill_setpoint(struct rk3_engine *e)
{
	/* tuned trough target + whatever the stream has had to add (RK3_ADAPT_*) */
	return e->out_fill_setpoint +
	       atomic_load_explicit(&e->out_extra, memory_order_relaxed);
}

/* True once the stream is past its start-up transient (adaptive margins only
 * learn from steady-state misses). */
static bool adapt_warm(struct rk3_engine *e, UInt64 now)
{
	return ticks_to_ms(now - e->audio_t0) >= RK3_ADAPT_WARMUP_S * 1000.0;
}

/*
 * Widen a ring margin after a miss of `shortfall` frames. A miss longer than
 * the largest margin we would ever hold is not jitter but a stall — the host
 * stopping, reconfiguring or hanging (a DAW changing its sample rate starved
 * the ring for 230 ms just before StopIO and taught the driver the maximum
 * margin for nothing). No margin covers those, so they are not learned from.
 */
static void adapt_bump(struct rk3_engine *e, atomic_int *extra, int shortfall)
{
	if (shortfall > e->rate * RK3_ADAPT_MAX_MS / 1000)
		return;
	int cap = e->rate * RK3_ADAPT_BUMP_MAX_MS / 1000;
	if (shortfall > cap) shortfall = cap;
	int v = atomic_load_explicit(extra, memory_order_relaxed) + shortfall +
		e->rate * RK3_ADAPT_BUMP_MS / 1000;
	int max = e->rate * RK3_ADAPT_MAX_MS / 1000;
	atomic_store_explicit(extra, v > max ? max : v, memory_order_relaxed);
}

/* Give 1/16 of a ring's extra margin back (called every RK3_ADAPT_DECAY_S quiet). */
static void adapt_decay(atomic_int *extra)
{
	int v = atomic_load_explicit(extra, memory_order_relaxed);
	if (v > 0)
		atomic_store_explicit(extra, v - (v / 16 > 0 ? v / 16 : 1),
				      memory_order_relaxed);
}

static int fpx_ok(int fpx)
{
	return fpx == 8 || fpx == 16 || fpx == 32;
}

int rk3_engine_set_tuning(struct rk3_engine *e, int in_fpx, int out_fpx,
			  int n_out, int out_setpoint48, int cap_floor48)
{
	if (!in_fpx)         in_fpx = atomic_load(&e->tune_in_fpx);
	if (!out_fpx)        out_fpx = atomic_load(&e->tune_out_fpx);
	if (!n_out)          n_out = atomic_load(&e->tune_n_out);
	if (!out_setpoint48) out_setpoint48 = atomic_load(&e->tune_out_setpoint);
	if (!cap_floor48)    cap_floor48 = atomic_load(&e->tune_cap_floor);

	if (!fpx_ok(in_fpx) || !fpx_ok(out_fpx) || in_fpx > RK3_FRAMES_PER_XFER ||
	    in_fpx % out_fpx != 0 || n_out < 2 || n_out > RK3_MAX_OUT_XFERS ||
	    (n_out - 1) * (out_fpx / 8) < 2 ||   /* < 2 ms lead: submits get rejected */
	    out_setpoint48 < 16 || out_setpoint48 > 2048 ||
	    cap_floor48 < 8 || cap_floor48 > 2048) {
		ELOG("set_tuning: rejected in_fpx=%d out_fpx=%d n_out=%d setpoint=%d floor=%d",
		     in_fpx, out_fpx, n_out, out_setpoint48, cap_floor48);
		return -1;
	}
	atomic_store(&e->tune_in_fpx, in_fpx);
	atomic_store(&e->tune_out_fpx, out_fpx);
	atomic_store(&e->tune_n_out, n_out);
	atomic_store(&e->tune_out_setpoint, out_setpoint48);
	atomic_store(&e->tune_cap_floor, cap_floor48);
	atomic_store(&e->out_extra, 0);       /* a new tuning starts from scratch */
	atomic_store(&e->cap_extra, 0);
	ELOG("set_tuning: in_fpx=%d out_fpx=%d n_out=%d setpoint=%d floor=%d "
	     "(takes effect at next audio_start)",
	     in_fpx, out_fpx, n_out, out_setpoint48, cap_floor48);
	return 0;
}

/* Seed the pending tuning with the compiled defaults the first time through
 * (the engine struct is zero-initialised; a reopen keeps whatever was tuned). */
static void tuning_init(struct rk3_engine *e)
{
	if (atomic_load(&e->tune_in_fpx) != 0)
		return;
	atomic_store(&e->tune_in_fpx, RK3_DEFAULT_IN_FPX);
	atomic_store(&e->tune_out_fpx, RK3_DEFAULT_OUT_FPX);
	atomic_store(&e->tune_n_out, RK3_DEFAULT_N_OUT);
	atomic_store(&e->tune_out_setpoint, RK3_OUT_FILL_SETPOINT);
	atomic_store(&e->tune_cap_floor, RK3_CAP_FILL_FLOOR);
}

/* Latch the pending tuning into the live geometry. Audio must be stopped. */
static void tuning_latch(struct rk3_engine *e)
{
	e->in_fpx  = atomic_load(&e->tune_in_fpx);
	e->out_fpx = atomic_load(&e->tune_out_fpx);
	e->n_out   = atomic_load(&e->tune_n_out);
	e->n_out_base = e->n_out;
	engine_apply_rate_scale(e);
}

int rk3_engine_out_latency_frames(const struct rk3_engine *e)
{
	/* servo-held standing OUT-ring fill + the n_out-deep isoc pipeline (one
	 * transfer = fpx microframes, 8 microframes = 1 ms). */
	return e->out_fill_setpoint + e->n_out * (e->out_fpx * e->rate / 8000);
}

int rk3_engine_in_latency_frames(const struct rk3_engine *e)
{
	return e->in_fpx * e->rate / 8000;   /* one capture transfer */
}

int rk3_engine_fault(struct rk3_engine *e)
{
	return atomic_load_explicit(&e->fault, memory_order_acquire) != 0;
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
	e->wd_timer = NULL;
	atomic_store(&e->audio_wanted, 0);
	atomic_store(&e->fault, 0);
	tuning_init(e);
	e->in_fpx  = atomic_load(&e->tune_in_fpx);
	e->out_fpx = atomic_load(&e->tune_out_fpx);
	e->n_out   = atomic_load(&e->tune_n_out);

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
	if (atomic_load_explicit(&e->audio_wanted, memory_order_acquire))
		return 0;                         /* already streaming */

	/* Rings/diagnostics are reset here (no consumer yet: StartIO hasn't returned
	 * so no DoIOOperation runs). The USB command I/O (AUDIO_PARAMS, which reads a
	 * reply on EP 0x81) and the isoc prime are done on the engine thread so they
	 * never run concurrently with the engine's own USB traffic. */
	tuning_latch(e);
	reset_audio_state(e);
	e->restarts = 0;
	atomic_store_explicit(&e->fault, 0, memory_order_release);
	atomic_store_explicit(&e->audio_wanted, 1, memory_order_release);
	atomic_store_explicit(&e->audio_running, 1, memory_order_release);

	__block int params_rc = -1;
	bool ok = engine_sync(e, 9000, ^{
		/*
		 * Quiesce the MIDI relay's async EP 0x81 read first: AUDIO_PARAMS reads
		 * its ack on the SAME bulk-in pipe, and two readers on one pipe conflict.
		 * We're on the engine thread, so this is strictly sequential; the aborted
		 * read's stale completion is harmless (its midi_running guard returns it).
		 */
		/* The power-on animation writes to the command pipe AUDIO_PARAMS is
		 * about to use (an overlapping write timed AUDIO_PARAMS out). */
		boot_anim_cancel(e);

		int relay_was_on = atomic_load_explicit(&e->midi_running, memory_order_acquire);
		if (relay_was_on) {
			atomic_store_explicit(&e->midi_running, 0, memory_order_release);
			(*e->usb.intf)->AbortPipe(e->usb.intf, e->usb.pipe_cmd_in);
		}

		/*
		 * AUDIO_PARAMS reads its ack on EP 0x81 (pipe_cmd_in), the pipe the MIDI
		 * relay's async read was just aborted on. Do NOT "resync" that pipe with
		 * ClearPipeStallBothEnds first: it resets the HOST's data toggle, but the
		 * RK3 firmware keeps its own, so half the time (whenever the device's
		 * toggle is odd) the two then disagree and the host discards the next
		 * packet — the ack. Measured with the device otherwise idle, GET_DEVICE_INFO
		 * request/reply:
		 *     plain                              20/20
		 *     ClearPipeStallBothEnds before each  9/20  (strictly alternating)
		 *     async read + AbortPipe, no clear   20/20
		 *     async read + AbortPipe + clear      9/20
		 * That clear was the cause of the "AUDIO_PARAMS attempt 1 failed" seen on
		 * about every other StartIO, and of the occasional three-in-a-row that
		 * failed StartIO outright. AbortPipe alone leaves the toggles in step.
		 *
		 * The retry stays for the one legitimate case: after the interface is
		 * re-opened the host toggle starts at 0, and a lost packet is what puts
		 * the two back in step — so simply asking again succeeds.
		 */
		for (int attempt = 0; attempt < 4; attempt++) {
			params_rc = caiaq_set_audio_params(&e->usb, e->rate, 24, e->bpp);
			if (params_rc == 0)
				break;
			ELOG("audio_start: AUDIO_PARAMS attempt %d got no ack; retrying",
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
		atomic_store_explicit(&e->audio_wanted, 0, memory_order_release);
		atomic_store_explicit(&e->audio_running, 0, memory_order_release);
		ELOG("audio_start: failed (sync_ok=%d params_rc=%d)", ok, params_rc);
		return -1;
	}
	ELOG("audio_start: isoc chains primed (rate=%d bpp=%d in_fpx=%d out_fpx=%d "
	     "n_out=%d setpoint=%d floor=%d)", e->rate, e->bpp, e->in_fpx, e->out_fpx,
	     e->n_out, e->out_fill_setpoint, e->cap_fill_floor);
	return 0;
}

void rk3_engine_audio_stop(struct rk3_engine *e)
{
	if (!atomic_load_explicit(&e->running, memory_order_acquire))
		return;
	/* audio_wanted (not audio_running) is the "streaming" state here: a tripped
	 * breaker has audio_running=0 but still needs the abort + the watchdog off. */
	if (!atomic_exchange_explicit(&e->audio_wanted, 0, memory_order_acq_rel))
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

#ifdef RK3_TEST_HOOKS
void rk3_engine_test_burst(struct rk3_engine *e)
{
	engine_sync(e, 1000, ^{
		for (int i = 0; i < 2 * RK3_BRK_WINDOW; i++)
			if (!brk_completion(e, e->in_res, kIOReturnSuccess))
				break;
	});
}
#endif

int rk3_engine_present(const struct rk3_engine *e)
{
	return rk3_usb_present(&e->usb);
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

/* Capture-ring margin: frames that will be LEFT after this read takes nframes
 * (0 if the read is about to underrun). This, not the pre-read fill, is what the
 * floor protects — the pre-read fill has to cover the host buffer first, so a
 * floor expressed in pre-read frames only worked at one buffer size. */
static inline unsigned cap_margin(unsigned w, unsigned r, int nframes)
{
	unsigned fill = ring_avail(w, r) / RK3_BYTES_PER_FRAME;
	return fill > (unsigned)nframes ? fill - (unsigned)nframes : 0;
}

void rk3_engine_read_input(struct rk3_engine *e, float *dst, int nframes)
{
	int need = nframes * RK3_BYTES_PER_FRAME;
	unsigned r = atomic_load_explicit(&e->cap_r, memory_order_relaxed);
	unsigned w = atomic_load_explicit(&e->cap_w, memory_order_acquire);
	/* tuned margin + what this stream has had to add (RK3_ADAPT_*) */
	const int cap_floor = e->cap_fill_floor +
		atomic_load_explicit(&e->cap_extra, memory_order_relaxed);

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
			unsigned hf = cap_margin(w, r, nframes);
			if (hf < e->cap_meas_min) e->cap_meas_min = hf;  /* find the trough */
			e->cap_measure_left -= nframes;
		} else {
			if (e->cap_meas_min != ~0u &&
			    e->cap_meas_min > (unsigned)cap_floor) {
				unsigned drop = (e->cap_meas_min - cap_floor)
						* RK3_BYTES_PER_FRAME;
				r = (r + drop) % RK3_RING_BYTES;   /* trough -> floor */
			}
			atomic_store_explicit(&e->cap_primed, 1, memory_order_relaxed);
			e->cap_drains++;
			e->cap_meas_min = ~0u;
			e->cap_measure_left = e->cap_redrain_init;
		}
	} else if (atomic_load_explicit(&e->cap_primed, memory_order_relaxed)) {
		/* Periodic re-drain (see RK3_CAP_REDRAIN_FRAMES): keep the capture
		 * trough on the floor as the OUT servo hands fill over to this ring. */
		unsigned hf = cap_margin(w, r, nframes);
		if (hf < e->cap_meas_min) e->cap_meas_min = hf;
		e->cap_measure_left -= nframes;
		if (e->cap_measure_left <= 0) {
			if (e->cap_meas_min != ~0u && e->cap_meas_min >
			    (unsigned)(cap_floor + e->cap_redrain_hyst)) {
				unsigned drop = (e->cap_meas_min - cap_floor)
						* RK3_BYTES_PER_FRAME;
				r = (r + drop) % RK3_RING_BYTES;
				e->cap_drains++;
			}
			e->cap_meas_min = ~0u;
			e->cap_measure_left = e->cap_redrain_init;
		}
	}

	unsigned avail = ring_avail(w, r);
	{                                   /* windowed-min capture margin telemetry */
		unsigned af = cap_margin(w, r, nframes);
		if (af < e->cap_fill_min) e->cap_fill_min = af;
	}
	if (e->cap_holdoff > 0)
		e->cap_holdoff -= nframes;
	e->cap_quiet += nframes;
	if (avail < (unsigned)need &&
	    atomic_load_explicit(&e->cap_primed, memory_order_relaxed)) {
		atomic_fetch_add_explicit(&e->cap_underruns, 1, memory_order_relaxed);
		/* one bump per disturbance (RK3_ADAPT_HOLD_S): a single late
		 * delivery can short many consecutive reads */
		if (e->cap_holdoff <= 0 && adapt_warm(e, mach_absolute_time())) {
			adapt_bump(e, &e->cap_extra,
				   (int)((unsigned)need - avail) / RK3_BYTES_PER_FRAME);
			e->cap_holdoff = e->rate * RK3_ADAPT_HOLD_S;
		}
		e->cap_quiet = 0;
	} else if (e->cap_quiet >= e->rate * RK3_ADAPT_DECAY_S) {
		adapt_decay(&e->cap_extra);
		e->cap_quiet = 0;
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
	    w >= (unsigned)e->out_cushion &&
	    w >= (unsigned)(2 * nframes + rk3_engine_out_fill_setpoint(e))
			* RK3_BYTES_PER_FRAME) {
		atomic_store_explicit(&e->pb_r, RK3_OUT_PHASE, memory_order_relaxed);
		atomic_store_explicit(&e->pb_started, 1, memory_order_release);
	}
}

bool rk3_engine_out_trough(struct rk3_engine *e, int *trough)
{
	if (!atomic_load_explicit(&e->pb_started, memory_order_acquire))
		return false;
	unsigned m = atomic_exchange_explicit(&e->out_trough, ~0u, memory_order_relaxed);
	unsigned u = atomic_load_explicit(&e->pb_underruns, memory_order_relaxed);
	unsigned missed = u - e->servo_underruns;   /* OUT packets sent as silence */
	e->servo_underruns = u;
	UInt64 now = mach_absolute_time();
	if (missed) {
		if (adapt_warm(e, now) &&
		    ticks_to_ms(now - e->out_bump_t) >= RK3_ADAPT_HOLD_S * 1000.0) {
			/* one packet carries bpp/8 frames (6 audio bytes per frame,
			 * 3 of every 4 packet bytes are audio) */
			adapt_bump(e, &e->out_extra, (int)missed * e->bpp / 8);
			e->out_bump_t = now;
		}
		e->out_decay_t = now;
		*trough = -e->out_fill_setpoint;
		return true;
	}
	if (ticks_to_ms(now - e->out_decay_t) >= RK3_ADAPT_DECAY_S * 1000.0) {
		adapt_decay(&e->out_extra);
		e->out_decay_t = now;
	}
	if (m == ~0u)
		return false;
	*trough = (int)m;
	return true;
}

int rk3_engine_out_fill_frames(struct rk3_engine *e)
{
	if (!atomic_load_explicit(&e->pb_started, memory_order_acquire))
		return -1;
	unsigned w = atomic_load_explicit(&e->pb_w, memory_order_acquire);
	unsigned r = atomic_load_explicit(&e->pb_r, memory_order_relaxed);
	return (int)(ring_avail(w, r) / RK3_BYTES_PER_FRAME);
}
