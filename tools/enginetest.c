/*
 * enginetest.c — run the REAL audio engine (rk3_engine.c) in an ordinary process.
 *
 * The installed driver runs this same engine inside coreaudiod, where a runaway
 * completion loop takes the whole audio subsystem down. Here the worst case is a
 * process you can kill, so this is where a new transport geometry (transfer
 * sizes / pipeline depth / fill targets) gets proven first.
 *
 * A real-time thread stands in for CoreAudio's IO thread: every --buf frames it
 * calls rk3_engine_read_input / rk3_engine_write_output, paced by the same
 * fill servo the plugin uses (Drv_GetZeroTimeStamp). It emits one impulse per
 * second and times its return on the input, so with a cable from an RK3 output
 * to an RK3 input it reports the engine's own round trip (no CoreAudio in the
 * path — compare configs with it, don't quote it as the DAW number).
 *
 * The device must be free: uninstall the driver first (sudo make uninstall-driver).
 *
 * Usage: ./build/enginetest [seconds] [--rate N] [--in-fpx N] [--out-fpx N]
 *            [--n-out N] [--setpoint N] [--floor N] [--buf N] [--thresh X]
 *            [--trip]     (inject a breaker trip at half time; checks recovery)
 */

#include "../rk3_engine.h"

#include <mach/mach.h>
#include <mach/mach_time.h>
#include <mach/thread_policy.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/resource.h>
#include <unistd.h>

#define MAX_BUF   1024
#define IMP_LEN   (g_rate / 1000)   /* 1 ms at any rate */
#define IMP_AMP   0.95f
#define MAX_MEAS  256

static struct rk3_engine g_eng;          /* ~1 MB of rings; keep off the stack */
static volatile sig_atomic_t g_stop;
static int    g_rate = 48000, g_buf = 64;
static float  g_thresh = 0.005f;
static double g_meas[MAX_MEAS];
static volatile int g_nmeas;
static volatile float g_inpeak;
static volatile int g_io_overruns;       /* IO ticks that woke > 1 period late */

static void on_sigint(int s) { (void)s; g_stop = 1; }

static double ticks_per_sec(void)
{
	mach_timebase_info_data_t tb;
	mach_timebase_info(&tb);
	return 1e9 * (double)tb.denom / (double)tb.numer;
}

static void *io_thread(void *arg)
{
	(void)arg;
	const double tps = ticks_per_sec();
	const double nominal = tps * g_buf / g_rate;       /* ticks per IO cycle */

	thread_time_constraint_policy_data_t pol = {
		.period = (uint32_t)nominal, .computation = (uint32_t)(nominal * 0.2),
		.constraint = (uint32_t)(nominal * 0.9), .preemptible = 1 };
	thread_policy_set(mach_thread_self(), THREAD_TIME_CONSTRAINT_POLICY,
			  (thread_policy_t)&pol, THREAD_TIME_CONSTRAINT_POLICY_COUNT);

	static float in[MAX_BUF * 2], out[MAX_BUF * 2];
	double t = 0;                         /* sample time at the block start */
	double next_imp = g_rate;             /* 1 s warm-up */
	double imp_time = -1;                 /* pending impulse start, -1 = idle */
	int imp_left = 0;
	double fill_avg = -1, trim = 0;
	double deadline = (double)mach_absolute_time();
	int cycles_per_servo = (int)(8192 / g_buf), servo_ctr = 0;

	while (!g_stop) {
		rk3_engine_read_input(&g_eng, in, g_buf);
		for (int i = 0; i < g_buf; i++) {
			float m = fabsf(in[i * 2]), m2 = fabsf(in[i * 2 + 1]);
			if (m2 > m) m = m2;
			if (m > g_inpeak) g_inpeak = m;
			if (imp_time >= 0 && m > g_thresh) {
				if (g_nmeas < MAX_MEAS)
					g_meas[g_nmeas++] = t + i - imp_time;
				imp_time = -1;
			}
		}
		if (imp_time >= 0 && t - imp_time > g_rate / 2)
			imp_time = -1;        /* never came back (no cable) */

		for (int i = 0; i < g_buf; i++) {
			if (imp_time < 0 && imp_left == 0 && t + i >= next_imp) {
				imp_left = IMP_LEN;
				imp_time = t + i;
				next_imp += g_rate;
			}
			float v = 0;
			if (imp_left > 0) { v = IMP_AMP; imp_left--; }
			out[i * 2] = out[i * 2 + 1] = v;
		}
		rk3_engine_write_output(&g_eng, out, g_buf);
		t += g_buf;

		/* Fill servo, sampled once per 8192 frames exactly like the plugin. */
		if (++servo_ctr >= cycles_per_servo) {
			servo_ctr = 0;
			int fill;
			if (rk3_engine_out_trough(&g_eng, &fill)) {
				int sp = rk3_engine_out_fill_setpoint(&g_eng);
				if (fill_avg < 0) fill_avg = sp;
				else fill_avg += 0.40 * (fill - fill_avg);   /* kFillServoAlpha */
				trim = -(0.75 * (fill_avg - sp)) / g_rate;
				if (trim > 0.01) trim = 0.01;
				if (trim < -0.01) trim = -0.01;
			}
		}

		deadline += nominal / (1.0 + trim);
		double now = (double)mach_absolute_time();
		if (now > deadline + nominal) {       /* woke late: don't burst to catch up */
			g_io_overruns++;
			deadline = now;
		}
		mach_wait_until((uint64_t)deadline);
	}
	return NULL;
}

static double cpu_seconds(void)
{
	struct rusage ru;
	getrusage(RUSAGE_SELF, &ru);
	return ru.ru_utime.tv_sec + ru.ru_utime.tv_usec / 1e6 +
	       ru.ru_stime.tv_sec + ru.ru_stime.tv_usec / 1e6;
}

static int cmp_d(const void *a, const void *b)
{
	double x = *(const double *)a, y = *(const double *)b;
	return (x > y) - (x < y);
}

int main(int argc, char **argv)
{
	double seconds = 12;
	int in_fpx = 0, out_fpx = 0, n_out = 0, setpoint = 0, floorv = 0, trip = 0;
	for (int i = 1; i < argc; i++) {
		int more = i + 1 < argc;
		if (!strcmp(argv[i], "--rate") && more) g_rate = atoi(argv[++i]);
		else if (!strcmp(argv[i], "--in-fpx") && more) in_fpx = atoi(argv[++i]);
		else if (!strcmp(argv[i], "--out-fpx") && more) out_fpx = atoi(argv[++i]);
		else if (!strcmp(argv[i], "--n-out") && more) n_out = atoi(argv[++i]);
		else if (!strcmp(argv[i], "--setpoint") && more) setpoint = atoi(argv[++i]);
		else if (!strcmp(argv[i], "--floor") && more) floorv = atoi(argv[++i]);
		else if (!strcmp(argv[i], "--buf") && more) g_buf = atoi(argv[++i]);
		else if (!strcmp(argv[i], "--thresh") && more) g_thresh = (float)atof(argv[++i]);
		else if (!strcmp(argv[i], "--trip")) trip = 1;
		else seconds = atof(argv[i]);
	}
	if (g_buf < 16 || g_buf > MAX_BUF) { fprintf(stderr, "--buf 16..%d\n", MAX_BUF); return 1; }

	signal(SIGINT, on_sigint);
	signal(SIGTERM, on_sigint);

	if (rk3_engine_open(&g_eng, g_rate) != 0) {
		fprintf(stderr, "engine open failed — is the RK3 plugged in and the "
			"driver uninstalled (sudo make uninstall-driver)?\n");
		return 1;
	}
	if (rk3_engine_set_tuning(&g_eng, in_fpx, out_fpx, n_out, setpoint, floorv) != 0) {
		fprintf(stderr, "invalid tuning\n");
		rk3_engine_close(&g_eng);
		return 1;
	}
	if (rk3_engine_audio_start(&g_eng) != 0) {
		fprintf(stderr, "audio_start failed\n");
		rk3_engine_close(&g_eng);
		return 1;
	}
	printf("rate=%d buf=%d  in_fpx=%d out_fpx=%d n_out=%d  setpoint=%d floor=%d  "
	       "(%d isoc completions/s nominal)\n", g_rate, g_buf, g_eng.in_fpx,
	       g_eng.out_fpx, g_eng.n_out, g_eng.out_fill_setpoint, g_eng.cap_fill_floor,
	       8000 / g_eng.in_fpx + 8000 / g_eng.out_fpx);

	pthread_t th;
	pthread_create(&th, NULL, io_thread, NULL);

	double cpu0 = cpu_seconds(), cpu_max = 0, cpu_sum = 0;
	int cpu_n = 0;
	int faults = 0, last_n = 0;
	for (int s = 1; s <= (int)seconds && !g_stop; s++) {
		sleep(1);
		double cpu1 = cpu_seconds(), cpu = (cpu1 - cpu0) * 100.0;
		cpu0 = cpu1;
		if (cpu > cpu_max) cpu_max = cpu;
		if (s > 2) { cpu_sum += cpu; cpu_n++; }   /* skip start-up */
		int fault = rk3_engine_fault(&g_eng);
		faults += fault;
		printf("%3ds cpu=%4.1f%%  out_fill=%4d "
		       "pb_under=%u cap_over=%u  errs=%d/%d resync=%u/%u io_late=%d "
		       "fault=%d restarts=%d chk_errs=%u out_panic=%d n_out=%d", s, cpu,
		       rk3_engine_out_fill_frames(&g_eng),
		       atomic_load(&g_eng.pb_underruns), atomic_load(&g_eng.cap_overruns),
		       g_eng.in_errs, g_eng.out_errs, g_eng.in_resyncs, g_eng.out_resyncs,
		       g_io_overruns, fault, g_eng.restarts, g_eng.codec.input_errs,
		       g_eng.codec.output_panic, g_eng.n_out);
		for (; last_n < g_nmeas; last_n++)
			printf("  lat=%.0f", g_meas[last_n]);
		printf("\n");
		fflush(stdout);
		if (trip && s == (int)seconds / 2) {
			printf("    -- injecting a completion burst (breaker test)\n");
			rk3_engine_test_burst(&g_eng);
		}
	}

	g_stop = 1;
	pthread_join(th, NULL);
	rk3_engine_stop(&g_eng);

	int n = g_nmeas;
	printf("peak input %.4f; cpu mean %.1f%% max %.1f%% of one core; "
	       "fault-seconds %d\n", (double)g_inpeak,
	       cpu_n ? cpu_sum / cpu_n : 0.0, cpu_max, faults);
	if (n > 0) {
		/* the last third is post-capture-drain steady state */
		int tail = n / 3 > 0 ? n / 3 : 1;
		double *st = g_meas + (n - tail);
		qsort(st, tail, sizeof(double), cmp_d);
		printf("round trip (engine only), last %d impulses: min %.0f  median %.0f  "
		       "max %.0f frames = %.2f / %.2f / %.2f ms\n", tail, st[0],
		       st[tail / 2], st[tail - 1], st[0] * 1e3 / g_rate,
		       st[tail / 2] * 1e3 / g_rate, st[tail - 1] * 1e3 / g_rate);
	} else {
		printf("no impulses detected (cable? threshold %.3f)\n", (double)g_thresh);
	}
	return 0;
}
