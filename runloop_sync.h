#pragma once
#include <CoreFoundation/CoreFoundation.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <time.h>
#include <unistd.h>

/* A copied block owns the shared byref state until it is executed/disposed.
 * Timeout cancels only queued work. Once work starts we must join it: returning
 * early would let the caller reset/close resources still used by that work.
 * Thus max_ms bounds queue admission, not execution of an admitted USB call.
 */
static inline double rk3_sync_now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1000000.0;
}
static inline bool rk3_runloop_sync(CFRunLoopRef loop, int max_ms, void (^work)(void))
{
    enum { queued, running, finished, cancelled };
    __block atomic_int state;
    atomic_init(&state, queued);
    double deadline = rk3_sync_now_ms() + max_ms;
    CFRunLoopPerformBlock(loop, kCFRunLoopDefaultMode, ^{
        int expected = queued;
        if (!atomic_compare_exchange_strong(&state, &expected, running))
            return;
        work();
        atomic_store_explicit(&state, finished, memory_order_release);
    });
    CFRunLoopWakeUp(loop);
    for (;;) {
        if (atomic_load_explicit(&state, memory_order_acquire) == finished)
            return true;
        if (rk3_sync_now_ms() >= deadline) {
            int expected = queued;
            if (atomic_compare_exchange_strong(&state, &expected, cancelled))
                return false;
            /* The callback won admission: join it before releasing its inputs. */
        }
        usleep(500);
    }
}
