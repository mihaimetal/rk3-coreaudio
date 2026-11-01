#include "../runloop_sync.h"
#include <assert.h>
#include <pthread.h>
#include <stdio.h>

static CFRunLoopRef worker_loop;
static atomic_bool ready, stop;
static void timer_tick(CFRunLoopTimerRef t, void *p) { (void)t; (void)p; }
static void *worker(void *p) {
    (void)p;
    worker_loop = CFRunLoopGetCurrent();
    CFRunLoopTimerRef timer = CFRunLoopTimerCreate(NULL, CFAbsoluteTimeGetCurrent()+10,
        10, 0, 0, timer_tick, NULL);
    CFRunLoopAddTimer(worker_loop, timer, kCFRunLoopDefaultMode);
    atomic_store(&ready, true);
    while (!atomic_load(&stop)) CFRunLoopRunInMode(kCFRunLoopDefaultMode, .01, false);
    CFRunLoopTimerInvalidate(timer); CFRelease(timer);
    return NULL;
}
/* Exactly the original failing ordering: return on timeout, then execute the
 * queued block. Also verifies cancelled work cannot touch expired caller data. */
static void cancelled_call(void) {
    int local = 0;
    int *pointer = &local;
    assert(!rk3_runloop_sync(CFRunLoopGetCurrent(), 1, ^{ *pointer = 99; }));
    assert(local == 0);
}
int main(void) {
    for (int i=0; i<100; ++i) {
        cancelled_call();
        CFRunLoopRunInMode(kCFRunLoopDefaultMode, .001, false);
    }
    pthread_t thread;
    assert(pthread_create(&thread, NULL, worker, NULL) == 0);
    while (!atomic_load(&ready)) usleep(500);
    __block int result = 0;
    assert(rk3_runloop_sync(worker_loop, 1000, ^{ result = 42; }));
    assert(result == 42);
    /* A running operation outlives its admission timeout, but not its caller. */
    double before = rk3_sync_now_ms();
    assert(rk3_runloop_sync(worker_loop, 100, ^{ usleep(200000); result = 43; }));
    assert(result == 43 && rk3_sync_now_ms()-before >= 190);
    for (int i=0; i<500; ++i) {
        __block int calls = 0;
        bool ok = rk3_runloop_sync(worker_loop, 0, ^{ calls++; });
        assert(calls == (ok ? 1 : 0));
    }
    atomic_store(&stop, true); CFRunLoopWakeUp(worker_loop);
    pthread_join(thread, NULL);
    puts("Run-loop cancellation, completion, in-flight join, and races passed.");
}
