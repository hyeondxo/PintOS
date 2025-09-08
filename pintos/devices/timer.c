#include "devices/timer.h"
#include "threads/interrupt.h"
#include "threads/io.h"
#include "threads/synch.h"
#include "threads/thread.h"
#include <debug.h>
#include <inttypes.h>
#include <round.h>
#include <stdio.h>

/* 8254 타이머 칩의 하드웨어 상세는 [8254] 문서를 참고. */

#if TIMER_FREQ < 19
#error 8254 timer requires TIMER_FREQ >= 19
#endif
#if TIMER_FREQ > 1000
#error TIMER_FREQ <= 1000 recommended
#endif

/* OS 부팅 이후 누적된 타이머 틱 수 */
static int64_t ticks;

/* 타이머 틱당 수행 가능한 루프(iteration) 수.
   timer_calibrate()에서 보정(calibration)된다. */
static unsigned loops_per_tick;

static intr_handler_func timer_interrupt;
static bool too_many_loops(unsigned loops);
static void busy_wait(int64_t loops);
static void real_time_sleep(int64_t num, int32_t denom);

/*
 * timer_init: 8254 PIT을 1초에 TIMER_FREQ회 인터럽트가 발생하도록 설정하고,
 *             해당 인터럽트(IRQ0, 벡터 0x20)의 핸들러를 등록한다.
 */
void timer_init(void) {
    /* 8254 입력 주파수(1,193,180 Hz)를 TIMER_FREQ로 나눈 카운트 값을 반올림하여 설정 */
    uint16_t count = (1193180 + TIMER_FREQ / 2) / TIMER_FREQ;

    outb(0x43, 0x34); /* 제어어: 카운터 0, LSB→MSB, 모드2(Rate Generator), 바이너리 */
    outb(0x40, count & 0xff);
    outb(0x40, count >> 8);

    intr_register_ext(0x20, timer_interrupt, "8254 Timer");
}

/* 짧은 지연 구현에 사용할 loops_per_tick 값을 보정(calibrate)한다. */
void timer_calibrate(void) {
    unsigned high_bit, test_bit;

    ASSERT(intr_get_level() == INTR_ON);
    printf("Calibrating timer...  ");

    /* 한 틱보다 작은 범위에서 가능한 가장 큰 2의 거듭제곱으로 대략값을 찾는다. */
    loops_per_tick = 1u << 10;
    while (!too_many_loops(loops_per_tick << 1)) {
        loops_per_tick <<= 1;
        ASSERT(loops_per_tick != 0);
    }

    /* 이후 8비트를 정밀 보정하여 더 정확한 값으로 다듬는다. */
    high_bit = loops_per_tick;
    for (test_bit = high_bit >> 1; test_bit != high_bit >> 10; test_bit >>= 1)
        if (!too_many_loops(high_bit | test_bit))
            loops_per_tick |= test_bit;

    printf("%'" PRIu64 " loops/s.\n", (uint64_t)loops_per_tick * TIMER_FREQ);
}

/* OS 부팅 이후 현재까지의 누적 타이머 틱 수를 반환한다. */
int64_t timer_ticks(void) {
    enum intr_level old_level = intr_disable();
    int64_t t = ticks;
    intr_set_level(old_level);
    barrier();
    return t;
}

/* 과거 시점 THEN(과거의 timer_ticks() 값)으로부터 경과한 타이머 틱 수를 반환한다. */
int64_t timer_elapsed(int64_t then) { return timer_ticks() - then; }

/* 약 TICKS 틱 동안 실행을 중단한다(바쁜 대기 대신 양보 기반). */
void timer_sleep(int64_t ticks) { //
    int64_t start = timer_ticks();

    ASSERT(intr_get_level() == INTR_ON);
    while (timer_elapsed(start) < ticks)
        thread_yield(); /* 남은 틱 동안 다른 스레드에게 CPU를 양보 */
}

/* 약 MS 밀리초 동안 실행을 중단한다. */
void timer_msleep(int64_t ms) { real_time_sleep(ms, 1000); }

/* 약 US 마이크로초 동안 실행을 중단한다. */
void timer_usleep(int64_t us) { real_time_sleep(us, 1000 * 1000); }

/* 약 NS 나노초 동안 실행을 중단한다. */
void timer_nsleep(int64_t ns) { real_time_sleep(ns, 1000 * 1000 * 1000); }

/* 타이머 통계를 출력한다(누적 틱 수). */
void timer_print_stats(void) { printf("Timer: %" PRId64 " ticks\n", timer_ticks()); }

/* 타이머 인터럽트 핸들러: 매 틱마다 ticks를 증가시키고 스케줄러에 틱을 통지 */
static void timer_interrupt(struct intr_frame *args UNUSED) {
    ticks++;
    thread_tick();
}

/* 주어진 반복 횟수(LOOPS)가 한 틱을 초과하는지 판정: 초과하면 true, 아니면 false */
static bool too_many_loops(unsigned loops) {
    /* 다음 틱이 올 때까지 대기 */
    int64_t start = ticks;
    while (ticks == start)
        barrier();

    /* LOOPS 만큼 바쁜 루프 실행 */
    start = ticks;
    busy_wait(loops);

    /* 틱 값이 변했다면 반복 시간이 한 틱을 초과한 것 */
    barrier();
    return start != ticks;
}

/* 간단한 루프를 LOOPS 횟수만큼 수행하여 매우 짧은 지연을 구현.
   NO_INLINE 이유: 코드 정렬(alignment)이 시간에 큰 영향을 줄 수 있어,
   다른 위치에 인라인될 경우 결과가 달라질 수 있기 때문이다. */
static void NO_INLINE busy_wait(int64_t loops) {
    while (loops-- > 0)
        barrier();
}

/* 약 NUM/DENOM 초 만큼 수면한다. */
static void real_time_sleep(int64_t num, int32_t denom) {
    /* NUM/DENOM 초를 타이머 틱으로 변환(내림).

       (NUM / DENOM) s
       ---------------------- = NUM * TIMER_FREQ / DENOM ticks
       1 s / TIMER_FREQ ticks
     */
    int64_t ticks = num * TIMER_FREQ / denom;

    ASSERT(intr_get_level() == INTR_ON);
    if (ticks > 0) {
        /* 한 틱 이상 대기해야 하는 경우: CPU 양보 기반의 timer_sleep() 사용 */
        timer_sleep(ticks);
    } else {
        /* 그보다 짧은 경우: 더 정확한 서브-틱 지연을 위해 바쁜 대기 사용
           (오버플로 위험을 줄이기 위해 분자/분모를 1000으로 스케일 다운) */
        ASSERT(denom % 1000 == 0);
        busy_wait(loops_per_tick * num / 1000 * TIMER_FREQ / (denom / 1000));
    }
}
