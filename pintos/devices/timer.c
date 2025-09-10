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
static struct list sleep_list;

/* 타이머 틱당 수행 가능한 루프(iteration) 수.
   timer_calibrate()에서 보정(calibration)된다. */
static unsigned loops_per_tick;

static intr_handler_func timer_interrupt;
static bool too_many_loops(unsigned loops);
static void busy_wait(int64_t loops);
static void real_time_sleep(int64_t num, int32_t denom);
static bool sleep_less(const struct list_elem *a, const struct list_elem *b, void *aux UNUSED);

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
    list_init(&sleep_list);
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

/*
 * timer_sleep: 약 ticks 틱 동안 현재 스레드를 "실제로" 잠재운다.
 * - busy waiting(while + yield) 대신, 깨울 절대시각(wake_tick)을 계산하여 전역
 *   수면 리스트(sleep_list)에 오름차순으로 삽입하고, 스레드를 BLOCKED로 전환한다.
 * - 리스트 조작과 BLOCKED 전환은 경쟁을 피하기 위해 인터럽트를 비활성화한
 *   원자 구간에서 수행한다.
 * - 기한이 도래하면 타이머 인터럽트 핸들러(timer_interrupt)가 READY로 깨운다.
 */
void timer_sleep(int64_t ticks) {
    if (ticks <= 0) {
        // 0 이하 요청은 대기 의미가 없으므로 즉시 복귀
        return;
    }
    // 현재 절대시각(누적 틱)에 요청한 상대 틱을 더해 깨울 절대시각 계산
    int64_t wake = timer_ticks() + ticks;

    // 리스트 조작 + 상태 전환을 원자적으로 수행하기 위해 인터럽트 비활성화
    enum intr_level old_level = intr_disable();
    // 현재 실행 중 스레드 포인터 획득
    struct thread *cur = thread_current();
    // 스레드에 자신의 깨울 시각 기록(정렬 비교 기준)
    cur->wake_tick = wake;
    // 깨울 시각 오름차순으로 수면 리스트에 삽입(가장 이른 항목이 맨 앞)
    list_insert_ordered(&sleep_list, &cur->sleep_elem, sleep_less, NULL);
    // 현재 스레드를 BLOCKED로 전환하여 스케줄 대상에서 제외(진짜 수면)
    thread_block();
    // 호출 전 인터럽트 상태로 정확히 복원
    intr_set_level(old_level);

    // int64_t start = timer_ticks();

    // ASSERT(intr_get_level() == INTR_ON);
    // while (timer_elapsed(start) < ticks)
    //     thread_yield(); /* 남은 틱 동안 다른 스레드에게 CPU를 양보 */
}

/* 약 MS 밀리초 동안 실행을 중단한다. */
void timer_msleep(int64_t ms) { real_time_sleep(ms, 1000); }

/* 약 US 마이크로초 동안 실행을 중단한다. */
void timer_usleep(int64_t us) { real_time_sleep(us, 1000 * 1000); }

/* 약 NS 나노초 동안 실행을 중단한다. */
void timer_nsleep(int64_t ns) { real_time_sleep(ns, 1000 * 1000 * 1000); }

/* 타이머 통계를 출력한다(누적 틱 수). */
void timer_print_stats(void) { printf("Timer: %" PRId64 " ticks\n", timer_ticks()); }

/*
 * timer_interrupt: 타이머 인터럽트 핸들러(매 틱 호출).
 * - 전역 틱 카운터(ticks)를 1 증가시키고, 스케줄러에 틱 경과를 통지(thread_tick).
 * - 수면 리스트의 맨 앞(가장 이른 기한)부터 현재 시각(now) 이하인 스레드를 READY로 깨운다.
 *   아직 기한이 남아 있으면(맨 앞 wake_tick > now) 더 뒤 항목도 모두 남았으므로 즉시 종료.
 */
static void timer_interrupt(struct intr_frame *args UNUSED) {
    // 한 틱 경과 표시
    ticks++;
    // 스케줄러에 틱 경과 통지(타임 슬라이스 관리/통계 등)
    thread_tick();

    // 현재 절대시각을 지역 변수로 보관(반복 연산 감소)
    int64_t now = ticks;
    // 수면 리스트에 후보가 있는 동안 반복
    while (!list_empty(&sleep_list)) {
        // 맨 앞: 가장 이른 기한을 가진 스레드
        struct thread *t = list_entry(list_front(&sleep_list), struct thread, sleep_elem);
        if (t->wake_tick > now) {
            // 아직 깨우기 이르고, 리스트는 정렬되어 있으므로 즉시 종료
            break;
        }
        // 맨 앞 항목 제거(이제 기상 시간 도래)
        list_pop_front(&sleep_list);
        // READY로 깨워 준비 큐에 삽입(인터럽트 컨텍스트에서는 수면 불가)
        thread_unblock(t);
    }

    /* 2) MLFQS 갱신(있을 때만) */
    if (thread_mlfqs) {
        mlfqs_increment(); /* 매 틱: running(=idle 제외) recent_cpu += 1 */

        if (ticks % TIMER_FREQ == 0) {
            mlfqs_load_avg();                           /* 1초마다 */
            mlfqs_recalc_all_recent_cpu_and_priority(); /* 전체 recent_cpu + priority */
        } else if (ticks % 4 == 0) {
            mlfqs_recalc_all_recent_cpu_and_priority(); /* 4틱마다 priority만 갱신 분기 */
        }
    }
}

/* 주어진 반복 횟수(LOOPS)가 한 틱을 초과하는지 판정: 초과하면 true, 아니면 false */
static bool too_many_loops(unsigned loops) {
    // 다음 틱이 올 때까지 대기
    int64_t start = ticks;
    while (ticks == start)
        barrier();

    // LOOPS 만큼 바쁜 루프 실행
    start = ticks;
    busy_wait(loops);

    // 틱 값이 변했다면 반복 시간이 한 틱을 초과한 것
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
        // 한 틱 이상 대기해야 하는 경우: CPU 양보 기반의 timer_sleep() 사용
        timer_sleep(ticks);
    } else {
        // 그보다 짧은 경우: 더 정확한 서브-틱 지연을 위해 바쁜 대기 사용
        // (오버플로 위험을 줄이기 위해 분자/분모를 1000으로 스케일 다운)
        ASSERT(denom % 1000 == 0);
        busy_wait(loops_per_tick * num / 1000 * TIMER_FREQ / (denom / 1000));
    }
}

// a, b: 정렬 비교 대상이 되는 리스트 노드 포인터(list_elem)
// - 이 노드들은 실제로 struct thread 안에 포함된 sleep_elem 필드의 주소
// aux: 추가 비교 정보가 필요할 때 넘겨줄 수 있는 사용자 데이터 포인터
// - 여기서는 사용하지 않으므로 UNUSED로 표시
static bool sleep_less(const struct list_elem *a, const struct list_elem *b, void *aux UNUSED) {
    // list_entry 매크로:
    // - 주어진 list_elem*가 "어느 구조체의 어떤 필드"였는지 알려주면,
    //   내부적으로 offsetof와 포인터 산술로 "부모 구조체의 시작 주소"를 역추적
    // - 여기서는 list_elem* a/b가 struct thread의 sleep_elem 필드였음을 알려,
    //   각각을 포함하고 있는 struct thread* (ta, tb)를 얻음
    const struct thread *ta = list_entry(a, struct thread, sleep_elem);
    const struct thread *tb = list_entry(b, struct thread, sleep_elem);

    // 비교 기준:
    // - 스레드의 깨울 시각(wake_tick)이 더 빠른(작은) 쪽이 리스트에서 "앞"에 오도록
    //   true/false를 반환
    // - 즉, 오름차순(작은 값이 먼저) 정렬
    return ta->wake_tick < tb->wake_tick;
}
