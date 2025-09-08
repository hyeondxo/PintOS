#ifndef THREADS_SYNCH_H
#define THREADS_SYNCH_H

#include <list.h>
#include <stdbool.h>

/* 카운팅 세마포어(counting semaphore).
   - value: 가용 자원의 개수(0 이상). 0이면 대기 필요.
   - waiters: 자원이 없을 때 수면(BLOCKED) 상태로 전환된 스레드들의 대기열.
     세마포어 up(V) 시 이 목록에서 하나를 깨운다. */
struct semaphore {
	unsigned value;             /* 현재 세마포어 값(가용 자원 수). */
	struct list waiters;        /* 대기 중인 스레드 리스트(깨울 대상 큐). */
};

void sema_init (struct semaphore *, unsigned value);
void sema_down (struct semaphore *);
bool sema_try_down (struct semaphore *);
void sema_up (struct semaphore *);
void sema_self_test (void);

/* 락(lock).
   - 동시에 하나의 스레드만 임계구역에 들어가도록 보장한다.
   - holder: 현재 락을 소유한 스레드(디버깅/검증용).
   - semaphore: 내부적으로 초기값 1의 이진 세마포어로 상호배제를 구현한다. */
struct lock {
	struct thread *holder;      /* 현재 락 소유자(디버깅/ASSERT용 확인 수단). */
	struct semaphore semaphore; /* 접근 제어를 담당하는 이진 세마포어(초깃값 1). */
};

void lock_init (struct lock *);
void lock_acquire (struct lock *);
bool lock_try_acquire (struct lock *);
void lock_release (struct lock *);
bool lock_held_by_current_thread (const struct lock *);

/* 조건변수(condition variable).
   - 특정 조건이 만족될 때까지 스레드를 수면시키고, 신호(signal)로 깨운다.
   - waiters: 대기 중인 스레드들의 큐(실제로는 각 스레드별 0값 세마포어 래퍼가 들어감).
   - Mesa 스타일(비원자) 모니터: signal 후에는 보통 조건을 재확인해야 한다. */
struct condition {
	struct list waiters;        /* 대기 중인 스레드(세마포어 래퍼) 리스트. */
};

void cond_init (struct condition *);
void cond_wait (struct condition *, struct lock *);
void cond_signal (struct condition *, struct lock *);
void cond_broadcast (struct condition *, struct lock *);

/* 최적화 장벽(optimization barrier).
 *
 * 이 매크로를 경계로 컴파일러가 메모리 접근을 재배치(reorder)하지 못하도록 막는다.
 * 메모리 가시성 보장이나 하드웨어/인터럽트 상호작용 시, 의도치 않은 재정렬을 방지하는 데 사용한다. */
#define barrier() asm volatile ("" : : : "memory")

#endif /* threads/synch.h */
