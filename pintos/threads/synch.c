/* This file is derived from source code for the Nachos
   instructional operating system.  The Nachos copyright notice
   is reproduced in full below. */

/* Copyright (c) 1992-1996 The Regents of the University of California.
   All rights reserved.

   Permission to use, copy, modify, and distribute this software
   and its documentation for any purpose, without fee, and
   without written agreement is hereby granted, provided that the
   above copyright notice and the following two paragraphs appear
   in all copies of this software.

   IN NO EVENT SHALL THE UNIVERSITY OF CALIFORNIA BE LIABLE TO
   ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
   CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE
   AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY OF CALIFORNIA
   HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

   THE UNIVERSITY OF CALIFORNIA SPECIFICALLY DISCLAIMS ANY
   WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
   PURPOSE.  THE SOFTWARE PROVIDED HEREUNDER IS ON AN "AS IS"
   BASIS, AND THE UNIVERSITY OF CALIFORNIA HAS NO OBLIGATION TO
   PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR
   MODIFICATIONS.
   */

#include "threads/synch.h"     // 세마포어/락/조건변수 인터페이스
#include "threads/interrupt.h" // 인터럽트 제어/상태
#include "threads/thread.h"    // 스레드/스케줄러 연동
#include <stdio.h>             // printf 등 표준 입출력
#include <string.h>            // memset 등 문자열 유틸

/* 세마포어 SEMA를 VALUE로 초기화.
   세마포어는 0 이상의 정수와 두 가지 원자적 연산으로 구성된다:

   - down("P"): 값이 양수가 될 때까지 기다렸다가 1 감소

   - up("V"): 값을 1 증가(대기 중인 스레드가 있으면 하나 깨움) */
void sema_init(struct semaphore *sema, unsigned value) {
    ASSERT(sema != NULL);

    sema->value = value;       // 현재 세마포어 값 설정
    list_init(&sema->waiters); // 대기자 리스트 초기화
}

/* 세마포어의 down("P"): SEMA 값이 양수가 될 때까지 기다렸다가 원자적으로 1 감소.

   이 함수는 수면할 수 있으므로 인터럽트 핸들러에서 호출하면 안 된다.
   인터럽트 비활성 상태에서 호출될 수는 있으나, 수면에 들어가면 다음 스케줄된 스레드가
   보통 인터럽트를 다시 켠다. */
void sema_down(struct semaphore *sema) {
    /**
     * 왜 현재 인터럽트 상태를 저장해야 하는가?
     * 세마포어/락 구현은 준비 큐 조작이나 스레드 상태 전환 등 원자적이어야 하는 구간이 많음
     * 이를 위해 이전 상태의 인터럽트를 저장 후 상태 변경 등을 안전하게 수행 한 뒤
     * 저장해두었던 상태로 정확히 복원하는 과정이 필요
     */
    enum intr_level old_level;

    ASSERT(sema != NULL);
    ASSERT(!intr_context());

    old_level = intr_disable(); // 원자적 큐 조작을 위해 인터럽트 비활성
    while (sema->value == 0) {  // 자원이 없으면 대기열에 들어가 BLOCKED
        list_insert_ordered(&sema->waiters, &thread_current()->elem, compare_thread_priority, NULL);
        thread_block();
    }
    sema->value--;             // 임계구역 진입 권한 획득
    intr_set_level(old_level); // 인터럽트 복원
}

/* 세마포어 down("P")의 논블로킹 버전: 값이 0이 아닐 때만 1 감소.
   감소에 성공하면 true, 아니면 false 반환.

   인터럽트 핸들러에서도 호출 가능. */
bool sema_try_down(struct semaphore *sema) {
    enum intr_level old_level;
    bool success;

    ASSERT(sema != NULL);

    old_level = intr_disable();
    if (sema->value > 0) // 자원이 있으면 즉시 획득
    {
        sema->value--;
        success = true;
    } else // 없으면 실패
        success = false;
    intr_set_level(old_level);

    return success;
}

/* 세마포어 up("V"): SEMA 값을 1 증가시키고, 대기 중인 스레드가 있으면 하나를 깨운다.
   인터럽트 핸들러에서도 호출 가능. */
void sema_up(struct semaphore *sema) {
    enum intr_level old = intr_disable();
    ASSERT(sema != NULL);

    sema->value++; // 먼저 올림
    struct thread *to_unblock = NULL;

    if (!list_empty(&sema->waiters)) {
        list_sort(&sema->waiters, compare_thread_priority, NULL);
        to_unblock = list_entry(list_pop_front(&sema->waiters), struct thread, elem);
        thread_unblock(to_unblock);
    }

    bool need_yield = (to_unblock && to_unblock->priority > thread_current()->priority);

    intr_set_level(old); // 인터럽트 복원 후 결정

    if (intr_context())
        intr_yield_on_return();
    else if (need_yield)
        thread_yield();
}

static void sema_test_helper(void *sema_);

/* 세마포어 셀프 테스트: 두 스레드 간에 제어권을 핑퐁시킴.
   진행 상황은 printf()를 삽입해 확인할 수 있다. */
void sema_self_test(void) {
    struct semaphore sema[2];
    int i;

    printf("Testing semaphores...");
    sema_init(&sema[0], 0);
    sema_init(&sema[1], 0);
    thread_create("sema-test", PRI_DEFAULT, sema_test_helper, &sema);
    for (i = 0; i < 10; i++) {
        sema_up(&sema[0]);   // 보조 스레드를 깨움
        sema_down(&sema[1]); // 보조 스레드의 신호를 기다림
    }
    printf("done.\n");
}

/* sema_self_test()에서 사용하는 보조 스레드 함수 */
static void sema_test_helper(void *sema_) {
    struct semaphore *sema = sema_;
    int i;

    for (i = 0; i < 10; i++) {
        sema_down(&sema[0]); // 메인 스레드의 신호를 기다림
        sema_up(&sema[1]);   // 메인 스레드에게 신호 보냄
    }
}

/* LOCK 초기화.
   락은 언제나 동시에 하나의 스레드만 보유할 수 있다. 이 구현의 락은 재귀적(recursive)이지 않아서,
   이미 보유 중인 스레드가 동일한 락을 다시 획득하려 하면 오류다.

   락은 초기값이 1인 세마포어의 특수화다. 하지만 차이가 두 가지 있다:
   (1) 세마포어 값은 1보다 클 수 있지만, 락은 항상 단일 소유자만 가질 수 있다.
   (2) 세마포어는 소유자 개념이 없어 한 스레드가 down하고 다른 스레드가 up할 수 있지만,
       락은 같은 스레드가 acquire와 release를 모두 수행해야 한다.
   이런 제약이 번거롭다면 락 대신 세마포어 사용을 고려하라. */
void lock_init(struct lock *lock) {
    ASSERT(lock != NULL);

    lock->holder = NULL;            // 현재 소유자 없음
    sema_init(&lock->semaphore, 1); // 이진 세마포어로 초기화
}

/* LOCK 획득. 필요하면 사용 가능할 때까지 수면.
   현재 스레드가 이미 해당 락을 보유하고 있으면 안 된다.

   이 함수는 수면할 수 있으므로 인터럽트 핸들러에서는 호출 금지.
   인터럽트 비활성 상태에서 호출될 수 있으나, 수면이 필요하면 인터럽트는 다시 켜질 수 있다. */
void lock_acquire(struct lock *lock) {
    ASSERT(lock != NULL);
    ASSERT(!intr_context());
    ASSERT(!lock_held_by_current_thread(lock));
    struct thread *cur = thread_current();

    // 락에 이미 소유자가 있고 그게 나 자신이 아니라면 나는 이 락을 기다리는 상태가 됨
    if (lock->holder && lock->holder != cur) {
        // 내가 지금 어떤 락을 기다리는지 표시(기부 체인 추적용)
        cur->wait_on_lock = lock;
        // 현재 스레드의 높은 우선순위를 락 소유자에게 기부하고
        // 소유자가 또다른 락을 기다리면 그 소유자에게도 연쇄적으로 기부를 전파함
        // -> 우선순위 역전 방지 목적
        donate_priority_chain(lock->holder);
    }
    // 락 내부의 세마포어를 내려 자원을 획득
    sema_down(&lock->semaphore); // 자원 획득까지 대기
    // 더이상 락을 기다리는 상태가 아님. 표시 해제
    cur->wait_on_lock = NULL;
    lock->holder = cur;                            // 이제 이 락의 소유자는 나임
    list_push_back(&cur->held_locks, &lock->elem); // 내가 보유중인 락 목록에 이 락을 추가
}

/* LOCK을 시도(acquire)하고, 성공 시 true, 실패 시 false 반환.
   현재 스레드가 이미 보유 중이면 안 된다.

   수면하지 않으므로 인터럽트 핸들러에서도 호출 가능. */
bool lock_try_acquire(struct lock *lock) {
    bool success;

    ASSERT(lock != NULL);
    ASSERT(!lock_held_by_current_thread(lock));

    success = sema_try_down(&lock->semaphore); // 즉시 획득 가능하면 성공
    if (success)
        lock->holder = thread_current(); // 소유자 기록
    return success;
}

// 특정 락 때문에 들어온 기부들을 모두 회수. 우선순위 정상화
static void remove_donation_for_lock(struct thread *cur, struct lock *lock) {
    // 현재 스레드(cur)에 대한 도네이터 목록을 순회하며, 이 lock 때문에 기부한 항목만 제거
    struct list_elem *e = list_begin(&cur->donations);
    while (e != list_end(&cur->donations)) {
        struct thread *d = list_entry(e, struct thread, donation_elem);
        struct list_elem *next = list_next(e);
        if (d->wait_on_lock == lock) {
            list_remove(e);
        }
        e = next;
    }
}

/* LOCK 해제. 현재 스레드가 소유 중이어야 한다.

   인터럽트 핸들러는 락을 획득할 수 없으므로, 여기서 락 해제를 시도하는 건 의미 없다. */
void lock_release(struct lock *lock) {
    ASSERT(lock != NULL);
    ASSERT(lock_held_by_current_thread(lock));
    struct thread *cur = thread_current();

    lock->holder = NULL; // 소유자 해제
    list_remove(&lock->elem);
    remove_donation_for_lock(cur, lock);
    refresh_priority(cur);

    sema_up(&lock->semaphore); // 대기자 하나 깨우고 자원 반환
}

/* 현재 스레드가 LOCK을 보유 중이면 true, 아니면 false 반환.
   (다른 스레드가 보유 중인지 검사하는 건 경쟁(racy) 상황이므로 지양) */
bool lock_held_by_current_thread(const struct lock *lock) {
    ASSERT(lock != NULL);

    return lock->holder == thread_current();
}

/* 조건변수 대기열에 들어가는 세마포어 래퍼 하나 */
struct semaphore_elem {
    struct list_elem elem;      /* 리스트 노드 */
    struct semaphore semaphore; /* 개별 세마포어 */
    int priority;
};

/* 조건변수 COND 초기화.
   한 코드가 조건 발생을 신호하고, 협력하는 다른 코드가 이를 수신하여 동작하도록 한다. */
void cond_init(struct condition *cond) {
    ASSERT(cond != NULL);

    list_init(&cond->waiters); // 조건변수 대기열 초기화
}

static bool compare_waiter_priority(const struct list_elem *a, const struct list_elem *b, void *aux UNUSED) {
    const struct semaphore_elem *sa = list_entry(a, struct semaphore_elem, elem);
    const struct semaphore_elem *sb = list_entry(b, struct semaphore_elem, elem);
    return sa->priority > sb->priority;
}
/* LOCK을 원자적으로 해제하고, 다른 코드가 보낸 COND 신호를 기다린다.
   신호를 받으면 반환 전에 LOCK을 다시 획득한다. 이 함수 호출 전 LOCK을 보유하고 있어야 한다.

   이 모니터는 Hoare 스타일이 아닌 Mesa 스타일이므로, 신호 보냄과 받음은 원자적이지 않다.
   따라서 wait가 끝난 뒤에는 보통 조건을 재검사하고, 필요하면 다시 대기해야 한다.

   하나의 조건변수는 하나의 락에만 연계되지만, 하나의 락에는 여러 조건변수가 연계될 수 있다
   (락:조건변수 = 1:다 관계).

   이 함수는 수면할 수 있으므로 인터럽트 핸들러에서는 호출 금지. 인터럽트 비활성에서 호출될 수 있으나,
   수면이 필요하면 인터럽트는 다시 켜질 수 있다. */
void cond_wait(struct condition *cond, struct lock *lock) {
    struct semaphore_elem waiter;

    ASSERT(cond != NULL);
    ASSERT(lock != NULL);
    ASSERT(!intr_context());
    ASSERT(lock_held_by_current_thread(lock));

    sema_init(&waiter.semaphore, 0); // 개별 세마포어를 0으로 초기화(대기 상태)
    waiter.priority = thread_current()->priority;
    // 아직 waiter.semaphore.waiters는 비어 있으므로, cond->waiters에는 일단 넣고
    // signal 시점에 compare_sema_elem으로 재정렬하여 최상위 우선순위를 보장한다.
    list_insert_ordered(&cond->waiters, &waiter.elem, compare_waiter_priority, NULL);

    // list_push_back(&cond->waiters, &waiter.elem);
    lock_release(lock);           // 락을 원자적으로 해제
    sema_down(&waiter.semaphore); // 신호를 받을 때까지 대기
    lock_acquire(lock);           // 깨어난 뒤 락 재획득
}

/* LOCK으로 보호되는 COND에서 대기 중인 스레드가 있다면 하나를 깨운다.
   호출 전 LOCK을 보유하고 있어야 한다.

   인터럽트 핸들러는 락을 획득할 수 없으므로, 조건변수 신호를 인터럽트 컨텍스트에서
   보내는 것은 의미 없다. */
void cond_signal(struct condition *cond, struct lock *lock UNUSED) {
    ASSERT(cond != NULL);
    ASSERT(lock != NULL);
    ASSERT(!intr_context());
    ASSERT(lock_held_by_current_thread(lock));

    if (!list_empty(&cond->waiters)) {
        // pop 전에 우선순위 기준으로 재정렬하여 항상 최상위가 먼저 깨어나도록 보장
        list_sort(&cond->waiters, compare_waiter_priority, NULL);
        struct semaphore_elem *se = list_entry(list_pop_front(&cond->waiters), struct semaphore_elem, elem);
        // 세마포어 래퍼 안의 개별 세마포어를 증가시켜야 함
        sema_up(&se->semaphore);
    }
}

/* LOCK으로 보호되는 COND에서 대기 중인 모든 스레드를 깨운다.
   호출 전 LOCK을 보유하고 있어야 한다.

   인터럽트 핸들러는 락을 획득할 수 없으므로, 조건변수 신호를 인터럽트 컨텍스트에서
   보내는 것은 의미 없다. */
void cond_broadcast(struct condition *cond, struct lock *lock) {
    ASSERT(cond != NULL);
    ASSERT(lock != NULL);

    while (!list_empty(&cond->waiters))
        cond_signal(cond, lock);
}
