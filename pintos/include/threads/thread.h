#ifndef THREADS_THREAD_H
#define THREADS_THREAD_H

#include "threads/interrupt.h"
#include <debug.h>
#include <list.h>
#include <stdint.h>
#ifdef VM
#include "vm/vm.h"
#endif

/* 스레드 생애주기 상태 집합 */
enum thread_status {
    THREAD_RUNNING, /* 실행 중인 스레드 */
    THREAD_READY,   /* 실행 중은 아니지만 즉시 실행 가능(준비 상태) */
    THREAD_BLOCKED, /* 특정 이벤트(조건) 대기 중인 상태 */
    THREAD_DYING    /* 곧 파괴(해제)될 상태 */
};

/* 스레드 식별자 타입.
   필요하다면 다른 정수형으로 재정의할 수 있다. */
typedef int tid_t;
#define TID_ERROR ((tid_t) - 1) /* 오류를 나타내는 tid 값 */

/* 스레드 우선순위 범위 */
#define PRI_MIN 0              /* 최소(가장 낮은) 우선순위 */
#define PRI_DEFAULT 31         /* 기본 우선순위 */
#define PRI_MAX 63             /* 최대(가장 높은) 우선순위 */
#define DONATION_DEPTH_LIMIT 8 // 최대 깊이 제한(순환, 무한 전파 방지)

/* 커널 스레드 또는 사용자 프로세스.
 *
 * 각 스레드 구조체(struct thread)는 자신의 4 kB 페이지 하나에 저장된다.
 * 스레드 구조체 자체는 페이지의 맨 아래(오프셋 0)에 위치하고,
 * 나머지 영역은 해당 스레드의 커널 스택 용도로 예약된다. 커널 스택은 페이지 상단(오프셋 4 kB)에서
 * 시작하여 아래 방향으로 성장한다. 메모리 배치는 다음과 같다:
 *
 *      4 kB +---------------------------------+
 *           |          kernel stack           |
 *           |                |                |
 *           |                |                |
 *           |                V                |
 *           |         grows downward          |
 *           |                                 |
 *           |                                 |
 *           |                                 |
 *           |                                 |
 *           |                                 |
 *           |                                 |
 *           |                                 |
 *           |                                 |
 *           +---------------------------------+
 *           |              magic              |
 *           |            intr_frame           |
 *           |                :                |
 *           |                :                |
 *           |               name              |
 *           |              status             |
 *      0 kB +---------------------------------+
 *
 * 이 설계가 주는 중요한 함의는 두 가지다:
 *
 *    1. struct thread의 크기가 너무 커지면 안 된다. 구조체가 커지면 같은 페이지 안에
 *       커널 스택을 위한 공간이 부족해진다. 기본 struct thread는 수십~수백 바이트 규모이며,
 *       바람직하게는 1 kB 미만으로 유지되어야 한다.
 *
 *    2. 커널 스택이 너무 커지도록 두어서는 안 된다. 스택 오버플로가 발생하면 스레드 상태
 *       (아래의 magic 값 포함)를 덮어써 손상시킬 수 있다. 그러므로 커널 함수에서 큰 구조체나
 *       배열을 비정적 지역변수로 할당하지 말고, 필요하면 malloc()이나 palloc_get_page() 같은
 *       동적 할당을 사용하라.
 *
 * 위 문제들의 첫 증상은 대개 thread_current()에서의 ASSERT 실패로 나타난다.
 * thread_current()는 현재 실행 중인 스레드의 struct thread 안에 있는 magic 필드가
 * THREAD_MAGIC인지 확인한다. 스택 오버플로는 이 값을 바꿔버려 ASSERT를 유발한다. */
/* elem 필드는 이중 용도를 가진다.
 * 실행 큐(run queue, thread.c)에서의 리스트 노드로도 쓰이고,
 * 세마포어 대기 리스트(synch.c)의 노드로도 쓰인다. 이 두 용도는 상호 배타적이어서 충돌하지 않는다:
 * READY 상태의 스레드만 실행 큐에 존재하고, BLOCKED 상태의 스레드만 세마포어 대기 리스트에 존재한다. */
struct thread {
    /* thread.c에서 관리하는 필드 */
    tid_t tid;                 /* 스레드 고유 식별자(tid). 시스템 전역에서 유일해야 함. */
    enum thread_status status; /* 스레드 상태(READY/RUNNING/BLOCKED/DYING). 스케줄러가 참조. */
    char name[16];             /* 스레드 이름(디버깅 및 로깅 용도). 최대 15자+NULL. */
    int priority;              /* 스케줄링 우선순위(PRI_MIN~PRI_MAX). 높을수록 우선. */

    /* thread.c와 synch.c에서 공유하여 사용 */
    struct list_elem elem; /* 실행 큐 또는 동기화 대기열에 들어갈 리스트 노드 */

    int64_t wake_tick;           // 깨울 시각
    struct list_elem sleep_elem; // 수면 리스트 노드

    int base_priority;              // 현재 스레드의 원래 우선순위
    struct lock *wait_on_lock;      // 현재 스레드가 획득 대기 중인 락 포인터 - 어떤 락을 얻기 위함인지?
    struct list donations;          // 나에게 우선순위를 기부한 스레드들 리스트
    struct list_elem donation_elem; // donation 리스트 전용 손잡이(중요)

    struct list held_locks; // 내가 현재 보유중인 락들의 리스트

#ifdef USERPROG
    /* userprog/process.c에서 관리 */
    uint64_t *pml4; /* 사용자 주소 공간의 최상위 페이지 테이블(PML4) 포인터 */
#endif
#ifdef VM
    /* 이 스레드가 소유한 전체 가상 메모리를 추적하는 보조 페이지 테이블 */
    struct supplemental_page_table spt;
#endif

    /* thread.c에서 관리 */
    struct intr_frame tf; /* 문맥 전환을 위한 레지스터/세그먼트 스냅샷 저장 영역 */
    unsigned magic;       /* 스택 오버플로 감지를 위한 센티넬 값(THREAD_MAGIC 기대) */
};

/* false(기본)이면 라운드로빈 스케줄러 사용.
   true면 MLFQS(다단계 피드백 큐) 스케줄러 사용.
   커널 커맨드라인 옵션 "-o mlfqs"로 제어. */
extern bool thread_mlfqs;

void thread_init(void);
void thread_start(void);

void thread_tick(void);
void thread_print_stats(void);

typedef void thread_func(void *aux);
tid_t thread_create(const char *name, int priority, thread_func *, void *);

void thread_block(void);
void thread_unblock(struct thread *);

struct thread *thread_current(void);
tid_t thread_tid(void);
const char *thread_name(void);

void thread_exit(void) NO_RETURN;
void thread_yield(void);

int thread_get_priority(void);
void thread_set_priority(int);

int thread_get_nice(void);
void thread_set_nice(int);
int thread_get_recent_cpu(void);
int thread_get_load_avg(void);

void do_iret(struct intr_frame *tf);

/* 우선순위 스케줄링용 비교자: 더 높은 priority가 앞에 오도록 */
bool compare_thread_priority(const struct list_elem *a, const struct list_elem *b, void *aux);

/* 우선순위 기부를 체인으로 전파 */
void donate_priority_chain(struct thread *donee);

void refresh_priority(struct thread *t);

#endif /* threads/thread.h */
