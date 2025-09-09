#include "threads/thread.h"     // 스레드 구조체와 스케줄러 관련 기본 선언
#include "intrinsic.h"          // 낮은 수준의 레지스터 접근(rrsp 등)
#include "threads/flags.h"      // EFLAGS 비트 정의
#include "threads/interrupt.h"  // 인터럽트 제어/상태 관련
#include "threads/intr-stubs.h" // 인터럽트 스텁 정의
#include "threads/palloc.h"     // 페이지 할당기
#include "threads/synch.h"      // 세마포어/락 등 동기화 원시 연산
#include "threads/vaddr.h"      // 가상주소 헬퍼 (pg_round_down 등)
#include <debug.h>              // ASSERT 등 디버그 헬퍼
#include <random.h>             // 난수 유틸 (mlfqs 등에서 사용 가능)
#include <stddef.h>             // size_t, NULL 등 표준 정의
#include <stdio.h>              // printf 등 표준 입출력
#include <string.h>             // memset/strlcpy 등 문자열 유틸
#ifdef USERPROG
#include "userprog/process.h" // 사용자 프로세스 주소공간 전환 등
#endif

/* struct thread의 `magic` 값.
   스택 오버플로 감지용. 자세한 내용은 thread.h 상단 주석 참조. */
#define THREAD_MAGIC 0xcd6abf4b // 스택 오버플로 감지를 위한 매직 값

/* 기본 스레드 식별용 난수 값.
   이 값은 변경하면 안 됨. */
#define THREAD_BASIC 0xd42df210 // 기본 스레드 식별용 상수(변경 금지)

/* THREAD_READY 상태, 즉 실행 준비는 되었으나 아직 실행 중은 아닌
   스레드들의 리스트(준비 큐). */
static struct list ready_list; // READY 상태 스레드들이 대기하는 준비 큐

/* 유휴(idle) 스레드. */
static struct thread *idle_thread; // 실행할 게 없을 때만 도는 빈 작업 스레드

/* 초기 스레드: init.c:main()을 실행하는 스레드. */
static struct thread *initial_thread; // 최초(main) 스레드 포인터. 부팅 직후 메인 스레드

/* allocate_tid()에서 사용하는 락. */
static struct lock tid_lock; // tid 할당 시 동시성 보호용 락

/* 스레드 파괴(해제) 요청 큐 */
static struct list destruction_req; // 파괴(해제) 대기중인 스레드 목록

/* 통계 */
static long long idle_ticks;   /* 유휴 상태에 소비된 틱 수 */
static long long kernel_ticks; /* 커널 스레드가 소비한 틱 수 */
static long long user_ticks;   /* 유저 프로그램이 소비한 틱 수 */

/* 스케줄링 */
#define TIME_SLICE 4          /* 스레드에 할당할 타임 슬라이스(틱 수) */
static unsigned thread_ticks; /* 마지막 양보(yield) 후 경과한 틱 수 */

/* false(기본)이면 라운드로빈 스케줄러 사용.
   true면 다단계 피드백 큐(MLFQS) 사용.
   커널 커맨드라인 옵션 "-o mlfqs"로 제어됨. */
bool thread_mlfqs; // true면 MLFQS 스케줄러 사용(명령행 옵션)

static void kernel_thread(thread_func *, void *aux); // 커널 스레드 진입 래퍼

static void idle(void *aux UNUSED);                                       // 유휴 스레드 본체
static struct thread *next_thread_to_run(void);                           // 다음 실행 스레드 선택
static void init_thread(struct thread *, const char *name, int priority); // 스레드 구조 초기화
static void do_schedule(int status);                                      // 상태 설정 후 스케줄링 진입
static void schedule(void);                                               // 컨텍스트 스위칭 드라이버
static tid_t allocate_tid(void);                                          // 고유 tid 할당
bool compare_thread_priority(const struct list_elem *a, const struct list_elem *b, void *aux UNUSED);
void donate_priority_chain(struct thread *donee);

/* T가 유효한 스레드를 가리키는지 여부 반환. */
// 스택 구조체의 맨 마지막인 magic이 변했는지를 검증.
// 기본 설정인 THREAD_MAGIC과 같다면 스레드 구조체(헤드 부분)가 오염되지 않은 것임
// 이 스레드의 magic이 오염되었다는 것은 커널 스택이 과도하게 커져(아래 주소가지 내려와 오염) 스레드 구조체를
// 오염시켰다는 뜻이 됨.
#define is_thread(t) ((t) != NULL && (t)->magic == THREAD_MAGIC) // 유효 스레드 검증 매크로

/* 현재 실행 중인 스레드를 반환.
 * CPU의 스택 포인터 rsp를 읽어 페이지 시작 주소로 내린다.
 * struct thread는 항상 페이지 시작에, 스택 포인터는 페이지 중간에 있으므로
 * 현재 스레드를 찾을 수 있다. */
// 실행 중인 스레드는 자신의 커널 스택을 사용하므로 rsp 값은 “해당 스레드의 4KB 페이지 내부 어딘가”에 위치.
// 따라서 rsp를 페이지 경계(4KB 배수)로 내리면 그 페이지의 시작 주소가 되고, 바로 거기에 struct thread가 있으므로 현재
// 스레드 구조체의 베이스 주소가 됨
// 이 매크로의 의미와 동작
// 코드: #define running_thread() ((struct thread *)(pg_round_down(rrsp())))
// rrsp(): 현재 CPU의 rsp 값을 읽어오는 헬퍼(인라인 어셈블리).
// pg_round_down(x): 주소 x를 페이지 크기(4KB) 경계로 내림(하위 12비트 0으로 만듦).
// 캐스팅: 그 페이지 시작 주소를 struct thread*로 해석.
// 결과: 지금 CPU에서 실행 중인 “현재 스레드”의 struct thread*를 O(1)로 획득.
#define running_thread()                                                                                               \
    ((struct thread *)(pg_round_down(rrsp()))) // 현재 rsp가 속한 페이지 시작 = 현재 스레드 구조 주소

// thread_start를 위한 전역 디스크립터 테이블(GDT).
// gdt는 thread_init 이후에 최종 설정되므로, 우선 임시 GDT를 세팅한다.
// GDT = 운영체제에서 메모리 구역 사용 규칙을 정의한 표
// 임시 GDT를 사용하는 이유?
// -> Pintos가 처음 실행될 때 아직 운영체제가 제대로 부팅되지 않음.
// 정식 GDT를 세팅하기 전이므로 간단한 임시 GDT를 만들어두는 것
// -> 운영체제 초기화 단계에서 잠깐 쓰는 기본 설정용 테이블임.
// 쉽게 64비트에선 커널 모드인지 유저 모드인지 권한을 구분한다고 생각하면 됨
static uint64_t gdt[3] = {0, 0x00af9a000000ffff, 0x00cf92000000ffff}; // 임시 GDT (커널 코드/데이터)

/* 현재 실행 중인 코드를 스레드로 승격하여 스레드 시스템을 초기화한다.
   일반적으로는 불가능하지만, loader.S가 스택 하단을 페이지 경계에 맞춰 두었기 때문에 가능하다.

   이 때 준비 큐와 tid 락도 초기화한다.

   이 함수를 호출한 뒤 thread_create()로 스레드를 만들기 전, 반드시 페이지 할당기를 초기화해야 한다.

   이 함수가 종료되기 전에는 thread_current()를 호출하면 안 된다. */
// 스레드 시스템 “전역” 초기화. 임시 GDT 로드, 준비 큐/파괴 큐/락 초기화, 현재 실행 중 코드를 “초기 스레드”로 등록.
void thread_init(void) {
    ASSERT(intr_get_level() == INTR_OFF); // 초기화 중에는 인터럽트 금지

    /* 커널을 위한 임시 GDT를 로드한다.
     * 이 GDT에는 사용자 컨텍스트가 포함되어 있지 않다.
     * 사용자 컨텍스트를 포함한 GDT는 gdt_init()에서 다시 구성된다. */
    struct desc_ptr gdt_ds = {
        // GDT 디스크립터 구성
        .size = sizeof(gdt) - 1, // GDT 바이트 크기 - 1
        .address = (uint64_t)gdt // GDT 베이스 주소
    };
    lgdt(&gdt_ds); // 임시 GDT 로드

    /* 전역 스레드 컨텍스트 초기화 */
    lock_init(&tid_lock);        // tid 락 초기화
    list_init(&ready_list);      // 준비 큐 초기화
    list_init(&destruction_req); // 파괴 요청 리스트 초기화

    /* 현재 실행 중인 코드를 위한 스레드 구조체 설정 */
    initial_thread = running_thread();                // 재 스택의 페이지 시작을 struct thread*로 캐스팅
    init_thread(initial_thread, "main", PRI_DEFAULT); // 이름/우선순위 설정
    initial_thread->status = THREAD_RUNNING;          // 현재 실행 중 표시
    initial_thread->tid = allocate_tid();             // tid 부여
}

/* 인터럽트를 허용하여 선점형 스케줄링을 시작한다.
   또한 idle 스레드를 생성한다. */
void thread_start(void) {
    /* idle 스레드 생성 및 초기화 완료까지 대기할 세마포어 준비 */
    struct semaphore idle_started;                       // idle 스레드 준비 신호용
    sema_init(&idle_started, 0);                         // 0으로 초기화(대기 상태)
    thread_create("idle", PRI_MIN, idle, &idle_started); // idle 스레드 생성

    /* 선점형 스케줄링 시작 */
    intr_enable(); // 선점형 스케줄링 시작(인터럽트 허용)

    /* idle_thread 초기화 완료까지 대기 */
    sema_down(&idle_started); // idle 스레드 준비 완료까지 대기
}

/* 매 타이머 틱마다 타이머 인터럽트 핸들러에 의해 호출된다.
   외부 인터럽트 컨텍스트에서 실행된다. */
void thread_tick(void) {
    struct thread *t = thread_current(); // 현재 실행 중 스레드

    /* 통계 갱신 */
    if (t == idle_thread) // 유휴 스레드면 idle 통계 증가
        idle_ticks++;
#ifdef USERPROG
    else if (t->pml4 != NULL) // 유저 주소공간이 있으면 user 통계 증가
        user_ticks++;
#endif
    else // 그 외는 커널 스레드
        kernel_ticks++;

    /* 선점 강제 */
    if (++thread_ticks >= TIME_SLICE) // 타임 슬라이스 소진 시
        intr_yield_on_return();       // 인터럽트 리턴 직후 양보
}

/* 스레드 통계 출력 */
void thread_print_stats(void) {
    printf("Thread: %lld idle ticks, %lld kernel ticks, %lld user ticks\n", idle_ticks, kernel_ticks,
           user_ticks); // 누적 통계 출력
}

/* 이름 NAME, 초기 우선순위 PRIORITY로 새로운 커널 스레드를 생성한다.
   이 스레드는 FUNCTION(AUX)를 실행하며, 준비 큐에 추가된다.
   성공 시 새 스레드의 tid를, 실패 시 TID_ERROR를 반환한다.

   thread_start()가 이미 호출된 상태라면, thread_create()가 반환되기 전에
   새 스레드가 스케줄될 수 있으며, 극단적으로는 반환 전에 종료될 수도 있다.
   반대로 원래 스레드가 새 스레드가 스케줄되기 전까지 계속 실행될 수도 있다.
   순서를 보장하려면 세마포어나 다른 동기화 수단을 사용하라.

   제공된 코드는 새 스레드의 priority 필드만 설정할 뿐 실제 우선순위 스케줄링은 구현되어 있지 않다.
   우선순위 스케줄링은 프로젝트 과제의 목표다. */
tid_t thread_create(const char *name, int priority, thread_func *function, void *aux) {
    struct thread *t; // 새 스레드 구조체 포인터
    tid_t tid;        // 새 스레드의 tid

    ASSERT(function != NULL); // 진입 함수는 필수

    /* 스레드 구조체 페이지 할당 */
    t = palloc_get_page(PAL_ZERO); // 1페이지를 0으로 초기화하여 할당
    if (t == NULL)                 // 메모리 부족 시 실패
        return TID_ERROR;

    /* 스레드 필드 초기화 */
    init_thread(t, name, priority); // 기본 필드 초기화
    tid = t->tid = allocate_tid();  // tid 할당 및 설정

    /* 스케줄되면 kernel_thread로 진입하도록 초기 레지스터를 설정한다.
     * 참고) rdi는 1번째 인자, rsi는 2번째 인자. */
    t->tf.rip = (uintptr_t)kernel_thread; // 커널 스레드 래퍼 진입 주소 설정
    t->tf.R.rdi = (uint64_t)function;     // 1번째 인자: 스레드 함수 포인터
    t->tf.R.rsi = (uint64_t)aux;          // 2번째 인자: 사용자 인자
    t->tf.ds = SEL_KDSEG;                 // 데이터 세그먼트 셀렉터
    t->tf.es = SEL_KDSEG;                 // ES 세그먼트
    t->tf.ss = SEL_KDSEG;                 // 스택 세그먼트
    t->tf.cs = SEL_KCSEG;                 // 코드 세그먼트
    t->tf.eflags = FLAG_IF;               // 인터럽트 허용 플래그 설정

    /* 준비 큐에 추가 */
    thread_unblock(t); // 준비 큐에 넣어 READY 상태로 전이

    // 새 스레드가 우선순위 높으면 현재 스레드를 양보
    if (t->priority > thread_current()->priority) {
        thread_yield();
    }

    return tid; // 새 스레드의 tid 반환
}

/* 현재 스레드를 수면(BLOCKED) 상태로 만든다. thread_unblock()으로 깨울 때까지
   스케줄되지 않는다.

   이 함수는 반드시 인터럽트 비활성 상태에서 호출해야 한다.
   보통은 synch.h의 동기화 원시를 사용하는 것이 더 안전하다. */
void thread_block(void) {
    ASSERT(!intr_context());                   // 인터럽트 컨텍스트에서 호출 금지
    ASSERT(intr_get_level() == INTR_OFF);      // 호출 전 인터럽트 비활성 필요
    thread_current()->status = THREAD_BLOCKED; // 현재 스레드를 BLOCKED로
    schedule();                                // 스케줄러로 전환
}

/* BLOCKED 상태의 스레드 T를 READY 상태로 전이시킨다.
   T가 BLOCKED가 아니라면 오류이다. (실행 중인 스레드를 READY로 만들려면 thread_yield() 사용)

   이 함수는 현재 실행 중인 스레드를 선점하지 않는다. 호출자가 인터럽트를 직접 비활성화한 경우,
   스레드를 원자적으로 깨우고 다른 데이터를 갱신하기를 기대할 수 있기 때문이다. */
void thread_unblock(struct thread *t) {
    enum intr_level old_level;  // 인터럽트 상태 저장용
    old_level = intr_disable(); // 원자적 조작 위해 인터럽트 비활성

    ASSERT(is_thread(t)); // 유효 스레드 확인
    ASSERT(t->status == THREAD_BLOCKED);

    // READY 큐에 우선순위 정렬 삽입 방식으로 변경
    /**
     * READY 큐는 “우선순위 내림차순”으로 관리
     * 인터럽트가 아닌 상황에서는 우선순위가 더 높은 스레드가 준비되면 즉시 선점
     * 인터럽트 중에는 intr_yield_on_return()으로 “리턴 직전 양보” 플래그만 설정
     */
    list_insert_ordered(&ready_list, &t->elem, compare_thread_priority, NULL);
    t->status = THREAD_READY; // 이제 스케줄 대상임
    // 현재 컨텍스트에 따라 선점 요청
    if (intr_context()) {
        // 인터럽트 컨텍스트에서는 리턴 직전 양보만 표시
        intr_yield_on_return();
    } else {
        // 일반 컨텍스트에서는 우선순위가 더 높은 스레드가 준비되면 즉시 양보
        if (t->priority > thread_current()->priority) {
            thread_yield();
        }
    }

    // 기본 pintos 큐 방식(Round Robin 방식)
    // ASSERT(t->status == THREAD_BLOCKED);   // BLOCKED 상태여야 함
    // list_push_back(&ready_list, &t->elem); // 준비 큐 꼬리에 삽입
    // t->status = THREAD_READY;              // READY 상태로 변경
    intr_set_level(old_level); // 인터럽트 복원
}

/* 현재 실행 중인 스레드의 이름 반환 */
const char *thread_name(void) {
    return thread_current()->name; // 현재 스레드의 이름 반환
}

/* 현재 실행 중인 스레드 반환.
   running_thread() 결과에 대해 몇 가지 유효성 검사를 추가로 수행한다.
   자세한 내용은 thread.h 상단 주석 참조. */
struct thread *thread_current(void) {
    struct thread *t = running_thread(); // rsp 기반으로 현재 스레드 얻기

    /* Make sure T is really a thread.
       If either of these assertions fire, then your thread may
       have overflowed its stack.  Each thread has less than 4 kB
       of stack, so a few big automatic arrays or moderate
       recursion can cause stack overflow. */
    ASSERT(is_thread(t));                // 매직 체크로 검증
    ASSERT(t->status == THREAD_RUNNING); // 반드시 RUNNING 상태여야 함

    return t; // 현재 스레드 포인터 반환
}

/* 현재 실행 중인 스레드의 tid 반환 */
tid_t thread_tid(void) {
    return thread_current()->tid; // 현재 스레드의 tid 반환
}

/* 현재 스레드를 스케줄에서 제거하고 파괴한다. 호출자로 반환하지 않는다. */
void thread_exit(void) {
    ASSERT(!intr_context()); // 인터럽트 컨텍스트에서 종료 금지

#ifdef USERPROG
    process_exit(); // 유저 프로세스 자원 정리
#endif

    /* 상태를 DYING으로 설정한 뒤 다른 스레드를 스케줄한다.
       실제 파괴는 schedule_tail() 경로에서 수행된다. */
    intr_disable();            // 스케줄링 전 인터럽트 비활성
    do_schedule(THREAD_DYING); // 상태를 DYING으로 바꾸고 전환
    NOT_REACHED();             // 여기 도달하면 오류
}

/* CPU를 양보한다. 현재 스레드는 수면 상태로 가지 않으며,
   스케줄러의 판단에 따라 즉시 다시 스케줄될 수 있다. */
void thread_yield(void) {
    struct thread *curr = thread_current(); // 현재 스레드
    enum intr_level old_level;              // 인터럽트 플래그 백업

    ASSERT(!intr_context());

    old_level = intr_disable(); // 원자적 큐 조작 준비
    if (curr != idle_thread)    // idle이 아니면
                                // 정렬 삽입으로 변경
        list_insert_ordered(&ready_list, &curr->elem, compare_thread_priority, NULL);
    do_schedule(THREAD_READY); // READY로 전환하고 스케줄링
    intr_set_level(old_level); // 인터럽트 복원
}

/* 현재 스레드의 우선순위를 NEW_PRIORITY로 설정 */
void thread_set_priority(int new_priority) {
    struct thread *cur = thread_current();
    cur->base_priority = new_priority;

    // 현재 donation 영향이 없다면 즉시 반영, 있으면 refresh가 올려줌
    refresh_priority(cur);

    // 내가 낮아졌고, 레디에 나보다 높은 스레드가 있으면 양보
    enum intr_level old = intr_disable();
    if (!list_empty(&ready_list)) {
        struct thread *top = list_entry(list_front(&ready_list), struct thread, elem);
        if (top->priority > cur->priority) {
            thread_yield();
        }
    }
    intr_set_level(old);
}

/* 현재 스레드의 우선순위 반환 */
int thread_get_priority(void) {
    return thread_current()->priority; // 현재 스레드 우선순위 반환
}

/* 현재 스레드의 nice 값을 NICE로 설정(MLFQS 관련) */
void thread_set_nice(int nice UNUSED) { /* TODO: mlfqs nice 값 설정(프로젝트 확장 과제) */ }

/* 현재 스레드의 nice 값 반환 */
int thread_get_nice(void) {
    /* TODO: mlfqs nice 값 조회 */
    return 0;
}

/* 시스템 load average * 100 반환 */
int thread_get_load_avg(void) {
    /* TODO: 시스템 load average*100 반환 */
    return 0;
}

/* 현재 스레드의 recent_cpu * 100 반환 */
int thread_get_recent_cpu(void) {
    /* TODO: 현재 스레드의 recent_cpu*100 반환 */
    return 0;
}

/* 유휴 스레드. 다른 실행 가능한 스레드가 없을 때 실행된다.

   idle 스레드는 thread_start()에 의해 준비 큐에 들어가며, 처음 한 번 스케줄되어
   idle_thread 전역을 초기화하고, 전달받은 세마포어를 up하여 thread_start()가 진행되도록 한 뒤
   즉시 BLOCKED 상태로 전환된다. 그 이후로 idle 스레드는 준비 큐에 나타나지 않으며,
   준비 큐가 비었을 때 next_thread_to_run()에서 특수하게 반환된다. */
static void idle(void *idle_started_ UNUSED) {
    struct semaphore *idle_started = idle_started_; // 초기화 완료 신호용 세마포어

    idle_thread = thread_current(); // 전역 idle_thread 설정
    sema_up(idle_started);          // thread_start()에 준비 완료 알림

    for (;;) {
        /* 다른 스레드가 실행될 수 있도록 함 */
        intr_disable(); // 다음 인터럽트 전까지 대기 준비
        thread_block(); // 스스로 BLOCKED로 전환

        /* 인터럽트를 다시 허용하고 다음 인터럽트를 대기한다.

           sti 명령은 다음 명령이 끝날 때까지 인터럽트를 비활성화하므로,
           아래 두 명령은 원자적으로 실행된다. 이 원자성 덕분에
           인터럽트를 허용하자마자 다음 HLT 전에 인터럽트가 처리되어
           한 틱을 낭비하는 일을 방지한다.

           관련 문서: [IA32-v2a] HLT, [IA32-v2b] STI, [IA32-v3a] 7.11.1 HLT Instruction */
        asm volatile("sti; hlt" : : : "memory"); // 인터럽트 허용 후 HLT(저전력 대기)
    }
}

/* 커널 스레드의 진입 래퍼 함수. */
static void kernel_thread(thread_func *function, void *aux) {
    ASSERT(function != NULL); // 유효한 함수 포인터 확인

    intr_enable(); /* 스레드 본체 실행 전 인터럽트 허용 */
    function(aux); /* 실제 스레드 함수 호출 */
    thread_exit(); /* 함수가 리턴되면 종료 처리 */
}

/* 이름 NAME을 가진 BLOCKED 상태 스레드 T의 기본 초기화 수행. */
// 새 스레드(struct thread)를 만들 때마다 호출되는 필드 초기화 루틴.
static void init_thread(struct thread *t, const char *name, int priority) {
    ASSERT(t != NULL);                                  // 널 포인터 금지
    ASSERT(PRI_MIN <= priority && priority <= PRI_MAX); // 우선순위 범위 체크
    ASSERT(name != NULL);                               // 이름은 필수

    memset(t, 0, sizeof *t);                           // 구조체 전체 0으로 초기화
    t->status = THREAD_BLOCKED;                        // 초기 상태는 BLOCKED
    strlcpy(t->name, name, sizeof t->name);            // 이름 설정(버퍼 크기 안전)
    t->tf.rsp = (uint64_t)t + PGSIZE - sizeof(void *); // 초기 스택 포인터 설정(페이지 끝)
    t->priority = priority;                            // 우선순위 설정(유효 우선순위)
    t->base_priority = priority;                       // 원래(base) 우선순위
    list_init(&t->donations);                          // 기부자 리스트 초기화
    // list_init(&t->donation_elem);
    list_init(&t->held_locks); // 보유 락 리스트 초기화
    t->wait_on_lock = NULL;    // 대기 중인 락 없음
    t->magic = THREAD_MAGIC;   // 매직 값 설정(오버플로 감지)
}

/* 다음에 스케줄할 스레드를 선택하여 반환한다. 준비 큐가 비어 있지 않다면
   준비 큐에서 하나를 반환하고, 준비 큐가 비었다면 idle_thread를 반환한다. */
static struct thread *next_thread_to_run(void) {
    if (list_empty(&ready_list)) // 준비 큐가 비었으면
        return idle_thread;      // idle 스레드 실행
    else
        return list_entry(list_pop_front(&ready_list), struct thread, elem); // 준비 큐 앞에서 하나 꺼냄
}

/* iretq를 사용하여 스레드를 실행 상태로 복귀 */
void do_iret(struct intr_frame *tf) {
    /* 저장된 intr_frame을 레지스터와 세그먼트에 복원하고 iretq로 사용자/커널 컨텍스트 복귀 */
    __asm __volatile("movq %0, %%rsp\n"
                     "movq 0(%%rsp),%%r15\n"
                     "movq 8(%%rsp),%%r14\n"
                     "movq 16(%%rsp),%%r13\n"
                     "movq 24(%%rsp),%%r12\n"
                     "movq 32(%%rsp),%%r11\n"
                     "movq 40(%%rsp),%%r10\n"
                     "movq 48(%%rsp),%%r9\n"
                     "movq 56(%%rsp),%%r8\n"
                     "movq 64(%%rsp),%%rsi\n"
                     "movq 72(%%rsp),%%rdi\n"
                     "movq 80(%%rsp),%%rbp\n"
                     "movq 88(%%rsp),%%rdx\n"
                     "movq 96(%%rsp),%%rcx\n"
                     "movq 104(%%rsp),%%rbx\n"
                     "movq 112(%%rsp),%%rax\n"
                     "addq $120,%%rsp\n"
                     "movw 8(%%rsp),%%ds\n"
                     "movw (%%rsp),%%es\n"
                     "addq $32, %%rsp\n"
                     "iretq"
                     :
                     : "g"((uint64_t)tf)
                     : "memory"); // tf를 스택 프레임으로 사용하여 복귀
}

// 유효 우선순위 갱신
void refresh_priority(struct thread *t) {
    t->priority = t->base_priority;
    if (!list_empty(&t->donations)) {
        // donations 리스트는 우선순위 내림차순 정렬
        struct thread *top = list_entry(list_front(&t->donations), struct thread, donation_elem);
        if (top->priority > t->priority) {
            t->priority = top->priority;
        }
    }
}

static bool compare_donation_prio(const struct list_elem *a, const struct list_elem *b, void *aux UNUSED) {
    const struct thread *da = list_entry(a, struct thread, donation_elem);
    const struct thread *db = list_entry(b, struct thread, donation_elem);
    return da->priority > db->priority;
}

// 우선순위 기부를 체인으로 전파 - Nested Donation 방식
// 현재 스레드(cur)가 어떤 락을 기다리는 중이고, 그 락의 소유자(donee)가 나보다 우선순위가 낮다면 donee의 우선순위를
// 끌어올리고, donee가 또 다른 락을 기다리고 있으면 그 락의 소유자에게도 계속(최대 깊이까지) 기부를 전파한다
void donate_priority_chain(struct thread *donee) {
    int depth = 0;                          // 최대 깊이 전파 제한 카운터 (무한루프 방지용)
    struct thread *cur = thread_current();  // 기부를 시작하는 현재 스레드(최초 donor)
    int donated_pri = cur->priority;        // 전파할 우선순위(체인 진행 시 갱신)

    while (donee && depth++ < DONATION_DEPTH_LIMIT) {
        // 최초 단계에서는 현재 스레드를 donee의 donations에 등록(중복 제거 후 삽입)
        if (depth == 1) {
            struct list_elem *e;
            for (e = list_begin(&donee->donations); e != list_end(&donee->donations);) {
                struct thread *d = list_entry(e, struct thread, donation_elem);
                if (d == cur) {
                    e = list_remove(e);
                    break;
                } else {
                    e = list_next(e);
                }
            }
            list_insert_ordered(&donee->donations, &cur->donation_elem, compare_donation_prio, NULL);
        }

        // donee의 유효 우선순위를 새로 계산한 뒤, 필요한 경우 donated_pri까지 끌어올림
        refresh_priority(donee);
        if (donee->priority < donated_pri) {
            donee->priority = donated_pri;
        }

        // 다음 전파 대상 탐색: donee가 또 다른 락을 기다리는 중이면 그 락의 holder로 전파
        if (donee->wait_on_lock && donee->wait_on_lock->holder != donee) {
            // 다음 단계에서 전파할 우선순위는 방금 상향된 donee의 우선순위
            donated_pri = donee->priority;
            donee = donee->wait_on_lock->holder;
        } else {
            break;
        }
    }
}

/* 새 스레드의 페이지 테이블을 활성화하여 스레드를 전환하고,
   이전 스레드가 DYING 상태라면 파괴한다.

   이 함수가 호출될 때는 이미 이전 스레드에서 전환된 상태이며,
   새 스레드가 실행 중이고 인터럽트는 여전히 비활성이다.

   스레드 전환이 완료되기 전까지는 printf() 호출이 안전하지 않다. */
static void thread_launch(struct thread *th) {
    uint64_t tf_cur = (uint64_t)&running_thread()->tf; // 현재 스레드의 intr_frame 주소
    uint64_t tf = (uint64_t)&th->tf;                   // 다음 스레드의 intr_frame 주소
    ASSERT(intr_get_level() == INTR_OFF);              // 전환 중에는 인터럽트 비활성이어야 함

    /* 전환 핵심 로직:
     *  - 현재 실행 문맥을 intr_frame(tf_cur)에 저장한다.
     *  - 다음 스레드의 intr_frame(tf)을 사용하여 do_iret로 전환한다.
     *  - 전환이 끝날 때까지 현재 스택을 사용하면 안 된다. */
    /* 현재 레지스터 상태를 tf_cur에 저장하고, th의 tf를 이용해 do_iret로 전환 */
    __asm __volatile(
        /* 이후 사용할 레지스터를 저장 */
        "push %%rax\n"
        "push %%rbx\n"
        "push %%rcx\n"
        /* 인자로 받은 주소들을 레지스터에 미리 적재 */
        "movq %0, %%rax\n"
        "movq %1, %%rcx\n"
        "movq %%r15, 0(%%rax)\n"
        "movq %%r14, 8(%%rax)\n"
        "movq %%r13, 16(%%rax)\n"
        "movq %%r12, 24(%%rax)\n"
        "movq %%r11, 32(%%rax)\n"
        "movq %%r10, 40(%%rax)\n"
        "movq %%r9, 48(%%rax)\n"
        "movq %%r8, 56(%%rax)\n"
        "movq %%rsi, 64(%%rax)\n"
        "movq %%rdi, 72(%%rax)\n"
        "movq %%rbp, 80(%%rax)\n"
        "movq %%rdx, 88(%%rax)\n"
        "pop %%rbx\n" // 저장해 두었던 rcx 복원 값을 tf_cur에 기록
        "movq %%rbx, 96(%%rax)\n"
        "pop %%rbx\n" // 저장해 두었던 rbx 복원 값을 tf_cur에 기록
        "movq %%rbx, 104(%%rax)\n"
        "pop %%rbx\n" // 저장해 두었던 rax 복원 값을 tf_cur에 기록
        "movq %%rbx, 112(%%rax)\n"
        "addq $120, %%rax\n"
        "movw %%es, (%%rax)\n"
        "movw %%ds, 8(%%rax)\n"
        "addq $32, %%rax\n"
        "call __next\n" // 현재 rip을 얻기 위한 더미 호출
        "__next:\n"
        "pop %%rbx\n"
        "addq $(out_iret -  __next), %%rbx\n"
        "movq %%rbx, 0(%%rax)\n" // rip 기록
        "movw %%cs, 8(%%rax)\n"  // cs 기록
        "pushfq\n"
        "popq %%rbx\n"
        "mov %%rbx, 16(%%rax)\n" // eflags 기록
        "mov %%rsp, 24(%%rax)\n" // rsp 기록
        "movw %%ss, 32(%%rax)\n"
        "mov %%rcx, %%rdi\n"
        "call do_iret\n"
        "out_iret:\n"
        :
        : "g"(tf_cur), "g"(tf)
        : "memory");
}

/* 새 스레드를 스케줄한다. 진입 시 인터럽트는 꺼져 있어야 한다.
 * 현재 스레드의 상태를 주어진 status로 바꾸고, 다음에 실행할 스레드를 찾아 전환한다.
 * schedule() 내부에서는 printf()를 호출하면 안전하지 않다. */
static void do_schedule(int status) {
    ASSERT(intr_get_level() == INTR_OFF);               // 스케줄링 중 인터럽트 금지
    ASSERT(thread_current()->status == THREAD_RUNNING); // 호출 전 RUNNING 상태
    while (!list_empty(&destruction_req)) {             // 지연된 해제 요청 처리
        struct thread *victim = list_entry(list_pop_front(&destruction_req), struct thread, elem); // 하나 꺼내고
        palloc_free_page(victim); // 페이지 해제(스택 포함)
    }
    thread_current()->status = status; // 현재 스레드 상태 갱신
    schedule();                        // 다음 스레드로 전환
}

static void schedule(void) {
    struct thread *curr = running_thread();     // 현재 스레드
    struct thread *next = next_thread_to_run(); // 다음 실행할 스레드 선택

    ASSERT(intr_get_level() == INTR_OFF);   // 전환 중 인터럽트 금지
    ASSERT(curr->status != THREAD_RUNNING); // 현재 스레드는 RUNNING이 아님
    ASSERT(is_thread(next));                // next 유효성 확인
    /* 다음 스레드를 RUNNING으로 표시 */
    next->status = THREAD_RUNNING; // 다음 스레드 RUNNING 표기

    /* 새 타임 슬라이스 시작 */
    thread_ticks = 0; // 타임 슬라이스 리셋

#ifdef USERPROG
    /* 새로운 주소 공간 활성화(유저 프로세스 문맥 전환) */
    process_activate(next);
#endif

    if (curr != next) { // 자신과 다를 때만 실제 전환
        /* 이전 스레드가 DYING 상태라면 그 스레드의 struct thread를 파괴해야 한다.
           이는 thread_exit()가 자신의 발밑을 걷어차지 않도록 함수 말미에 수행되어야 한다.
           현재 스택이 사용 중이므로 여기서는 페이지 해제 요청만 큐에 넣고,
           실제 파괴는 schedule()의 시작부에서 처리한다. */
        if (curr && curr->status == THREAD_DYING && curr != initial_thread) {
            ASSERT(curr != next);                          // 자신을 곧 전환할 next가 아님을 보장
            list_push_back(&destruction_req, &curr->elem); // 스택 사용 중이므로 나중에 해제
        }

        /* 스레드를 전환하기 전에, 우선 현재 실행 중인 문맥 정보를 저장한다. */
        thread_launch(next); // 레지스터/세그먼트 복원 후 iret 전환
    }
}

/* 새 스레드에 사용할 tid를 반환 */
static tid_t allocate_tid(void) {
    static tid_t next_tid = 1; // 전역 증가형 tid 시퀀스
    tid_t tid;                 // 반환할 tid

    lock_acquire(&tid_lock); // 원자적 증가를 위해 락 획득
    tid = next_tid++;        // 현재 값을 반환하고 1 증가
    lock_release(&tid_lock); // 락 해제

    return tid; // 새 tid 반환
}

// 스레드 우선순위 비교자 - args : 리스트 노드 포인터 a, b를 받아옴
bool compare_thread_priority(const struct list_elem *a, const struct list_elem *b, void *aux UNUSED) {
    // a, b가 속한 스레드 구조체를 복원
    const struct thread *ta = list_entry(a, struct thread, elem);
    const struct thread *tb = list_entry(b, struct thread, elem);
    // a가 더 크면 true -> 우선순위가 높은 스레드를 리스트의 앞쪽에 오게 한다.
    return ta->priority > tb->priority;
}
