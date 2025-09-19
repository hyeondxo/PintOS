#include "userprog/exception.h"
#include "intrinsic.h"
#include "threads/interrupt.h"
#include "threads/thread.h"
#include "userprog/gdt.h"
#include "userprog/syscall.h"
#include <inttypes.h>
#include <stdio.h>

/* 처리된 페이지 폴트의 개수 */
static long long page_fault_cnt;

static void kill(struct intr_frame *);
static void page_fault(struct intr_frame *);

/* 유저 프로그램에 의해 발생할 수 있는 예외(interrupt/exception) 핸들러를 등록합니다.
 *
 * 실제 유닉스 계열 OS에서는 대부분의 예외가 시그널(signal) 형태로
 * 유저 프로세스에 전달됩니다([SV-386] 3-24, 3-25 참고).
 * 하지만 Pintos에서는 시그널을 구현하지 않으므로
 * 단순히 예외를 발생시킨 유저 프로세스를 종료시킵니다.
 *
 * 페이지 폴트(Page Fault)는 예외적으로 다르게 다루어야 합니다.
 * 지금은 다른 예외처럼 단순 종료 처리하지만,
 * 프로젝트 3(VM)에서는 이를 수정해 가상 메모리를 지원해야 합니다.
 *
 * 각 예외에 대한 설명은 [IA32-v3a] 5.15절
 * "Exception and Interrupt Reference"를 참고하세요. */
void exception_init(void) {
    /* 다음 예외들은 유저 프로그램이 명시적으로 발생시킬 수 있습니다.
       예: INT, INT3, INTO, BOUND 명령어 사용 시.
       따라서 DPL(Descriptor Privilege Level)을 3으로 설정하여
       유저 프로그램에서도 호출 가능하게 합니다. */
    intr_register_int(3, 3, INTR_ON, kill, "#BP Breakpoint Exception");
    intr_register_int(4, 3, INTR_ON, kill, "#OF Overflow Exception");
    intr_register_int(5, 3, INTR_ON, kill, "#BR BOUND Range Exceeded Exception");

    /* 다음 예외들은 DPL=0이므로 유저 프로그램이 INT 명령어로 직접 호출할 수는 없습니다.
       그러나 간접적으로는 발생할 수 있습니다.
       예: #DE는 0으로 나눌 때 발생합니다. */
    intr_register_int(0, 0, INTR_ON, kill, "#DE Divide Error");
    intr_register_int(1, 0, INTR_ON, kill, "#DB Debug Exception");
    intr_register_int(6, 0, INTR_ON, kill, "#UD Invalid Opcode Exception");
    intr_register_int(7, 0, INTR_ON, kill, "#NM Device Not Available Exception");
    intr_register_int(11, 0, INTR_ON, kill, "#NP Segment Not Present");
    intr_register_int(12, 0, INTR_ON, kill, "#SS Stack Fault Exception");
    intr_register_int(13, 0, INTR_ON, kill, "#GP General Protection Exception");
    intr_register_int(16, 0, INTR_ON, kill, "#MF x87 FPU Floating-Point Error");
    intr_register_int(19, 0, INTR_ON, kill, "#XF SIMD Floating-Point Exception");

    /* 대부분의 예외는 인터럽트를 켠 상태에서도 안전하게 처리할 수 있습니다.
       하지만 페이지 폴트는 CR2 레지스터에 fault 주소가 저장되는데,
       인터럽트가 켜져 있으면 다른 fault로 값이 덮어씌워질 수 있습니다.
       따라서 페이지 폴트는 인터럽트를 꺼둔 상태(INTR_OFF)에서 처리해야 합니다. */
    intr_register_int(14, 0, INTR_OFF, page_fault, "#PF Page-Fault Exception");
}

/* 예외 통계 출력 */
void exception_print_stats(void) { printf("Exception: %lld page faults\n", page_fault_cnt); }

/* 유저 프로세스(또는 커널 버그)에 의해 발생한 예외를 처리하는 핸들러 */
static void kill(struct intr_frame *f) {
    /* 이 예외는 보통 유저 프로세스에 의해 발생한 것입니다.
       예: 잘못된 메모리 접근 → 페이지 폴트 발생.
       현재는 단순히 유저 프로세스를 종료시킵니다.
       (실제 OS에서는 시그널을 보냄) */

    /* 예외가 어디서 발생했는지는 코드 세그먼트 값(cs)으로 구분할 수 있습니다. */
    switch (f->cs) {
    case SEL_UCSEG:
        sys_exit(-1); // bad-* 테스트 통과용. 커널 패닉이 아닌 -1로 프로세스 종료
        /* 유저 코드 세그먼트에서 발생 → 유저 예외 → 유저 프로세스 종료 */
        printf("%s: dying due to interrupt %#04llx (%s).\n", thread_name(), f->vec_no, intr_name(f->vec_no));
        intr_dump_frame(f);                 // 예외 당시 레지스터/스택 상태 출력
        thread_current()->exit_status = -1; // 비정상 예외 종료이므로 실패 코드 유지
        thread_exit();                      // 현재 스레드 종료

    case SEL_KCSEG:
        /* 커널 코드 세그먼트에서 발생 → 커널 내부 버그임.
           (페이지 폴트 등은 여기 오면 안 됨)
           따라서 커널 패닉 발생시킴. */
        intr_dump_frame(f);
        PANIC("Kernel bug - unexpected interrupt in kernel");

    default:
        /* 알 수 없는 코드 세그먼트에서 발생 → 이 경우도 비정상.
           따라서 종료 처리. */
        printf("Interrupt %#04llx (%s) in unknown segment %04x\n", f->vec_no, intr_name(f->vec_no), f->cs);
        thread_exit();
    }
}

/* 페이지 폴트 핸들러.
 * 현재는 스켈레톤이며, Project 3(가상 메모리)에서 확장해야 합니다.
 *
 * - 잘못 접근한 가상 주소는 CR2 레지스터에서 읽습니다.
 * - fault 원인 정보(error_code)는 intr_frame의 error_code에 담겨 있습니다.
 *   (PF_P, PF_W, PF_U 매크로 참고)
 *
 * 실제 OS에서는:
 * - not_present이면: 해당 페이지를 디스크에서 로드하거나, 스택을 확장해야 할 수 있음.
 * - rights violation이면: 잘못된 접근(예: read-only 페이지에 write) → 보통 kill.
 */
static void page_fault(struct intr_frame *f) {
    bool not_present; /* true: 매핑이 없는 페이지 접근, false: 권한 위반 */
    bool write;       /* true: 쓰기 접근, false: 읽기 접근 */
    bool user;        /* true: 유저 모드 접근, false: 커널 모드 접근 */
    void *fault_addr; /* 문제를 일으킨 가상 주소 */

    /* fault가 발생한 주소를 CR2에서 읽음.
       (실제로 잘못 접근한 데이터/코드의 주소일 수도 있고,
       단순히 잘못된 포인터일 수도 있음.) */
    fault_addr = (void *)rcr2();

    /* 인터럽트 다시 켬.
       (처음엔 CR2 값 보존을 위해 꺼뒀음) */
    intr_enable();

    /* fault 원인을 error_code 비트에서 해석 */
    not_present = (f->error_code & PF_P) == 0; // P=0 → 페이지 없음
    write = (f->error_code & PF_W) != 0;       // W=1 → 쓰기 접근
    user = (f->error_code & PF_U) != 0;        // U=1 → 유저 모드 접근

#ifdef VM
    /* Project 3: 가상 메모리에서 실제로 fault를 처리하는 부분.
       lazy loading, stack growth 등을 여기서 해결해야 함. */
    if (vm_try_handle_fault(f, fault_addr, user, write, not_present))
        return;
#endif

    /* 페이지 폴트 횟수 증가 */
    page_fault_cnt++;

    /* 현재는 단순히 정보 출력 후 프로세스를 kill */
    printf("Page fault at %p: %s error %s page in %s context.\n", fault_addr,
           not_present ? "not present" : "rights violation", write ? "writing" : "reading", user ? "user" : "kernel");
    kill(f);
}
