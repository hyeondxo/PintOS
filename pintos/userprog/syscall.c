#include "userprog/syscall.h"
#include "intrinsic.h"
#include "threads/flags.h"
#include "threads/interrupt.h"
#include "threads/loader.h"
#include "threads/thread.h"
#include "userprog/gdt.h"
#include <stdio.h>
#include <syscall-nr.h>

void syscall_entry(void);                  // 어셈블리 레벨에서 정의된 syscall 진입점 (intrinsic.asm 등)
void syscall_handler(struct intr_frame *); // 실제 C 레벨에서 시스템 콜을 처리하는 함수

/* 시스템 콜.
 *
 * 과거에는 시스템 콜 서비스가 인터럽트 핸들러(예: 리눅스의 int 0x80)에 의해 처리되었습니다.
 * 그러나 x86-64에서는 제조사가 시스템 콜을 요청하기 위한 더 효율적인 경로인
 * `syscall` 명령어를 제공합니다.
 *
 * syscall 명령어는 모델별 레지스터(MSR: Model Specific Register)의 값을 읽어 동작합니다.
 * 자세한 내용은 매뉴얼을 참고하십시오. */

#define MSR_STAR 0xc0000081         /* 세그먼트 선택자용 MSR (SYSCALL 시 유저/커널 코드 세그먼트 설정) */
#define MSR_LSTAR 0xc0000082        /* 롱 모드에서의 SYSCALL 진입 지점 주소를 저장하는 MSR */
#define MSR_SYSCALL_MASK 0xc0000084 /* SYSCALL 시 EFLAGS에서 마스킹할 비트들을 저장하는 MSR */

void syscall_init(void) {
    /* MSR_STAR:
     *   상위 16비트에 "유저 모드 코드 세그먼트 - 0x10" 값이,
     *   하위 16비트에 "커널 모드 코드 세그먼트" 값이 기록됩니다.
     *   → SYSCALL 실행 시 CPU가 적절한 세그먼트로 전환할 수 있게 해줍니다.
     */
    write_msr(MSR_STAR, ((uint64_t)SEL_UCSEG - 0x10) << 48 | ((uint64_t)SEL_KCSEG) << 32);

    /* MSR_LSTAR:
     *   SYSCALL 실행 시 점프할 "진입 함수 주소"를 지정합니다.
     *   여기서는 어셈블리로 작성된 syscall_entry()로 연결됩니다.
     */
    write_msr(MSR_LSTAR, (uint64_t)syscall_entry);

    /* MSR_SYSCALL_MASK:
     *   SYSCALL 직후에는 아직 사용자 스택을 커널 스택으로 교체하기 전이므로
     *   CPU가 인터럽트나 트랩을 받아들이면 위험합니다.
     *   따라서 EFLAGS의 특정 비트(인터럽트 플래그 등)를 마스킹하여
     *   커널 진입이 안정적으로 완료될 때까지 차단합니다.
     */
    write_msr(MSR_SYSCALL_MASK, FLAG_IF | FLAG_TF | FLAG_DF | FLAG_IOPL | FLAG_AC | FLAG_NT);
}

/* 메인 시스템 콜 인터페이스
 *
 * syscall_entry(어셈블리 루틴)에서 유저 모드 → 커널 모드 전환이 끝나면,
 * 이 함수가 호출됩니다.
 *
 * - struct intr_frame *f:
 *     SYSCALL 직전의 유저 레지스터 상태를 담고 있는 구조체입니다.
 *     여기서 시스템 콜 번호(RAX)와 인자들(RDI, RSI, RDX, R10, R8, R9)을 꺼내
 *     알맞은 시스템 콜 함수를 호출해야 합니다.
 *
 * - 구현해야 할 기능:
 *     1. RAX에서 syscall 번호 추출
 *     2. 인자 값 검증 (특히 포인터가 유저 메모리인지 확인)
 *     3. 해당 syscall 수행 (예: read, write, exit, exec, wait 등)
 *     4. 반환값을 다시 RAX에 저장하여 유저 모드로 복귀 시 전달
 *
 * 현재는 단순히 "system call!"을 출력하고 thread_exit()으로 종료하는 더미 코드입니다.
 */
void syscall_handler(struct intr_frame *f UNUSED) {
    // TODO: 여기에 실제 시스템 콜 구현을 작성해야 합니다.
    printf("system call!\n");

    // 현재는 테스트용으로 시스템 콜을 호출한 프로세스를 바로 종료합니다.
    thread_exit();
}