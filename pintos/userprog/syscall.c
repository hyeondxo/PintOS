#include "userprog/syscall.h"
#include "devices/input.h"
#include "intrinsic.h"
#include <stddef.h>
#include "lib/kernel/stdio.h"
#include "threads/flags.h"
#include "threads/init.h"
#include "threads/interrupt.h"
#include "threads/loader.h"
#include "threads/mmu.h"
#include "threads/thread.h"
#include "threads/vaddr.h"
#include "userprog/gdt.h"
#include <debug.h>
#include <stdbool.h>
#include <stdint.h>
#include <syscall-nr.h>

void syscall_entry(void);                  // 어셈블리 레벨에서 정의된 syscall 진입점 (intrinsic.asm 등)
void syscall_handler(struct intr_frame *); // 실제 C 레벨에서 시스템 콜을 처리하는 함수

#define STDIN_FILENO 0
#define STDOUT_FILENO 1
#define IO_CHUNK 256

static void die_bad_access(void) NO_RETURN;                     // 잘못된 유저 주소 접근 시 즉시 프로세스 종료
static bool get_user_u8(uint8_t *dst, const uint8_t *uaddr);    // 단일 바이트 안전 읽기
static bool put_user_u8(uint8_t *uaddr, uint8_t value);         // 단일 바이트 안전 쓰기
static void copy_in(void *dst, const void *usrc, size_t size);  // 사용자->커널 버퍼 복사
static void copy_out(void *udst, const void *src, size_t size); // 커널->사용자 버퍼 복사

static void sys_halt(void) NO_RETURN;
static void sys_exit(int status) NO_RETURN;
static int sys_write(int fd, const void *ubuf, size_t size);
static int sys_read(int fd, void *ubuf, size_t size);

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
 */
void syscall_handler(struct intr_frame *f) {
    uint64_t syscall_no = f->R.rax; // 시스템 콜 번호는 RAX에 위치
    uint64_t arg1 = f->R.rdi;       // 첫 번째 인자: RDI
    uint64_t arg2 = f->R.rsi;       // 두 번째 인자: RSI
    uint64_t arg3 = f->R.rdx;       // 세 번째 인자: RDX

    switch (syscall_no) {
    case SYS_HALT: // 커널 전원을 즉시 종료
        sys_halt();
        NOT_REACHED();

    case SYS_EXIT: // 현재 프로세스 종료 및 상태 기록
        sys_exit((int)arg1);
        NOT_REACHED();

    case SYS_WRITE:                                                                  // 표준 출력(write)에 대해서만 처리
        f->R.rax = (uint64_t)sys_write((int)arg1, (const void *)arg2, (size_t)arg3); // 반환값을 RAX에 기록
        break;

    case SYS_READ: // 표준 입력(read)에 대해서만 처리
        f->R.rax = (uint64_t)sys_read((int)arg1, (void *)arg2, (size_t)arg3);
        break;

    default: // 미지원 syscall → 즉시 실패 종료
        sys_exit(-1);
        NOT_REACHED();
    }
}

static void die_bad_access(void) {
    sys_exit(-1); // 포인터 검증 실패 시 관례적으로 -1 코드로 종료
}

/**
 * 사용자 영역 주소 하나에서 1바이트를 안전하게 읽어 오는 유틸 함수
 * dst : 검증이 끝나고 안전하게 읽어 온 값을 저장할 커널 측 버퍼의 위치
 * - 함수가 성공적으로 값을 가져오면 *dst에 해당 바이트를 써주고 true를 반환
 * - 실패 시 dst는 변경되지 않고 false 반환
 * uaddr : 사용자 프로그램이 제공한 원본 주소. 여기서 1바이트를 읽어오고 싶다는 뜻이라 가정하고
 * 먼저 이 주소가 유저 영역인지 현재 프로세스의 페이지 테이블에 실제 매핑이 있는지를 검사
 */
static bool get_user_u8(uint8_t *dst, const uint8_t *uaddr) {
    if (!is_user_vaddr(uaddr)) // 커널 영역 접근 시 즉시 실패
        return false;

    struct thread *t = thread_current();
    if (t->pml4 == NULL) // 주소 공간이 정리된 상황(종료 직전 등)
        return false;

    void *kaddr = pml4_get_page(t->pml4, uaddr); // 현재 매핑된 커널 가상 주소 획득
    if (kaddr == NULL)
        return false;
    // dst에 커널 주소가 가리키는(검증을 통과한 사용자 메모리에서 안전하게 읽어온 1바이트) 값 쓰기
    *dst = *(uint8_t *)kaddr;
    return true;
}

/**
 * 사용자 메모리 영역에 1바이트를 안전하게 써 넣는 함수
 * uaddr : 사용자 공간에 있는 목적지 주소. get_user_u8과 마찬가지로 검사
 * value : 사용자 공간에 기록할 실제 데이터(1바이트)
 */
static bool put_user_u8(uint8_t *uaddr, uint8_t value) {
    if (!is_user_vaddr(uaddr)) // 쓰기 대상도 반드시 유저 주소여야 함
        return false;

    struct thread *t = thread_current();
    // 현재 스레드의 페이지 테이블이 존재하는지
    if (t->pml4 == NULL)
        return false;

    // 사용자 주소에 대응하는 가상 주소를 얻기
    void *kaddr = pml4_get_page(t->pml4, uaddr); // 쓰기 가능한 페이지인지 확인(공유 여부 포함)
    if (kaddr == NULL)
        return false;

    // 커널 주소에 value 값 써넣기
    *(uint8_t *)kaddr = value;
    return true;
}

/**
 * 사용자 버퍼(usrc)에서 size 바이트만큼을 커널 버퍼(dst)로 가져옴
 */
static void copy_in(void *dst, const void *usrc, size_t size) {
    uint8_t *d = dst;        // 커널 측 목적지 버퍼. 검사 완료된 데이터가 여기 저장
    const uint8_t *s = usrc; // 사용자 프로그램이 제공한 원본 포인터

    // size만큼 돌며 get_user_u8로 매 바이트 안전성을 확인한 뒤 읽은 값을 커널 버퍼에 넣음
    for (size_t i = 0; i < size; i++) {
        uint8_t byte;
        if (!get_user_u8(&byte, s + i)) // 매 바이트마다 주소 유효성 검사
            // 실패 시 즉시 프로세스 종료
            die_bad_access();
        d[i] = byte;
    }
}

/**
 * 커널 버퍼(src)의 내용을 사용자 버퍼(udst)로 size 바이트만큼 쓰기
 */
static void copy_out(void *udst, const void *src, size_t size) {
    const uint8_t *s = src; // 커널에서 준비한 원본 데이터
    uint8_t *d = udst;      // 사용자 측 목적지 버퍼. 실제 사용자 주소이므로 접근 전 매 바이트 검사

    // size만큼 돌며 put_user_u8로 사용자 버퍼 위치가 쓰기 가능한지 확인하고, 성공하면 해당 위치에 한 바이트 저장
    for (size_t i = 0; i < size; i++) {
        if (!put_user_u8(d + i, s[i])) // 사용자 버퍼에 실제로 쓰기 가능한지 확인
            die_bad_access();
    }
}

/**
 * 사용자 프로그램이 halt 시스템 콜을 호출했을 때 곧바로 pintos를 종료. 정상복귀 x
 */
static void sys_halt(void) { power_off(); }

/**
 * 현재 스레드의 exit_status 필드에 전달된 값을 저장하고 자원 정리
 */
static void sys_exit(int status) {
    struct thread *t = thread_current();
    t->exit_status = status; // process_exit()에서 출력할 종료 코드를 미리 저장
    thread_exit();           // thread_exit() -> process_exit() 경유
}

/**
 * 사용자 공간 버퍼의 ubuf에서 최대 size 바이트를 읽어 표준 출력을 내보냄.
 * fd : 출력 대상 파일 디스크립터
 * ubuf : 사용자 프로그램이 넘긴 데이터의 원본 주소. copy_in을 통해 매번 안전하게 커널 버퍼로 옮긴 뒤 출력
 * size : 출력할 바이트 수
 */
static int sys_write(int fd, const void *ubuf, size_t size) {
    if (size == 0)
        return 0;

    if (fd == STDOUT_FILENO) {    // 표준 출력 : 1
        size_t written = 0;       // 현재까지 출력한 총 바이트 수
        uint8_t buffer[IO_CHUNK]; // 최대 IO_CHUNK 크기의 임시 커널 버퍼
        // 남은 데이터가 있으면 반복
        while (written < size) {
            // 한 번에 처리할 크기 제한
            size_t chunk = size - written;
            if (chunk > IO_CHUNK)
                chunk = IO_CHUNK;

            // 유저 버퍼를 안전하게 복사
            copy_in(buffer, (const uint8_t *)ubuf + written, chunk);
            // 콘솔에 출력
            putbuf((const char *)buffer, chunk);
            written += chunk; // 쓴 양 갱신
        }
        return (int)size; // 모두 다 썼다면 크기 반환
    }

    return -1;
}

/**
 * 표준 입력에서 최대 size 바이트를 읽어 사용자 버퍼로 전달
 * 마찬가지로 ubuf가 사용자 버퍼
 */
static int sys_read(int fd, void *ubuf, size_t size) {
    if (size == 0)
        return 0;

    if (fd == STDIN_FILENO) {  // 표준 입력 : 0
        size_t read_bytes = 0; // 읽은 바이트 수
        uint8_t buffer[IO_CHUNK];

        while (read_bytes < size) {
            size_t chunk = size - read_bytes;
            if (chunk > IO_CHUNK)
                chunk = IO_CHUNK;

            // 키도브에서 문자를 받아 buffer에 채우기
            for (size_t i = 0; i < chunk; i++)
                buffer[i] = input_getc();
            // 방금 읽은 내용을 사용자 버퍼로 안전 복사
            copy_out((uint8_t *)ubuf + read_bytes, buffer, chunk);
            read_bytes += chunk;
        }
        return (int)size;
    }

    return -1;
}
