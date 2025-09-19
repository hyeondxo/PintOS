#include "userprog/syscall.h"
#include "devices/input.h"
#include "filesys/file.h"
#include "filesys/filesys.h"
#include "intrinsic.h"
#include "lib/kernel/stdio.h"
#include "threads/flags.h"
#include "threads/init.h"
#include "threads/interrupt.h"
#include "threads/loader.h"
#include "threads/mmu.h"
#include "threads/palloc.h"
#include "threads/synch.h"
#include "threads/thread.h"
#include "threads/vaddr.h"
#include "userprog/gdt.h"
#include "userprog/process.h"
#include <debug.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <syscall-nr.h>

void syscall_entry(void);                  // 어셈블리 레벨에서 정의된 syscall 진입점 (intrinsic.asm 등)
void syscall_handler(struct intr_frame *); // 실제 C 레벨에서 시스템 콜을 처리하는 함수

#define STDIN_FILENO 0   // 표준 입력 파일 디스크립터
#define STDOUT_FILENO 1  // 표준 출력 파일 디스크립터
#define IO_CHUNK 256     // 사용자 버퍼를 한 번에 처리할 최대 바이트 수
#define MAX_PATH_LEN 512 // 파일/프로그램 경로 임시 버퍼 크기(널 포함)

static void die_bad_access(void) NO_RETURN;                           // 잘못된 유저 주소 접근 시 즉시 프로세스 종료
static bool get_user_u8(uint8_t *dst, const uint8_t *uaddr);          // 단일 바이트 안전 읽기
static bool put_user_u8(uint8_t *uaddr, uint8_t value);               // 단일 바이트 안전 쓰기
static void copy_in(void *dst, const void *usrc, size_t size);        // 사용자->커널 버퍼 복사
static void copy_out(void *udst, const void *src, size_t size);       // 커널->사용자 버퍼 복사
static void copy_in_string(char *dst, const char *usrc, size_t size); // 사용자 문자열을 NUL까지 안전 복사

static void sys_halt(void) NO_RETURN;
static int sys_write(int fd, const void *ubuf, size_t size);
static int sys_read(int fd, void *ubuf, size_t size);
static int sys_open(const char *ufile);
static int sys_filesize(int fd);
static void sys_seek(int fd, unsigned position);
static unsigned sys_tell(int fd);
static void sys_close(int fd);
static bool sys_create(const char *ufile, unsigned initial_size);
static bool sys_remove(const char *ufile);
static void sys_exec(const char *ucommand) NO_RETURN;
static int sys_wait(tid_t pid);
static tid_t sys_fork(const char *uname, struct intr_frame *f);

static struct file *fd_to_file(int fd);
static int alloc_fd(struct file *file);
static void free_fd(int fd);

struct lock filesys_lock; // 파일 시스템 API 호출을 직렬화하기 위한 전역 락

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
    lock_init(&filesys_lock); // 파일 시스템 연산을 직렬화할 전역 락 준비

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
    // 유저 프로그램이 syscall 인스트럭션은 실행하면 시스템콜 번호는 rax 레지스터에 담겨 넘어옴
    uint64_t syscall_no = f->R.rax; // 시스템 콜 번호는 RAX에 위치
    uint64_t arg1 = f->R.rdi;       // 첫 번째 인자: RDI
    uint64_t arg2 = f->R.rsi;       // 두 번째 인자: RSI
    uint64_t arg3 = f->R.rdx;       // 세 번째 인자: RDX

    switch (syscall_no) {
    case SYS_HALT: // halt(): 인자 없음. 커널 전원을 즉시 종료
        sys_halt();
        NOT_REACHED();

    case SYS_EXIT: // exit(status): RDI=status. 현재 프로세스를 종료하고 종료 코드를 기록
        sys_exit((int)arg1);
        NOT_REACHED();

    case SYS_FORK: // fork(name): RDI=스레드 이름. 부모는 자식 tid, 자식은 0 반환
        f->R.rax = (uint64_t)sys_fork((const char *)arg1, f);
        break;

    case SYS_EXEC: // exec(cmd): RDI=커맨드 문자열. 성공 시 현재 프로세스를 새 프로그램으로 교체
        sys_exec((const char *)arg1);
        NOT_REACHED();

    case SYS_WAIT: // wait(pid): RDI=자식 tid. 종료 상태 반환
        f->R.rax = (uint64_t)sys_wait((tid_t)arg1);
        break;

    case SYS_CREATE: // create(path, size)
        f->R.rax = sys_create((const char *)arg1, (unsigned)arg2);
        break;

    case SYS_REMOVE: // remove(path)
        f->R.rax = sys_remove((const char *)arg1);
        break;

    case SYS_WRITE: // write(fd, buf, size): RDI=fd, RSI=buf, RDX=size. STDOUT 또는 열린 파일에 기록
        f->R.rax = (uint64_t)sys_write((int)arg1, (const void *)arg2, (size_t)arg3);
        break;

    case SYS_READ: // read(fd, buf, size): RDI=fd, RSI=buf, RDX=size. STDIN 또는 열린 파일에서 읽기
        f->R.rax = (uint64_t)sys_read((int)arg1, (void *)arg2, (size_t)arg3);
        break;

    case SYS_OPEN: // open(path): RDI=사용자 문자열 포인터. 성공 시 새 fd를 RAX로 반환
        f->R.rax = (uint64_t)sys_open((const char *)arg1);
        break;

    case SYS_FILESIZE: // filesize(fd): RDI=fd. 파일 길이를 RAX로 반환
        f->R.rax = (uint64_t)sys_filesize((int)arg1);
        break;

    case SYS_SEEK: // seek(fd, position): RDI=fd, RSI=next offset. 반환값 없음(RAX=0)
        sys_seek((int)arg1, (unsigned)arg2);
        f->R.rax = 0;
        break;

    case SYS_TELL: // tell(fd): RDI=fd. 현재 오프셋을 RAX로 반환
        f->R.rax = (uint64_t)sys_tell((int)arg1);
        break;

    case SYS_CLOSE: // close(fd): RDI=fd. fd를 닫고 RAX=0
        sys_close((int)arg1);
        f->R.rax = 0;
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

/* 사용자 문자열을 커널 버퍼로 복사하면서 널 문자를 반드시 확인한다. */
static void copy_in_string(char *dst, const char *usrc, size_t size) {
    if (size == 0)
        die_bad_access();

    for (size_t i = 0; i < size; i++) {
        uint8_t byte;                                       // 임시로 한 바이트 저장할 버퍼
        if (!get_user_u8(&byte, (const uint8_t *)usrc + i)) // 사용자 주소 usrc+i에서 안전하게 1바이트 읽기
            die_bad_access();                               // 읽기에 실패하면 즉시 프로세스 종료
        dst[i] = byte;                                      // 커널 버퍼에 복사
        if (byte == '\0')                                   // 널 종단을 만났다면 문자열 복사가 완료된 것
            return;                                         // 호출자에게 성공적으로 반환
    }
    die_bad_access(); // size 이내에서 널 문자를 찾지 못하면 잘못된 문자열로 간주
}

/**
 * 사용자 프로그램이 halt 시스템 콜을 호출했을 때 곧바로 pintos를 종료. 정상복귀 x
 */
static void sys_halt(void) { power_off(); }

/**
 * 현재 스레드의 exit_status 필드에 전달된 값을 저장하고 자원 정리
 */
void sys_exit(int status) {
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
        while (written < size) {
            size_t chunk = size - written;
            if (chunk > IO_CHUNK)
                chunk = IO_CHUNK;

            copy_in(buffer, (const uint8_t *)ubuf + written, chunk);
            putbuf((const char *)buffer, chunk);
            written += chunk;
        }
        return (int)size; // 모두 다 썼다면 크기 반환
    }

    if (fd == STDIN_FILENO)
        return -1; // 표준 입력으로의 쓰기는 허용하지 않음

    struct file *file = fd_to_file(fd);
    if (file == NULL)
        return -1; // 유효하지 않은 fd이면 오류 반환

    size_t written = 0;       // 누적 출력 바이트 수
    uint8_t buffer[IO_CHUNK]; // 사용자 버퍼를 임시로 담을 커널 버퍼

    while (written < size) {
        size_t chunk = size - written;
        if (chunk > IO_CHUNK)
            chunk = IO_CHUNK; // 한 번에 처리할 최대 크기 제한

        copy_in(buffer, (const uint8_t *)ubuf + written, chunk); // 사용자 버퍼를 안전하게 복사

        lock_acquire(&filesys_lock);                 // 파일 시스템 접근 직전 전역 락 획득
        int bytes = file_write(file, buffer, chunk); // 실제 파일 쓰기
        lock_release(&filesys_lock);                 // 쓰기 완료 후 즉시 락 해제

        if (bytes <= 0)
            break;        // 더 이상 쓸 수 없는 경우 종료
        written += bytes; // 실제 쓴 양만큼 누적
        if ((size_t)bytes < chunk)
            break; // 부분만 쓴 경우(예: 공간 부족) 종료
    }

    return (int)written; // 총 출력 바이트 수 반환
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

            for (size_t i = 0; i < chunk; i++)
                buffer[i] = input_getc();
            copy_out((uint8_t *)ubuf + read_bytes, buffer, chunk);
            read_bytes += chunk;
        }
        return (int)size;
    }

    if (fd == STDOUT_FILENO)
        return -1; // 표준 출력으로부터의 읽기는 허용하지 않음

    struct file *file = fd_to_file(fd);
    if (file == NULL)
        return -1; // 잘못된 fd -> 오류 반환

    size_t read_bytes = 0;    // 누적 읽은 바이트 수
    uint8_t buffer[IO_CHUNK]; // 한 번에 읽어올 임시 커널 버퍼

    while (read_bytes < size) {
        size_t chunk = size - read_bytes;
        if (chunk > IO_CHUNK)
            chunk = IO_CHUNK; // 읽기 단위를 IO_CHUNK 이하로 제한

        lock_acquire(&filesys_lock);                // 파일 시스템 접근 직전 락 획득
        int bytes = file_read(file, buffer, chunk); // 실제 파일에서 읽기 수행
        lock_release(&filesys_lock);                // 읽기 완료 후 즉시 락 해제

        if (bytes <= 0)
            break; // EOF 또는 오류 -> 루프 종료

        copy_out((uint8_t *)ubuf + read_bytes, buffer, bytes); // 커널 버퍼를 사용자 버퍼로 복사
        read_bytes += bytes;                                   // 누적 읽은 양 업데이트
        if ((size_t)bytes < chunk)
            break; // chunk보다 덜 읽었으면 추가 데이터 없음
    }

    return (int)read_bytes; // 실제 읽은 바이트 수 반환
}

/* 사용자 경로 문자열을 검증 후 파일을 열어 새 fd를 할당한다. */
static int sys_open(const char *ufile) {
    char path[MAX_PATH_LEN];                  // 커널 측 경로 버퍼
    copy_in_string(path, ufile, sizeof path); // 사용자 문자열을 안전하게 복사

    if (path[0] == '\0') // 빈 문자열은 허용하지 않음
        return -1;

    lock_acquire(&filesys_lock);            // 파일 시스템 접근 보호
    struct file *file = filesys_open(path); // 파일 열기 시도
    if (file == NULL) {
        lock_release(&filesys_lock); // 실패하면 락 해제 후 -1
        return -1;
    }

    int fd = alloc_fd(file); // 새 fd 슬롯 할당
    if (fd == -1) {
        file_close(file); // 슬롯 없음 -> 파일 닫기
        lock_release(&filesys_lock);
        return -1;
    }

    lock_release(&filesys_lock); // 파일 등록 완료 후 락 해제
    return fd;                   // 성공 시 새 fd 반환
}

/* 열린 파일의 총 길이를 조회한다. 실패 시 -1 반환. */
static int sys_filesize(int fd) {
    struct file *file = fd_to_file(fd); // fd에 해당하는 파일 객체 조회
    if (file == NULL)
        return -1; // 잘못된 fd면 -1 반환

    lock_acquire(&filesys_lock);
    int length = file_length(file); // 파일 시스템에서 길이 가져오기
    lock_release(&filesys_lock);
    return length; // 길이를 그대로 반환
}

/* 파일 오프셋을 지정 위치로 이동한다. 존재하지 않는 fd는 무시. */
// position : 다음 읽기/쓰기 오프셋으로 설정할 파일 내 바이트 위치
static void sys_seek(int fd, unsigned position) {
    struct file *file = fd_to_file(fd); // fd에 대응하는 파일 객체 조회
    if (file == NULL)
        return; // 잘못된 fd면 아무 것도 하지 않음

    lock_acquire(&filesys_lock); // 파일 위치 변경은 파일 시스템 락으로 보호
    file_seek(file, position);   // 지정한 위치로 오프셋 이동
    lock_release(&filesys_lock);
}

/* 현재 파일 오프셋을 반환하고, 실패 시 (unsigned)-1을 돌려준다. */
static unsigned sys_tell(int fd) {
    struct file *file = fd_to_file(fd);
    if (file == NULL)
        return (unsigned)-1;

    lock_acquire(&filesys_lock); // 위치 조회 전 파일 시스템 락 획득
    off_t pos = file_tell(file); // 현재 파일 오프셋 얻기
    lock_release(&filesys_lock);
    if (pos < 0)
        return (unsigned)-1; // 오류 시 unsigned -1 반환
    return (unsigned)pos;    // 정상 시 위치를 unsigned로 캐스팅해 반환
}

/* 파일 디스크립터를 닫고 슬롯을 비운다. 표준 입출력은 무시. */
static void sys_close(int fd) {
    if (fd < 2)
        return; // 표준 입출력(0,1)은 닫지 않음

    struct file *file = fd_to_file(fd);
    if (file == NULL)
        return; // 이미 닫힌 fd는 무시

    free_fd(fd); // fd 테이블에서 슬롯 비우기

    lock_acquire(&filesys_lock); // 파일 시스템 접근 보호
    file_close(file);            // 마지막 참조라면 파일 객체까지 정리
    lock_release(&filesys_lock);
}

/**
 * 사용자로부터 받은 파일 경로와 초기 크기를 바탕으로 새 파일을 생성
 * ufile : 사용자 공간의 문자열 포인터(생성할 파일 경로)
 * initial_size : 새 파일의 초기 바이트 크기
 * 반환 : 생성 성공 시 true. 실패 시 false
 */
static bool sys_create(const char *ufile, unsigned initial_size) {
    char path[MAX_PATH_LEN];                  // 커널 공간에 경로 문자열을 담을 임시 버퍼
    copy_in_string(path, ufile, sizeof path); // ufile로부터 커널 버퍼 path로 문자열을 안전 복사
    if (path[0] == '\0')                      // 빈 문자열은 파일 이름으로 허용하지 x
        return false;
    lock_acquire(&filesys_lock);
    bool ok = filesys_create(path, initial_size); // 실제 파일 시스템 계층에 파일 생성 요청
    lock_release(&filesys_lock);
    return ok;
}

// 사용자로부터 받은 경로의 파일을 파일시스템에서 삭제
static bool sys_remove(const char *ufile) {
    char path[MAX_PATH_LEN];
    copy_in_string(path, ufile, sizeof path);
    if (path[0] == '\0')
        return false;
    lock_acquire(&filesys_lock);
    bool ok = filesys_remove(path);
    lock_release(&filesys_lock);
    return ok;
}

/**
 * 현재 프로세스의 이미지를 새로운 실행 파일로 교체. 성공하면 반환하지 않고 새 프로그램으로 점프 실패하면 프로세스 종료
 * ucommand : 사용자 공간의 문자열 포인터 - 프로그램 경로와 인자를 포함한 커맨드 라인
 */
static void sys_exec(const char *ucommand) {
    // 전체 커맨드 라인을 담기 위해 한 페이지(4KB) 크기의 커널 버퍼를 페이지 할당자로 확보
    // exec는 인자 길이가 길 수 있어 스택의 고정 소형 버퍼 대신 페이지 단위의 버퍼를 사용
    char *cmdline = palloc_get_page(0);
    if (cmdline == NULL) // 메모리 부족으로 버퍼 확보 실패
        sys_exit(-1);

    copy_in_string(cmdline, ucommand, PGSIZE); // ucommand -> cmdline으로 복사

    // 프로세스 주소 공간을 비우고(기존 프로그램 폐기), cmdline으로 지정한 실행 파일을 로드한 뒤 인자 스택을 구성하고
    // 유저 모드로 점프할 준비를 함. 성공 시 이 호출은 실제로 반환하지 않고 do_iret로 넘어감
    int rc = process_exec(cmdline);
    if (rc == -1)
        sys_exit(-1);

    NOT_REACHED();
}

// 현재 프로세스가 지정한 자식 프로세스의 종료를 기다리고 그 자식의 종료 코드를 반환
// pid : 기다릴 대상 자식의 프로세스 ID(tid)
static int sys_wait(tid_t pid) { return process_wait(pid); }

// 현재 프로세스를 복제. 부모는 새 자식의 tid를, 자식은 0을 받도록 구성
static tid_t sys_fork(const char *uname, struct intr_frame *f) {
    char name[16];                            // 자식 스레드의 이름을 담을 커널 측 임시 버퍼
    copy_in_string(name, uname, sizeof name); // 사용자(uname) -> 커널 버퍼로 name을 안전 복사
    tid_t tid = process_fork(name, f);        // 부모의 intr_frame f와 복사한 이름을 넘겨 실제 fork 수행
    if (tid == TID_ERROR)
        return -1;
    return tid;
}

/* fd -> file* 매핑 테이블에서 파일 객체를 조회한다. */
static struct file *fd_to_file(int fd) {
    if (fd < 2 || fd >= FD_TABLE_SIZE)
        return NULL;                       // fd가 유효 범위를 벗어나면 NULL 반환
    return thread_current()->fd_table[fd]; // 해당 fd 슬롯에 저장된 file 포인터 반환
}

/* 가장 작은 사용 가능한 fd 번호를 찾아 file 포인터를 등록한다. */
static int alloc_fd(struct file *file) {
    struct thread *t = thread_current();
    for (int fd = 2; fd < FD_TABLE_SIZE; fd++) {
        if (t->fd_table[fd] == NULL) {
            t->fd_table[fd] = file;
            return fd;
        }
    }
    return -1;
}

/* fd 범위가 올바른 경우 테이블에서 해당 슬롯을 비워 중복 닫기를 방지한다. */
static void free_fd(int fd) {
    if (fd < 2 || fd >= FD_TABLE_SIZE)
        return;
    thread_current()->fd_table[fd] = NULL;
}
