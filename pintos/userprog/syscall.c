#include "userprog/syscall.h"
#include <stdio.h>
#include <syscall-nr.h>
#include "threads/interrupt.h"
#include "threads/thread.h"
#include "threads/loader.h"
#include "userprog/gdt.h"
#include "threads/flags.h"
#include "intrinsic.h"
#include "lib/stdio.h"
#include "devices/input.h"
#include "filesys/file.h"
#include "filesys/filesys.h"
#include "threads/mmu.h"
#include "threads/palloc.h"
#include "threads/synch.h"
#include "threads/vaddr.h"
#include "userprog/process.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <debug.h>

void syscall_entry (void);
void syscall_handler (struct intr_frame *);

#define IO_CHUNK 256
#define MAX_PATH_LEN 512

static void sys_halt (void) NO_RETURN;
static tid_t sys_fork (const char *uname, struct intr_frame *f);
static int sys_exec (const char *ucommand);
static int sys_wait (tid_t pid);
static bool sys_create (const char *file, unsigned initial_size);
static bool sys_remove (const char *ufile);
static int sys_open (const char *ufile);
static int sys_filesize (int fd);
static int sys_read (int fd, void *ubuf , size_t size);
static int sys_write (int fd, const void *ubuf, size_t size);
static void sys_seek (int fd, unsigned position);
static unsigned sys_tell (int fd);
static void sys_close (int fd);

/* 헬퍼 함수 시그니처 */
static void free_fd (int fd);
static struct file *fd_to_file (int fd);
static int alloc_fd (struct file *file);
static void die_bad_access(void);
void check_address (void *addr);
static bool get_user_u8 (uint8_t *dst, const uint8_t *uaddr);
static void copy_in_string (char *dst, const char *usrc, size_t size);
static void copy_in(void *dst, const void *usrc, size_t size);
static void copy_out(void *udst, const void *src, size_t size);
static void copy_in_string (char *dst, const char *usrc, size_t size);

struct lock filesys_lock; // 파일 시스템 API 호출을 직렬화하기 위한 전역 락

/* System call.
 *
 * Previously system call services was handled by the interrupt handler
 * (e.g. int 0x80 in linux). However, in x86-64, the manufacturer supplies
 * efficient path for requesting the system call, the `syscall` instruction.
 *
 * The syscall instruction works by reading the values from the the Model
 * Specific Register (MSR). For the details, see the manual. */

#define MSR_STAR 0xc0000081         /* Segment selector msr */
#define MSR_LSTAR 0xc0000082        /* Long mode SYSCALL target */
#define MSR_SYSCALL_MASK 0xc0000084 /* Mask for the eflags */

/* 헬퍼 함수들 정의 */

static void free_fd (int fd) {
	if (fd < 2 || fd > FD_TABLE_SIZE) return;
	thread_current()->fd_table[fd] = NULL;
}

static struct file *fd_to_file (int fd) {
	if (fd < 2 || fd >= FD_TABLE_SIZE) return NULL;
	return thread_current()->fd_table[fd]; 
}

static int alloc_fd (struct file *file) {
	struct thread *t = thread_current();
	for (int fd = 2; fd < FD_TABLE_SIZE; fd++) {
		if (t->fd_table[fd] == NULL) {
			t->fd_table[fd] = file;
			return fd;
		}
	}
	return -1;
}

static void die_bad_access(void) {
    sys_exit(-1); // 포인터 검증 실패 시 관례적으로 -1 코드로 종료
}

/* 유효한 주소인지 체크 */
void check_address (void *addr) {
	struct thread *cur = thread_current();
	/* --- Project 2: User memory access --- */
	// if (!is_user_vaddr(addr)||addr == NULL) 
	//-> 이 경우는 유저 주소 영역 내에서도 할당되지 않는 공간 가리키는 것을 체크하지 않음. 그래서 
	// pml4_get_page를 추가해줘야!
	if (!is_user_vaddr(addr) || addr == NULL || 
	pml4_get_page(cur->pml4, addr)== NULL) { sys_exit(-1); }
}

static bool get_user_u8 (uint8_t *dst, const uint8_t *uaddr) {
	if (!is_user_vaddr(uaddr)) return false;	// 커널 영역 접근 시 즉시 실패

	struct thread *t = thread_current();
	if (t->pml4 == NULL) // 주소 공간 정리된 상황
		return false;
	
	void *kaddr = pml4_get_page(t->pml4, uaddr); // 현재 매핑된 가상 주소 획득
	if (kaddr == NULL) return false;
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
/* 사용자 문자열을 커널 버퍼로 복사하면서 널 문자 반드시 확인 */
static void copy_in_string (char *dst, const char *usrc, size_t size) {
	if (size == 0) die_bad_access();

	for (size_t i = 0; i < size; i++) {
		uint8_t byte;											// 임시로 한 바이트 저장
		if (!get_user_u8(&byte, (const uint8_t *)usrc + i)) 	// 사용자 주소 usrc+i에서 안전하게 1바이트 읽기.
			die_bad_access();	 								//실패 시 프로세스 종료
		dst[i] = byte;											//커널 버퍼에 복사
		if (byte == '\0') return;								// NULL(\0)만나면 복사 완료 의미, 반환
	}
	die_bad_access();
}

void
syscall_init (void) {
	lock_init(&filesys_lock); // 파일 시스템 연산을 직렬화할 전역 락 준비

	write_msr(MSR_STAR, ((uint64_t)SEL_UCSEG - 0x10) << 48  |
			((uint64_t)SEL_KCSEG) << 32);
	write_msr(MSR_LSTAR, (uint64_t) syscall_entry);

	/* The interrupt service rountine should not serve any interrupts
	 * until the syscall_entry swaps the userland stack to the kernel
	 * mode stack. Therefore, we masked the FLAG_FL. */
	write_msr(MSR_SYSCALL_MASK,
			FLAG_IF | FLAG_TF | FLAG_DF | FLAG_IOPL | FLAG_AC | FLAG_NT);
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
void
syscall_handler (struct intr_frame *f) {
	// TODO: Your implementation goes here.
	// printf ("system call!\n");
	// thread_exit ();
	int sys_input = f->R.rax;
	uint64_t arg0 = f->R.rdi;
	uint64_t arg1 = f->R.rsi;
	uint64_t arg2 = f->R.rdx;
	uint64_t arg3 = f->R.r10;
	uint64_t arg4 = f->R.r8;
	uint64_t arg5 = f->R.r9;

	switch (sys_input) {
		case SYS_HALT: // power_off() 호출해 pintos 종료. 디버깅 정보 유실
			sys_halt ();
			NOT_REACHED();
	
		case SYS_EXIT: // 현재 사용자 프로그램 종료, 상태 코드 커널에 반환. 부모가 wait하면 부모에게 반환 (성공 == 0)
			sys_exit ((int)arg0);
			NOT_REACHED();
		
		case SYS_FORK: 
			f->R.rax = (uint64_t)sys_fork((const char *)arg0, f);
			break;

		case SYS_EXEC: // exec(cmd): RDI=커맨드 문자열. 성공 시 현재 프로세스를 새 프로그램으로 교체
			sys_exec ((const char *)arg0);
			NOT_REACHED();

		case SYS_WAIT: // wait(tid), RDI=자식 tid. 종료 상태 반환
			f->R.rax = (uint64_t)sys_wait ((tid_t)arg0);
			break;

		case SYS_CREATE:
			f->R.rax = sys_create ((const char *)arg0, (unsigned)arg1);
			break;

		case SYS_REMOVE:
			f->R.rax = sys_remove ((const char *)arg0);
			break;

		case SYS_OPEN:
			f->R.rax = (uint64_t)sys_open ((const char *)arg0);
			break;
		
		case SYS_FILESIZE:
			f->R.rax = (uint64_t)sys_filesize ((int)arg0);
			break;

		case SYS_READ:
			f->R.rax = (uint64_t)sys_read ((int)arg0, (void *)arg1, (size_t)arg2);
			break;

		case SYS_WRITE: 
			f->R.rax = (uint64_t)sys_write ((int)arg0, (const void *)arg1, (size_t)arg2);
			break;
		
		case SYS_SEEK:
			sys_seek ((int)arg0, (size_t)arg1);
			f->R.rax = 0;
			break;

		case SYS_TELL:
			f->R.rax = (uint64_t)sys_tell ((int)arg0);
			break;

		case SYS_CLOSE:
			sys_close ((int)arg0);
			f->R.rax = 0;
			break;

		default: 
			sys_exit(-1);
			NOT_REACHED();		
	}
}

/* pintos 종료시키는 함수 */
static void sys_halt(void) {
	power_off();
}

/* 현재 프로세스를 종료시키는 시스템 콜 */
void sys_exit(int status) {
	struct thread *t = thread_current();
	t->exit_status = status;
	/* 정상적으로 종료됐다면 status는 0 */
	/* status: 프로그램이 정상적으로 종료됐는지 확인 */
	thread_exit();
}

static tid_t sys_fork (const char *uname, struct intr_frame *f) {
	// 넉넉한 커널 버퍼(한 페이지)로 안전 복사
	char *kname = palloc_get_page(0);
	if (!kname) return -1;
	copy_in_string(kname, uname, PGSIZE);    // 충분히 큰 사이즈

	// 스레드 이름은 짧게 잘라 사용 (16바이트 표시, 실제 fork 로직엔 영향 없음)
	char shortname[16];
	strlcpy(shortname, kname, sizeof shortname);
	palloc_free_page(kname);

	tid_t tid = process_fork(shortname, f);
	return tid == TID_ERROR ? -1 : tid;
}

/**
 * 현재 프로세스의 이미지를 새로운 실행 파일로 교체. 성공하면 반환하지 않고 새 프로그램으로 점프 실패하면 프로세스 종료
 * ucommand : 사용자 공간의 문자열 포인터 - 프로그램 경로와 인자를 포함한 커맨드 라인
 */
static int sys_exec (const char *ucommand) {
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

static int sys_wait (tid_t pid) {
	return process_wait(pid);
}

/* 파일 생성하는 시스템 콜 */
static bool sys_create (const char *file, unsigned initial_size) {
	/* 성공이면 true, 실패면 false */
	char path[MAX_PATH_LEN];					// 커널 공간에 경로 문자열을 담을 임시 버퍼
	copy_in_string(path, file, sizeof path);	// file로부터 커널 버퍼 path로 문자열 안전 복사
	if (path[0] == '\0') return false;			// 빈 문자열은 파일 이름 X
	
	lock_acquire(&filesys_lock);
	bool ok = filesys_create(path, initial_size);
	lock_release(&filesys_lock);

	return ok;
}

/* 사용자로부터 받은 경로의 파일을 파일 시스템에서 삭제 */
static bool sys_remove (const char *ufile) {
	char path[MAX_PATH_LEN];
	copy_in_string(path, ufile, sizeof path);
	if (path[0] == '\0') return false;

	lock_acquire(&filesys_lock);
	bool ok = filesys_remove(ufile);
	lock_release(&filesys_lock);
	return ok;
}

/* 사용자 경로 문자열 검증 후 파일을 열고 새 fd 할당 */
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

static int sys_filesize (int fd) {
	struct file *file = fd_to_file(fd);
	if (file == NULL) return -1;

	lock_acquire(&filesys_lock);
	int length = file_length(file);
	lock_release(&filesys_lock);
	return length;
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

            copy_in (buffer, (const uint8_t *)ubuf + written, chunk);
            putbuf ((const char *)buffer, chunk);
            written += chunk;
        }
        return (int)size; // 모두 다 썼다면 크기 반환
    }

    if (fd == STDIN_FILENO)
        return -1; // 표준 입력으로의 쓰기는 허용하지 않음

    struct file *file = fd_to_file (fd);
    if (file == NULL)
        return -1; // 유효하지 않은 fd이면 오류 반환

    size_t written = 0;       // 누적 출력 바이트 수
    uint8_t buffer[IO_CHUNK]; // 사용자 버퍼를 임시로 담을 커널 버퍼

    while (written < size) {
        size_t chunk = size - written;
        if (chunk > IO_CHUNK)
            chunk = IO_CHUNK; // 한 번에 처리할 최대 크기 제한

        copy_in (buffer, (const uint8_t *)ubuf + written, chunk); // 사용자 버퍼를 안전하게 복사

        lock_acquire (&filesys_lock);                 // 파일 시스템 접근 직전 전역 락 획득
        int bytes = file_write (file, buffer, chunk); // 실제 파일 쓰기
        lock_release (&filesys_lock);                 // 쓰기 완료 후 즉시 락 해제

        if (bytes <= 0)
            break;        // 더 이상 쓸 수 없는 경우 종료
        written += bytes; // 실제 쓴 양만큼 누적
        if ((size_t)bytes < chunk)
            break; // 부분만 쓴 경우(예: 공간 부족) 종료
    }

    return (int)written; // 총 출력 바이트 수 반환
}

/* 파일 오프셋을 지정 위치로 이동한다. 존재하지 않는 fd는 무시. */
// position : 다음 읽기/쓰기 오프셋으로 설정할 파일 내 바이트 위치
static void sys_seek (int fd, unsigned position) {
	struct file *file = fd_to_file(fd);
	if (file == NULL) return;

	lock_acquire(&filesys_lock);
	file_seek(file, position);
	lock_release(&filesys_lock);
}

/* 현재 파일 오프셋을 반환하고, 실패 시 (unsigned)-1을 돌려준다. */
static unsigned sys_tell (int fd) {
	struct file *file = fd_to_file(fd);
	if (file == NULL) return (unsigned)-1;
	
	lock_acquire(&filesys_lock);
	off_t pos = file_tell(file);
	lock_release(&filesys_lock);
	if (pos < 0) return (unsigned)-1;
	return (unsigned)pos;
}

static void sys_close (int fd) {
	if (fd < 2 ) return;				// 표준 입출력은 닫지 않음

	struct file *file = fd_to_file(fd);
	if (file == NULL) return;			// 이미 닫힘

	free_fd(fd);						// fd테이블에서 슬롯 비우기

	lock_acquire(&filesys_lock);		// 파일 시스템 접근 보호
	file_close(file);					// 마지막 참조라면 파일 객체 정리
	lock_release(&filesys_lock);
}