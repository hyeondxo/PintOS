#include "userprog/process.h"
#include "filesys/directory.h"
#include "filesys/file.h"
#include "filesys/filesys.h"
#include "intrinsic.h"
#include "threads/flags.h"
#include "threads/init.h"
#include "threads/interrupt.h"
#include "threads/loader.h"
#include "threads/malloc.h"
#include "threads/mmu.h"
#include "threads/palloc.h"
#include "threads/synch.h"
#include "threads/thread.h"
#include "threads/vaddr.h"
#include "userprog/gdt.h"
#include "userprog/syscall.h"
#include "userprog/tss.h"
#include <debug.h>
#include <inttypes.h>
#include <round.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef VM
#include "vm/vm.h"
#endif

// 부모가 만든 자식 프로세스 구조체 -> process.c 안에서만 생성, 해제, 활용되므로 process.h에 두지 x
struct child_process {
    tid_t tid;             // 자식의 프로세스/스레드 번호. id
    struct thread *parent; // 이 자식을 만든 부모 스레드 포인터
    struct list_elem elem; // 부모의 자식 리스트에 넣기 위한 연결 리스트 노드
    int exit_status;       // 자식이 exit로 남긴 종료 코드
    bool exited;           // 자식이 정말 종료했는지 여부
    bool waited;           // 부모가 이미 이 자식을 wait했는지 표시. 중복으로 기다리는 일 방지
    int load_success;      // 자식 프로그램이 로딩에 성공했는지. exec/fork 실행 결과를 부모에게 알려줄 때 사용
	/**
     * -> fork로 새 자식 생성 이후 자식의 복제가 완전히 끝나기 전까지 부모를 기다리게 하는 역할
     * 자식이 로드 성공or실패를 확정지을 때까지 load_sema를 down하고 자식이 로드를 끝낸 시점에 up해 깨우면서
     * load_success에 플래그(1/0)을 남긴다. 부모는 깨어난 뒤 load_success를 보고 자식의 결과를 올바르게 확인
     */
    struct semaphore load_sema; // 로드 완료 통보용 (부모는 exec/fork 직후 결과 확정까지 대기)
	/**
     * 자식이 살아있으면 종료될 때까지 블록, 종료되면 exit_status를 받아 리턴
     * 자식이 process_exit으로 종료처리할 때 up되어 부모는 깨어나 exit_status를 읽고 자식을 정리
     */
    struct semaphore wait_sema; // 종료 완료 통보용. (부모는 wait(pid)에서 자식 종료까지 대기)
};

// 새 프로세스를 만들 때 자식 스레드의 시작 함수로 전달할 초기 인자 묶음.
struct exec_args {
    // 부모가 페이지 단위로 안전하게 복사해두고 자식 스레드의 시작에서 이 버퍼를 사용해 넘김
    // 경합을 피하기 위해 부모가 가진 원본 포인터를 쓰지 않고 별도 버퍼로 전달
    char *cmdline; // 유저가 요청한 실행 문자열.
    // 부모 자식 간 연결고리 역할
    struct child_process *child; // 부모가 생성한 struct child_process 노드 포인터
};

// fork에서 자식 스레드의 시작 함수(do fork)로 넘길 부모 컨텍스트 + 동기화 정보 묶음
// 부모의 유저 레지스터 상태와 부모/자식 연결고리를 전달
struct fork_args {
    // 자식은 이를 기반으로 사용자 모드로 부모가 돌아갈 자리에 복귀해야함
    struct intr_frame parent_if; // 부모가 fork를 호출했을 때의 사용자 레지스터 스냅샷
    struct thread *parent;       // 부모 스레드 포인터. 자식 초기화 시 부모의 리소스를 참조하기 위해 사용
    struct child_process *child; // 부모가 미리 만들어둔 자식 프로세스 노드 포인터
};

#define MAX_ARGS (LOADER_ARGS_LEN / 2 + 1)

static void process_cleanup (void);
static bool load (const char *file_name, struct intr_frame *if_);
static void initd (void *f_name);
static void __do_fork (void *);

/* initd 및 다른 프로세스를 위한 공통 초기화자. */
static void
process_init (void) {
	struct thread *current = thread_current ();
#ifdef USERPROG
	current->exit_status = -1; // 사용자 프로세스 기본 종료 코드를 실패(-1)로 시작
	current->running_file = NULL;
	memset(current->fd_table, 0, sizeof current->fd_table); // fd 2~127 슬롯을 모두 비워 새 주소 공간에서 깨끗하게 시작
	list_init(&current->children);                          // 자식 리스트 초기화
	current->child_info = NULL;                             // 부모 child 노드 포인터 초기화
	current->parent = NULL;                                 // 기본 부모 없음
#endif
}

#ifdef USERPROG
// 부모 스레드 parent의 자식 정보를 담을 child_process 노드를 생성해 부모의 자식 리스트에 붙이고 포인터를 반환
static struct child_process *child_process_create(struct thread *parent) {
    struct child_process *child = malloc(sizeof *child); // 자식 메모 구조체 동적 할당
    if (child == NULL)
        return NULL;                                 // 메모리 부족이면 실패
    child->tid = TID_ERROR;                          // 아직 tid 미정 표시
    child->parent = parent;                          // 역참조용 부모 포인터 저장
    child->exit_status = -1;                         // 기본 종료 코드는 -1로 시작
    child->exited = false;                           // 아직 종료되지 않음
    child->waited = false;                           // 부모가 wait하지 않은 상태
    child->load_success = 0;                         // 자식 로드 결과 미확정(0)
    sema_init(&child->load_sema, 0);                 // 로드 완료 통보 세마포어 초기화
    sema_init(&child->wait_sema, 0);                 // 종료 통보 세마포어 초기화
    list_push_back(&parent->children, &child->elem); // 부모의 자식 리스트에 등록
    return child;
}

// 부모의 children 리스트에서 특정 tid 자식을 찾아 반환한다.
static struct child_process *child_process_find(struct thread *parent, tid_t tid) {
    struct list_elem *e;
    for (e = list_begin(&parent->children); e != list_end(&parent->children); e = list_next(e)) {
        struct child_process *child = list_entry(e, struct child_process, elem);
        if (child->tid == tid)
            return child; // 매칭되는 자식 발견 시 즉시 반환
    }
    return NULL; // 찾지 못하면 NULL
}

// 부모 리스트에서 자식 노드를 분리하고 필요 시 메모리를 해제한다.
static void child_process_detach(struct child_process *child) {
    if (child == NULL)
        return; // 이미 없음
    if (child->parent != NULL)
        list_remove(&child->elem); // 부모 children 리스트에서 제거
    child->parent = NULL;          // 더 이상 부모와 연결되지 않음 표시
    if (child->exited || child->waited)
        free(child); // 자식이 종료되었거나 부모가 이미 기다렸다면 메모리 해제
}
#endif

/* 첫 번째 유저랜드 프로그램 "initd"를 FILE_NAME에서 로드하여 시작합니다.
 * - init 프로세스 생성의 진입점입니다.
 * - 스레드를 만들어 user 프로그램 로딩 루틴(initd)로 진입하게 합니다.
 * 새 스레드는 스케줄링될 수 있으며(심지어 종료될 수도 있음),
 * process_create_initd()가 반환되기 전에 실행될 수 있습니다.
 * initd의 스레드 id를 반환하고, 생성에 실패하면 TID_ERROR를 반환합니다.
 * 이 함수는 반드시 한 번만 호출되어야 합니다. */
tid_t
process_create_initd (const char *file_name) {
	char *fn_copy;
	tid_t tid;

	char thread_name[16]; // 스레드 이름을 잠시 담아둘 버퍼
    char *save_ptr;       // strtok_r이 다음 토큰 위치를 기억할 수 있게 해주는 저장용 포인터

    // 원본 문자열 file_name을 위에서 만든 버퍼로 복사
    strlcpy(thread_name, file_name, sizeof thread_name);
    // 복사해둔 문자열을 공백을 기준으로 잘라 첫 번째 토큰을 구함
    char *first_token = strtok_r(thread_name, " ", &save_ptr);
    if (first_token == NULL)
        first_token = thread_name;

	/* FILE_NAME의 사본을 만든다.
	 * 그렇지 않으면 호출자와 load() 사이에 경합이 생긴다. */
	fn_copy = palloc_get_page (0);
	if (fn_copy == NULL)
		return TID_ERROR;
	strlcpy (fn_copy, file_name, PGSIZE);

	// 지금 실행중인 스레드가 부모임. 부모의 children 리스트에 새 자식 노드를 하나 만들어 붙이고 그 포인터를 받음
	struct child_process *child = child_process_create (thread_current ());
	if (child == NULL) {
		palloc_free_page (fn_copy); // 페이지 메모리 해제
		return TID_ERROR;			// 쓰레드 생성 실패 리턴
	}

	// 자식 쓰레드 시작 함수로 넘길 인자 패키지를 커널 힙에 동적 할당
	// exec에 필요한 정보(커맨드 라인, child 노드)
	struct exec_args *args = malloc (sizeof *args);
	if (args == NULL) {
		list_remove (&child->elem);
		free(child);
		palloc_free_page (fn_copy);
		return TID_ERROR;
	}
	
    // 부모가 페이지에 복사해둔 커맨드 라인 문자열을 exec_args에 넣음. 자식 시작 루틴이 process_exec에서 사용
	args->cmdline = fn_copy;
	args->child = child;	// 부모가 만든 child_process 노드 포인터

	/* FILE_NAME을 실행할 새 스레드를 생성합니다. */
    // 새 사용자 스레드 생성
    // 이름은 first_token, 우선순위 기본값, 시작 함수는 initd, args 패키지를 넘김
	tid = thread_create (first_token, PRI_DEFAULT, initd, args);
	if (tid == TID_ERROR) {
		free(args);
		list_remove(&child->elem);
		free(child);
		palloc_free_page(fn_copy);
		return TID_ERROR;
	}

	child->tid = tid;	// 성공적으로 스레드가 만들어졌으므로 자식 노드에 실제 tid 기록
    // 자식 결과 대기. 부모는 여기서 블록되어 자식 측의 로드 성공or실패 통보를 기다림.
    // 자식은 initd->process exec 경로에서 끝난 뒤 sema_up을 호출
	sema_down (&child->load_sema);

	// 자식의 로드가 실패
	if (!child->load_success) {
		child_process_detach (child); // 부모 리스트에서 노드를 떼기
		return TID_ERROR;
	}

	return tid;
}

/* 첫 사용자 프로세스를 구동하는 스레드 함수. */
static void
initd (void *aux) {
#ifdef VM
	supplemental_page_table_init (&thread_current ()->spt);
#endif

	process_init ();

	// 부모가 thread_create로 넘겨준 인자를 exec_args로 해석 - command line buffer, child pointer

	struct exec_args *args = aux;
	struct child_process *child = args->child;	// 부모가 미리 만들어 둔 자식 추적용 노드를 꺼냄 
	char *file_name = args->cmdline;

	// 현재 쓰레드 = 자기 자신 = child
	thread_current()->child_info = child;
	if (child != NULL) {							// child가 유효하면 부모자식 연결
		thread_current()->parent = child->parent;	// 나(자식)의 부모 기록
		child->tid = thread_current()->tid;			// 자식 tid 기록
	}
	free(args);		// exec_args는 이제 불필요

	// 현재 쓰레드의 주소 공간 초기화, file_name에 해당하는 실행 파일 로드한 뒤 인자 스택 구성
	// 실패 시 -1 반환, 성공 시 유저 모드로 점프하여 돌아오지 않음
	if (process_exec (file_name) < 0) {
		thread_current()->exit_status = -1;
		thread_exit();
	}
	NOT_REACHED ();
}

/* 현재 프로세스를 `name`으로 복제한다. 새 프로세스의 스레드 ID를
 * 반환하고, 스레드를 생성할 수 없으면 TID_ERROR를 반환한다. */
tid_t
process_fork (const char *name, struct intr_frame *if_) {
#ifdef USERPROG
	struct thread *parent = thread_current ();						// 현재 쓰레드는 부모 쓰레드가 됨
	struct child_process *child = child_process_create (parent);	// 자식 생성
	if (child == NULL) return TID_ERROR;
	
	// 자식 스레드로 전달할 인자 묶음 동적 할당
	struct fork_args *args = malloc (sizeof *args);
	if (args == NULL) {
		list_remove (&child->elem);
		free(child);
		return TID_ERROR;
	}

	// 부모의 사용자 레지스터 스냅샷(intr_frame)을 자식으로 넘길 준비
	// 자식은 이를 기반으로 fork 호출 직후 지점으로 복귀
	memcpy(&args->parent_if, if_, sizeof(struct intr_frame));
	args->parent = parent;	// 자식이 부모 리소스 복제에 접근할 수 있는 포인터
	args->child = child;	// 부모가 만든 child_process 노드
 
	// 자식 스레드 생성
	// 스레드 이름은 name, 우선순위는 부모와 같고 시작 함수는 do_fork, 인자는 args
	tid_t tid = thread_create (name, parent->priority, __do_fork, args);
	if (tid == TID_ERROR) {
		free(args);
		list_remove (&child->elem);
		free(child);
		return TID_ERROR;
	}

	child->tid = tid;	// 성공적으로 생성된 자식의 tid를 child에 기록 - 부모가 참조
	// 부모는 여기서 블록, 자식의 fork 준비 완료 신호 기다림. 자식 쪽 do_fork에서 성공/실패 설정하고 up
	sema_down (&child->load_sema);
	if (!child->load_success) {
		child_process_detach (child);
		return TID_ERROR;
	}
	// 성공했을 경우 부모 컨텍스트에서 자식 tid 반환
	return tid;
#else // USERPROG가 아닐 때 더미 처리로 오류 반환
	(void)name;
	(void)if_;
	return TID_ERROR;
#endif
}

#ifndef VM
/* 부모의 주소 공간을 pml4_for_each에 이 함수를 전달하여 복제합니다.
 * 이 함수는 Project 2에서만 사용됩니다.
 * - PML4를 순회하면서 각 유저 페이지를 자식에게 새로 할당/복사하여
 *   동일한 유저 가상 주소 레이아웃을 구성합니다. */
static bool
duplicate_pte (uint64_t *pte, void *va, void *aux) {
    // 지금 복제를 수행중인 자식 스레드의 포인터. 이 자식의 pml4(페이지 테이블)에 새 매핑을 추가해야 함
    struct thread *current = thread_current();
    // 부모의 주소공간에서 원본 페이지를 조회하기 위한부모 스레드 포인터
    struct thread *parent = (struct thread *)aux;
    void *parent_page; // 부모 주소공간에서 va를 해석했을 때 반환되는 부모 페이지의 커널 가상주소를 담을 변수
    void *newpage;     // 자식에게 새로 할당할 사용자 페이지 프레임. 부모 페이지 내용응 memcpy로 복사
    bool writable;     // 해당 va가 쓰기 가능한 페이지인지 여부를 저장. 쓰기 비트를 확인하고 설정

    /* 1. TODO: parent_page가 커널 페이지라면 즉시 반환하세요.
     *    - 커널 주소 영역은 사용자 주소 공간 복제 대상이 아닙니다.
     *    - 커널 매핑은 전역(공유)로 유지되므로 별도 복제가 필요 없습니다. */
	if (is_kernel_vaddr(va)) return true;

    /* 2. 부모의 PML4에서 VA를 해석합니다.
     *    - 부모가 VA에 매핑한 실제 커널 물리 프레임(커널 가상주소)을 얻어옵니다. */
	parent_page = pml4_get_page (parent->pml4, va);
	if (parent_page == NULL) return true;

    /* 3. TODO: 자식용 PAL_USER 페이지를 새로 할당하고
     *    TODO: 그 결과를 NEWPAGE에 설정하세요.
     *    - 자식에게 독립적인 페이지 프레임을 부여합니다. */
	newpage = palloc_get_page(PAL_USER);
	if (newpage == NULL) return false;

    /* 4. TODO: 부모의 페이지 내용을 새 페이지로 복제하고,
     *    TODO: 부모 페이지가 쓰기 가능한지 여부를 확인하여
     *    TODO: 그 결과에 따라 WRITABLE을 설정하세요.
     *    - memcpy 등으로 내용 전체를 복사합니다.
     *    - pte 비트 또는 보조 API로 쓰기 권한을 확인합니다. */
	memcpy(newpage, parent_page, PGSIZE);
	writable = (*pte & PTE_W) != 0;

    /* 5. 자식의 페이지 테이블에 VA 주소로 NEWPAGE를 WRITABLE 권한으로 매핑합니다.
     *    - 자식의 PML4에 동일한 VA로 매핑되어야 부모와 동일한 주소 공간을 이룹니다. */
	if (!pml4_set_page (current->pml4, va, newpage, writable)) {
		/* 6. TODO: 매핑 삽입에 실패한 경우 오류 처리를 수행하세요.
		*    - 할당한 페이지를 해제하고 false를 반환하거나
		*      상위에서 롤백 루틴을 호출할 수 있도록 에러 경로를 구성합니다. */
		palloc_free_page(newpage);
		return false;
	}
	return true;
}
#endif

/* 부모의 실행 컨텍스트를 복사하는 스레드 함수.
 * 힌트) parent->tf는 사용자 영역의 컨텍스트를 가지고 있지 않습니다.
 *       즉, 이 함수에는 process_fork()의 두 번째 인자를 전달해야 합니다.
 * - 여기서 하는 일:
 *   1) 부모 유저 컨텍스트(intr_frame) 복사
 *   2) 자식용 페이지 테이블 및 보조 구조 복제
 *   3) 파일 등 커널 리소스 복제/공유 설정
 *   4) 준비가 끝나면 do_iret로 유저 모드 진입 */
static void
__do_fork (void *aux) {
    struct fork_args *args = aux;              // 부모가 전달한 fork 인자 묶음
    struct intr_frame if_;                     // 자식이 사용할 레지스터 복사본
    struct thread *parent = args->parent;      // 부모 스레드 포인터
    struct child_process *child = args->child; // 부모가 만든 child_process 노드
    struct thread *current = thread_current(); // 현재(자식) 스레드
    bool succ = true;                          // 복제 성공 여부

	memcpy (&if_, &args->parent_if, sizeof (struct intr_frame));// 부모 레지스터 스냅샷 복사
	free (args);												// 전달용 구조체 해제

#ifdef VM
	supplemental_page_table_init (&current->spt);
#endif

	/* TODO: 여기에 코드를 작성하라.
	 * TODO: 힌트) 파일 객체를 복제하려면 include/filesys/file.h의
	 * TODO:       `file_duplicate`를 사용하라. 이 함수가 부모의
	 * TODO:       자원을 성공적으로 복제할 때까지 부모는 fork()에서
	 * TODO:       반환하면 안 된다.*/

	process_init ();				// 자식 스레드의 프로세스 상태 초기화
	current->parent = parent;		// 부모 포인터 연결
	current->child_info = child;	// 로드/종료 통보용 child 노드 연결
	if (child != NULL) 
		child->tid = current->tid;	// child 노드에 자식 tid 기록

	current->pml4 = pml4_create();	// 자식용 페이지 테이블 생성
	if (current->pml4 == NULL)
		succ = false;				// 실패 시 플래그 내림
	else
		process_activate(current);	// 새 주소 공간 활성화

#ifdef VM
	if (!supplemental_page_table_copy (&current->spt, &parent->spt))
		succ = false;

#else
	if (!pml4_for_each (parent->pml4, duplicate_pte, parent))
		succ = false;
#endif

	// 주소공간 복제가 완료된 경우에만 파일 핸들 복제 진행
	if (succ) {
		lock_acquire(&filesys_lock);					// 파일 시스템 자원 접근 시 전역 동기화 확보
		for (int fd = 2; fd < FD_TABLE_SIZE; fd++) {	// fd 0,1(STDIN/OUT) 제외 부모의 열려 있는 fd 순회
			struct file *pf = parent->fd_table[fd];		// 부모 fd 슬롯 확인
			if (pf != NULL) {							// 실제로 열려있는 파일이면
				struct file *dup = file_duplicate(pf);	// 동일 inode를 참조하는 새 file 구조체 생성 (pos/deny_write 복제)
				if (dup == NULL) {
					succ = false;
					break;
				}
				current->fd_table[fd] = dup;			// 자식 fd 테이블의 동일 번호 슬롯에 저장
			}
		}

		// 부모가 실행 파일 핸들을 보유 중이면
		if (succ && parent->running_file != NULL) {
			current->running_file = file_duplicate (parent->running_file);	// 자식도 실행 파일 핸들 복제
			if (current->running_file == NULL) succ = false;	// 복제 실패 시 오류 처리
		}
		lock_release (&filesys_lock);	// 파일 복제 작업 끝냈으니 락 해제
	}

	// 복제 도중 실패하면 자원 정리 후 실패 알림
	if (!succ) {
		lock_acquire(&filesys_lock);	// 파일 핸들 저리 시 전역 락 확보
		for (int fd = 2; fd < FD_TABLE_SIZE; fd++) {
			if (current->fd_table[fd] != NULL) {	// 복제된 파일 핸들 닫기
				file_close (current->fd_table[fd]);
				current->fd_table[fd] = NULL;
			}
		}
		if (current->running_file != NULL) {	// 실행 파일 핸들도 정리
			file_close (current->running_file);
			current->running_file = NULL;
		}
		lock_release (&filesys_lock);
		if (child != NULL) {
			child->load_success = 0;		// 부모에게 실패 통보 플래그 설정
			sema_up (&child->load_sema);	// 부모 깨우기
		}
		thread_exit();	// 자식 쓰레드 종료 -> 부모는 -1 반환 
	}

	if_.R.rax = 0; /* 자식은 fork()에서 0 반환 */

    if (child != NULL) {
        child->load_success = 1; // 성공 시 부모에게 알림
        sema_up(&child->load_sema);
    }

    do_iret(&if_); // 사용자 모드로 복귀 (자식 실행 시작)
}

/**
 * exec가 성공한 직후 사용자 프로그래밍 기대하는 방식에 맞춰 스택과 레지스터를 정리한다
 */
static void argument_stack(char **argv, int argc, struct intr_frame *if_) {
    // 방금 스택에 복사한 각 문자열의 시작 주소를 기억해두기 위한 임시 배열 나중에 argv 테이블을 만들 때 사용
    char *arg_addr[MAX_ARGS];

    // 인자를 뒤에서부터 하나씩 꺼내 스택에 복사
    for (int i = argc - 1; i >= 0; --i) {
        // rsp를 문자열 길이(+NULL)만큼 내리고, 그 위치에 argv[i] 문자열을 그대로 복사
        size_t len = strlen(argv[i]) + 1;
        if_->rsp -= len;
        memcpy((void *)if_->rsp, argv[i], len);
        // 복사된 위치의 주소를 arg_addr[i]로 기억한다
        arg_addr[i] = (char *)if_->rsp;
    }

    // 현재 rsp가 8의 배수가 될 때까지 한 바이트씩 0을 채워 넣는다
    while (if_->rsp % 8 != 0) {
        if_->rsp -= 1;
        *(uint8_t *)if_->rsp = 0;
    }

    // argv[argc] = NULL 규칙을 지키기 위해 포인터 하나를 더 push
    if_->rsp -= sizeof(char *);
    *(char **)(if_->rsp) = NULL;

    // 마찬가지로 역순으로 진행하며 arg_addr에 저장해 둔 각 문자열 주소를 스택에 push
    for (int i = argc - 1; i >= 0; --i) {
        if_->rsp -= sizeof(char *);
        *(char **)(if_->rsp) = arg_addr[i];
    }

    // 결과는 현재 rsp가 argv[0] 위치를 가리키게 되고 나중에 레지스터에 넣어줄 값이 됨
    char **argv_base = (char **)if_->rsp;

    // main 함수가 return했을 때 돌아갈 곳이 없으므로, 0(Null 주소)로 채워 사용자가 잘못된 리턴을 시도했을 때 곧바로
    // 예외가 발생하도록 한다
    if_->rsp -= sizeof(void *);
    *(void **)(if_->rsp) = 0;

    // 유저 mode 진입 시 argc/argv는 RDI/RSI 레지스터로 전달된다
    // 호출 규약에 맞게 첫 번째 인자(argc)는 RDI, 두 번째 인자(argv)는 RSI에 넣는다
    if_->R.rdi = (uint64_t)argc;
    if_->R.rsi = (uint64_t)argv_base;
}

/* 현재 실행 컨텍스트를 f_name으로 전환한다.
 * 실패하면 -1을 반환한다. */
int process_exec(void *f_name) {
    char *file_name = f_name;
    bool success;
    char *argv[MAX_ARGS];  // 최대 인자 개수만큼 문자열 포인터를 담을 배열
    int argc = 0;          // 현재까지 파싱한 인재 개수 카운트
    char *save_ptr = NULL; // strtok_r이 다음 토큰 위치를 기억할 때 쓰는 상태 변수
    char *token;           // 방금 잘라낸 토큰의 주소를 담을 포인터 변수
	struct child_process *child = thread_current()->child_info;  // ★ 부모 통보용 핸들
    // file_name에서 공백을 기준으로 차례차례 잘라가며 토큰을 꺼냄. 더 이상 토큰이 없으면 루프가 끝남
    for (token = strtok_r(file_name, " ", &save_ptr); token != NULL;
								 token = strtok_r(NULL, " ", &save_ptr)) {
        // 최대 인자를 초과하면 동적 할당했던 커맨드 문자열 페이지를 해제
        if (argc >= MAX_ARGS) {
            palloc_free_page(file_name);
			if (child) { child->load_success = 0; sema_up(&child->load_sema); }
            return -1; // 실패 반환
        }
        // 초과가 아니라면 지금 토큰의 시작 주소를 배열에 저장하고 argc를 1 증가
        argv[argc++] = token;
    }
    // 한 개의 토큰도 나오지 않았을 경우 -> 해제, 실패
    if (argc == 0) {
        palloc_free_page(file_name);
		if (child) { child->load_success = 0; sema_up(&child->load_sema); }
        return -1;
    }

    /* 스레드 구조체에 있는 intr_frame은 사용할 수 없습니다.
     * 현재 스레드가 리스케줄될 때, 실행 정보가 그 멤버에 저장되기 때문입니다.
     * - 지역 intr_frame을 만들어 로딩 결과 레지스터 상태를 담습니다. */
    struct intr_frame _if;
    _if.ds = _if.es = _if.ss = SEL_UDSEG;
    _if.cs = SEL_UCSEG;
    _if.eflags = FLAG_IF | FLAG_MBS; /* 인터럽트 허용 + 반드시 1이어야 하는 비트 */
	
	struct thread *t = thread_current();
	if (t->running_file != NULL) {
		lock_acquire(&filesys_lock);
		file_allow_write(t->running_file);
		file_close(t->running_file);
		lock_release(&filesys_lock);
		t->running_file = NULL;
	}

    /* 먼저 현재 컨텍스트를 정리합니다.
     * - 기존 주소 공간/리소스 파괴(유저 공간 해제) */
    process_cleanup();

    /* 그 다음 바이너리를 로드합니다.
     * - load()는 ELF를 검사하고, 세그먼트를 매핑하고, 초기 스택을 구성합니다. */
    // 파싱한 인자 중 첫 번째 항목인 프로그램 이름을 넘겨 ELF 실행 파일을 로드하고
    // 결과 레지스터 값을 _if에 채움
    success = load(argv[0], &_if);

    if (!success) {
        palloc_free_page(file_name);
		if (child) { child->load_success = 0; sema_up(&child->load_sema); }
        return -1;
    }

    // 파싱해둔 인자 목록을 바탕으로 사용자 스택과 레지스터를 세팅
    argument_stack(argv, argc, &_if);

    // hex_dump(_if.rsp, _if.rsp, USER_STACK - (uint64_t)*&_if.rsp, true);
	
    // 커맨드 문자열 버퍼는 더이상 쓸 일이 없음
    palloc_free_page(file_name);
 	if (child) { child->load_success = 1; sema_up(&child->load_sema); }
    /* 전환된 프로세스를 시작합니다.
     * - do_iret()은 _if에 적힌 유저 레지스터로 복귀(유저 모드 점프)합니다. */
    do_iret(&_if);
    NOT_REACHED();
}


/* 스레드 TID가 종료될 때까지 기다렸다가 그 종료 상태를 반환한다.
 * 커널에 의해 종료되었으면(예: 예외로 인해 kill된 경우) -1을 반환한다.
 * TID가 유효하지 않거나 호출 프로세스의 자식이 아니거나,
 * 혹은 해당 TID에 대해 process_wait()가 이미 성공적으로 호출된 경우에는
 * 기다리지 않고 즉시 -1을 반환한다.
 *
 * 이 함수는 문제 2-2에서 구현된다. 지금은 아무 것도 하지 않는다. */
int
process_wait (tid_t child_tid UNUSED) {
#ifdef USERPROG
	struct thread *curr = thread_current();								// 현재 쓰레드 = 부모
	struct child_process *child = child_process_find(curr, child_tid);	// 자식 목록에서 대상 검색
	if (child == NULL) return -1;		// 내 자식이 아님
	if (child->waited) return -1;		// 이미 기다림

	child->waited = true;				// 중복 wait 방지
	if (!child->exited) sema_down (&child->wait_sema);	// 아직 살아있으면 종료될 때까지 대기
	int status = child->exit_status;					// 자식이 남긴 종료 코드 획득

	list_remove (&child->elem);		// 부모 리스트에서 제거
	child->parent = NULL;			// 부모 연결 끊기
	free(child);					// 추적 구조체 해제
	return status;					// 종료 코드 반환

#else
	(void)child_tid;
	return -1;
#endif
}

/* 프로세스를 종료합니다. 이 함수는 thread_exit()에 의해 호출됩니다.
 * - 종료 메시지 출력(format 엄수)
 * - 열린 파일/FD/자식 관계/세마포어 등 정리
 * - 주소 공간 해제(process_cleanup 호출) */
void
process_exit (void) {
	struct thread *curr = thread_current ();
	if (curr->pml4 != NULL)
		printf("%s: exit(%d)\n", curr->name, curr->exit_status);	// 사용자 프로세스 종료 메세지 출력

#ifdef USERPROG
	for (int fd = 2; fd < FD_TABLE_SIZE; fd++) {
		struct file *file = curr->fd_table[fd];
		if (file != NULL) {
			lock_acquire(&filesys_lock);
			file_close(file);		// 마지막 참조라면 inode write counter가 감소하면서 정리됨
			lock_release(&filesys_lock);
			curr->fd_table[fd] = NULL;	// 계정에서 fd 슬롯 비우기 (중복 close 방지)
		}
	}

	if (curr->running_file != NULL) {
		lock_acquire(&filesys_lock);
		file_allow_write(curr->running_file);
		file_close(curr->running_file);
		lock_release(&filesys_lock);
		curr->running_file = NULL;
	}

	struct child_process *self = curr->child_info;
	if (self != NULL) {
		self->exit_status = curr->exit_status;
		self->exited = true;
		if (self->parent != NULL)
			sema_up(&self->wait_sema);
		else
			free(self);
		curr->child_info = NULL;
	}

	struct list_elem *e = list_begin(&curr->children);
	while (e != list_end(&curr->children)) {
		struct child_process *child = list_entry(e, struct child_process, elem);
		e = list_next(e);				// 다음 노드 미리 잡아둠
		list_remove(&child->elem);		// 나의 children 리스트에서 제거
		child->parent = NULL;			// 자식 입장에서 이제 부모 없음
		if (child->exited || child->waited) // 자식이 종료했거나 부모가 wait했으면
			free(child);					// child_process 구조체 해제
	}

#endif
	process_cleanup ();
}

/* 현재 프로세스의 자원을 해제한다. */
static void
process_cleanup (void) {
	struct thread *curr = thread_current ();

#ifdef VM
	supplemental_page_table_kill (&curr->spt);
#endif

	uint64_t *pml4;
	/* 현재 프로세스의 페이지 디렉터리를 파괴하고
	 * 커널 전용 페이지 디렉터리로 다시 전환한다. */
	pml4 = curr->pml4;
	if (pml4 != NULL) {
		/* 여기서의 올바른 순서는 매우 중요하다. 타이머 인터럽트가
		 * 프로세스 페이지 디렉터리로 다시 전환하지 않도록
		 * 페이지 디렉터리를 전환하기 전에 cur->pagedir을 NULL로
		 * 설정해야 한다. 또한 프로세스의 페이지 디렉터리를 파괴하기 전에
		 * 기본 페이지 디렉터리를 활성화해야 한다. 그렇지 않으면
		 * 우리가 활성화한 페이지 디렉터리가 이미 해제(그리고 초기화)된
		 * 것이 될 수 있다. */
		curr->pml4 = NULL;
		pml4_activate (NULL);
		pml4_destroy (pml4);
	}
}

/* 다음 스레드에서 사용자 코드를 실행할 수 있도록 CPU를 설정한다.
 * 이 함수는 매 컨텍스트 스위치 때 호출된다. */
void
process_activate (struct thread *next) {
	/* 스레드의 페이지 테이블을 활성화한다. */
	pml4_activate (next->pml4);

	/* 인터럽트 처리를 위한 스레드의 커널 스택을 설정한다. */
	tss_update (next);
}

/* 우리는 ELF 바이너리를 로드한다.
 * 아래 정의들은 ELF 스펙 [ELF1]에서 거의 그대로 가져왔다.  */

/* ELF 타입. [ELF1] 1-2 참고. */
#define EI_NIDENT 16

#define PT_NULL    0            /* 무시. */
#define PT_LOAD    1            /* 적재 가능한 세그먼트. */
#define PT_DYNAMIC 2            /* 동적 로딩 정보. */
#define PT_INTERP  3            /* 동적 로더의 이름. */
#define PT_NOTE    4            /* 보조 정보. */
#define PT_SHLIB   5            /* 예약됨. */
#define PT_PHDR    6            /* 프로그램 헤더 테이블. */
#define PT_STACK   0x6474e551   /* 스택 세그먼트. */

#define PF_X 1          /* 실행 가능. */
#define PF_W 2          /* 쓰기 가능. */
#define PF_R 4          /* 읽기 가능. */

/* 실행 파일 헤더. [ELF1] 1-4 ~ 1-8 참고.
 * ELF 바이너리의 맨 앞에 위치한다. */
struct ELF64_hdr {
	unsigned char e_ident[EI_NIDENT];
	uint16_t e_type;
	uint16_t e_machine;
	uint32_t e_version;
	uint64_t e_entry;
	uint64_t e_phoff;
	uint64_t e_shoff;
	uint32_t e_flags;
	uint16_t e_ehsize;
	uint16_t e_phentsize;
	uint16_t e_phnum;
	uint16_t e_shentsize;
	uint16_t e_shnum;
	uint16_t e_shstrndx;
};

struct ELF64_PHDR {
	uint32_t p_type;
	uint32_t p_flags;
	uint64_t p_offset;
	uint64_t p_vaddr;
	uint64_t p_paddr;
	uint64_t p_filesz;
	uint64_t p_memsz;
	uint64_t p_align;
};

/* 축약형 */
#define ELF ELF64_hdr
#define Phdr ELF64_PHDR

static bool setup_stack (struct intr_frame *if_);
static bool validate_segment (const struct Phdr *, struct file *);
static bool load_segment (struct file *file, off_t ofs, uint8_t *upage,
		uint32_t read_bytes, uint32_t zero_bytes,
		bool writable);

/* FILE_NAME에서 ELF 실행 파일을 현재 스레드로 로드한다.
 * 실행 파일의 진입점은 *RIP에 저장하고,
 * 초기 스택 포인터는 *RSP에 저장한다.
 * 성공 시 true, 실패 시 false를 반환한다. */
static bool
load (const char *file_name, struct intr_frame *if_) {
	struct thread *t = thread_current ();
	struct ELF ehdr;
	struct file *file = NULL;
	off_t file_ofs;
	bool success = false;
	int i;
	bool filesys_locked = false;

	/* 페이지 디렉터리를 할당하고 활성화한다. */
	t->pml4 = pml4_create ();
	if (t->pml4 == NULL)
		goto done;
	process_activate (thread_current ());

	/* 실행 파일을 연다. */
	lock_acquire(&filesys_lock);
	filesys_locked = true;
	file = filesys_open (file_name);
	if (file == NULL) {
		printf ("load: %s: open failed\n", file_name);
		goto done;
	}

	/* 실행 파일 헤더를 읽고 검증한다. */
	if (file_read (file, &ehdr, sizeof ehdr) != sizeof ehdr
			|| memcmp (ehdr.e_ident, "\177ELF\2\1\1", 7)
			|| ehdr.e_type != 2
			|| ehdr.e_machine != 0x3E // amd64
			|| ehdr.e_version != 1
			|| ehdr.e_phentsize != sizeof (struct Phdr)
			|| ehdr.e_phnum > 1024) {
		printf ("load: %s: error loading executable\n", file_name);
		goto done;
	}

	/* 프로그램 헤더들을 읽는다. */
	file_ofs = ehdr.e_phoff;
	for (i = 0; i < ehdr.e_phnum; i++) {
		struct Phdr phdr;

		if (file_ofs < 0 || file_ofs > file_length (file))
			goto done;
		file_seek (file, file_ofs);

		if (file_read (file, &phdr, sizeof phdr) != sizeof phdr)
			goto done;
		file_ofs += sizeof phdr;
		switch (phdr.p_type) {
			case PT_NULL:
			case PT_NOTE:
			case PT_PHDR:
			case PT_STACK:
			default:
				/* 이 세그먼트는 무시한다. */
				break;
			case PT_DYNAMIC:
			case PT_INTERP:
			case PT_SHLIB:
				goto done;
			case PT_LOAD:
				if (validate_segment (&phdr, file)) {
					bool writable = (phdr.p_flags & PF_W) != 0;
					uint64_t file_page = phdr.p_offset & ~PGMASK;
					uint64_t mem_page = phdr.p_vaddr & ~PGMASK;
					uint64_t page_offset = phdr.p_vaddr & PGMASK;
					uint32_t read_bytes, zero_bytes;
					if (phdr.p_filesz > 0) {
						/* 일반적인 세그먼트.
						 * 처음 부분은 디스크에서 읽고 나머지는 0으로 채운다. */
						read_bytes = page_offset + phdr.p_filesz;
						zero_bytes = (ROUND_UP (page_offset + phdr.p_memsz, PGSIZE)
								- read_bytes);
					} else {
						/* 전부 0으로 채운다.
						 * 디스크에서 아무 것도 읽지 않는다. */
						read_bytes = 0;
						zero_bytes = ROUND_UP (page_offset + phdr.p_memsz, PGSIZE);
					}
					if (!load_segment (file, file_page, (void *) mem_page,
								read_bytes, zero_bytes, writable))
						goto done;
				}
				else
					goto done;
				break;
		}
	}

	/* 스택을 설정한다. */
	if (!setup_stack (if_))
		goto done;

	/* 시작 주소. */
	if_->rip = ehdr.e_entry;

	/* TODO: 여기에 코드를 작성하라.
	 * TODO: 인자 전달(Argument Passing)을 구현하라
	 * TODO: (project2/argument_passing.html 참고). 
	 * process_exec에서 구현 */

	success = true;

done:
	/* 로드의 성공 여부와 상관없이 여기로 온다. */
	if (filesys_locked) {								// 파일 open시 획득한 락이 아직 유지 중
		if (!success) {									// 로드 실패 경로
			if (file != NULL) file_close(file);			// 열린 실행 파일 닫아 정리
		} else {
			struct thread *current = thread_current();
			file_deny_write(file);						// 실행 중 자기 수정을 막기 위해 쓰기 금지
			current->running_file = file;				// 종료 시까지 유지할 실행 파일 핸들 보관
		}
		lock_release (&filesys_lock);					// 락 해제
	} else if (!success && file != NULL) {				// 락 없이 실패한 경우도 파일은 닫아야 함
		file_close(file);
	}
	return success;		// 전체 로드 성공 여부 반환
}


/* PHDR가 FILE에서 유효하고 로드 가능한 세그먼트를 기술하는지 확인하고,
 * 그렇다면 true, 아니면 false를 반환한다. */
static bool
validate_segment (const struct Phdr *phdr, struct file *file) {
	/* p_offset과 p_vaddr는 동일한 페이지 오프셋을 가져야 한다. */
	if ((phdr->p_offset & PGMASK) != (phdr->p_vaddr & PGMASK))
		return false;

	/* p_offset은 FILE 내부를 가리켜야 한다. */
	if (phdr->p_offset > (uint64_t) file_length (file))
		return false;

	/* p_memsz는 p_filesz보다 크거나 같아야 한다. */
	if (phdr->p_memsz < phdr->p_filesz)
		return false;

	/* 세그먼트는 비어 있으면 안 된다. */
	if (phdr->p_memsz == 0)
		return false;

	/* 가상 메모리 영역의 시작과 끝은 모두 사용자 주소 공간 범위
	   내에 있어야 한다. */
	if (!is_user_vaddr ((void *) phdr->p_vaddr))
		return false;
	if (!is_user_vaddr ((void *) (phdr->p_vaddr + phdr->p_memsz)))
		return false;

	/* 영역이 커널 가상 주소 공간을 가로질러 "랩어라운드"되면 안 된다. */
	if (phdr->p_vaddr + phdr->p_memsz < phdr->p_vaddr)
		return false;

	/* 페이지 0 매핑 금지.
	   페이지 0을 매핑하는 것은 나쁜 아이디어일 뿐만 아니라,
	   만약 허용한다면 사용자 코드가 널 포인터를 시스템 콜에 전달했을 때
	   memcpy() 등의 널 포인터 어서션을 통해 커널 패닉이 날 가능성이 높다. */
	if (phdr->p_vaddr < PGSIZE)
		return false;

	/* 문제 없음. */
	return true;
}

#ifndef VM
/* 이 블록의 코드는 오직 프로젝트 2에서만 사용된다.
 * 프로젝트 전체를 위해 함수를 구현하려면 #ifndef 매크로 밖에서 구현하라. */

/* load() 보조 함수들. */
static bool install_page (void *upage, void *kpage, bool writable);

/* FILE의 OFS 오프셋에서 시작하는 세그먼트를 주소 UPAGE에 로드한다.
 * 총 READ_BYTES + ZERO_BYTES 바이트의 가상 메모리가 다음과 같이 초기화된다:
 *
 * - UPAGE에서 시작하는 READ_BYTES 바이트는 FILE의 OFS에서 읽는다.
 *
 * - UPAGE + READ_BYTES에서 시작하는 ZERO_BYTES 바이트는 0으로 채운다.
 *
 * WRITABLE이 true이면 사용자 프로세스가 이 페이지들을 쓸 수 있어야 하며,
 * 그렇지 않으면 읽기 전용이어야 한다.
 *
 * 성공 시 true, 메모리 할당 오류 또는 디스크 읽기 오류 시 false를 반환한다. */
static bool
load_segment (struct file *file, off_t ofs, uint8_t *upage,
		uint32_t read_bytes, uint32_t zero_bytes, bool writable) {
	ASSERT ((read_bytes + zero_bytes) % PGSIZE == 0);
	ASSERT (pg_ofs (upage) == 0);
	ASSERT (ofs % PGSIZE == 0);

	file_seek (file, ofs);
	while (read_bytes > 0 || zero_bytes > 0) {
		/* 이 페이지를 어떻게 채울지 계산한다.
		 * FILE에서 PAGE_READ_BYTES 바이트를 읽고
		 * 마지막 PAGE_ZERO_BYTES 바이트를 0으로 채운다. */
		size_t page_read_bytes = read_bytes < PGSIZE ? read_bytes : PGSIZE;
		size_t page_zero_bytes = PGSIZE - page_read_bytes;

		/* 메모리 페이지를 하나 얻는다. */
		uint8_t *kpage = palloc_get_page (PAL_USER);
		if (kpage == NULL)
			return false;

		/* 이 페이지를 로드한다. */
		if (file_read (file, kpage, page_read_bytes) != (int) page_read_bytes) {
			palloc_free_page (kpage);
			return false;
		}
		memset (kpage + page_read_bytes, 0, page_zero_bytes);

		/* 페이지를 프로세스의 주소 공간에 추가한다. */
		if (!install_page (upage, kpage, writable)) {
			printf("fail\n");
			palloc_free_page (kpage);
			return false;
		}

		/* 진행. */
		read_bytes -= page_read_bytes;
		zero_bytes -= page_zero_bytes;
		upage += PGSIZE;
	}
	return true;
}

/* USER_STACK에 0으로 채워진 페이지를 매핑하여 최소한의 스택을 만든다 */
static bool
setup_stack (struct intr_frame *if_) {
	uint8_t *kpage;
	bool success = false;

	kpage = palloc_get_page (PAL_USER | PAL_ZERO);
	if (kpage != NULL) {
		success = install_page (((uint8_t *) USER_STACK) - PGSIZE, kpage, true);
		if (success)
			if_->rsp = USER_STACK;
		else
			palloc_free_page (kpage);
	}
	return success;
}

/* 사용자 가상 주소 UPAGE에서 커널 가상 주소 KPAGE로의 매핑을
 * 페이지 테이블에 추가한다.
 * WRITABLE이 true이면 사용자 프로세스가 페이지를 수정할 수 있고,
 * 그렇지 않으면 읽기 전용이다.
 * UPAGE는 이미 매핑되어 있지 않아야 한다.
 * KPAGE는 palloc_get_page()로 사용자 풀에서 얻은 페이지여야 한다.
 * 성공 시 true, UPAGE가 이미 매핑되어 있거나 메모리 할당에 실패하면 false를 반환한다. */
static bool
install_page (void *upage, void *kpage, bool writable) {
	struct thread *t = thread_current ();

	/* 해당 가상 주소에 이미 페이지가 없는지 확인한 뒤, 거기에 우리의 페이지를 매핑한다. */
	return (pml4_get_page (t->pml4, upage) == NULL
			&& pml4_set_page (t->pml4, upage, kpage, writable));
}
#else
/* 여기부터는 프로젝트 3 이후에 사용되는 코드이다.
 * 프로젝트 2에 대해서만 함수를 구현하려면 위쪽 블록에서 구현하라. */

static bool
lazy_load_segment (struct page *page, void *aux) {
	/* TODO: 파일에서 세그먼트를 로드한다 */
	/* TODO: 이 함수는 주소 VA에서 첫 페이지 폴트가 발생했을 때 호출된다. */
	/* TODO: 이 함수를 호출할 때 VA는 사용 가능하다. */
}

/* FILE의 OFS 오프셋에서 시작하는 세그먼트를 주소 UPAGE에 로드한다.
 * 총 READ_BYTES + ZERO_BYTES 바이트의 가상 메모리가 다음과 같이 초기화된다:
 *
 * - UPAGE에서 시작하는 READ_BYTES 바이트는 FILE의 OFS에서 읽는다.
 *
 * - UPAGE + READ_BYTES에서 시작하는 ZERO_BYTES 바이트는 0으로 채운다.
 *
 * WRITABLE이 true이면 사용자 프로세스가 이 페이지들을 쓸 수 있어야 하며,
 * 그렇지 않으면 읽기 전용이어야 한다.
 *
 * 성공 시 true, 메모리 할당 오류 또는 디스크 읽기 오류 시 false를 반환한다. */
static bool
load_segment (struct file *file, off_t ofs, uint8_t *upage,
		uint32_t read_bytes, uint32_t zero_bytes, bool writable) {
	ASSERT ((read_bytes + zero_bytes) % PGSIZE == 0);
	ASSERT (pg_ofs (upage) == 0);
	ASSERT (ofs % PGSIZE == 0);

	while (read_bytes > 0 || zero_bytes > 0) {
		/* 이 페이지를 어떻게 채울지 계산한다.
		 * FILE에서 PAGE_READ_BYTES 바이트를 읽고
		 * 마지막 PAGE_ZERO_BYTES 바이트를 0으로 채운다. */
		size_t page_read_bytes = read_bytes < PGSIZE ? read_bytes : PGSIZE;
		size_t page_zero_bytes = PGSIZE - page_read_bytes;

		/* TODO: lazy_load_segment에 정보를 전달하기 위해 aux를 설정한다. */
		void *aux = NULL;
		if (!vm_alloc_page_with_initializer (VM_ANON, upage,
					writable, lazy_load_segment, aux))
			return false;

		/* 진행. */
		read_bytes -= page_read_bytes;
		zero_bytes -= page_zero_bytes;
		upage += PGSIZE;
	}
	return true;
}

/* USER_STACK에 스택 PAGE를 생성한다. 성공 시 true를 반환한다. */
static bool
setup_stack (struct intr_frame *if_) {
	bool success = false;
	void *stack_bottom = (void *) (((uint8_t *) USER_STACK) - PGSIZE);

	/* TODO: stack_bottom에 스택을 매핑하고 즉시 페이지를 확보(claim)한다.
	 * TODO: 성공하면 rsp를 적절히 설정한다.
	 * TODO: 해당 페이지를 스택으로 표시해야 한다. */
	/* TODO: 여기에 코드를 작성하라 */

	return success;
}
#endif /* VM */
