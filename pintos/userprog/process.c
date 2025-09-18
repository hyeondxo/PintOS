#include "userprog/process.h"
#include "filesys/directory.h"
#include "filesys/file.h"
#include "filesys/filesys.h"
#include "intrinsic.h"
#include "threads/flags.h"
#include "threads/init.h"
#include "threads/interrupt.h"
#include "threads/loader.h"
#include "threads/mmu.h"
#include "threads/palloc.h"
#include "threads/thread.h"
#include "threads/vaddr.h"
#include "userprog/gdt.h"
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

/*
 * Pintos 부트로더가 전달할 수 있는 전체 커맨드 라인 길이는 LOADER_ARGS_LEN(=128)으로 제한된다.
 * 공백으로 구분된 인자 수는 해당 길이의 절반을 넘지 않으므로 안전한 최대 argc를 미리 계산해 둔다.
 */
#define MAX_ARGS (LOADER_ARGS_LEN / 2 + 1)

static void process_cleanup(void);
static bool load(const char *file_name, struct intr_frame *if_);
static void initd(void *f_name);
static void __do_fork(void *);
static void argument_stack(char **argv, int argc, struct intr_frame *if_);

/* initd 및 그 외 프로세스를 위한 공통 초기화 함수
 * - 현재 스레드의 프로세스 레벨 공용 초기화 지점을 제공합니다.
 * - 필요 시 향후 프로세스별 초기화 항목(예: 파일 디스크립터 테이블 초기화)을
 *   이 함수에 추가하면 됩니다. */
static void process_init(void) {
    struct thread *current = thread_current();
#ifdef USERPROG
    current->exit_status = -1; // 사용자 프로세스 기본 종료 코드를 실패(-1)로 시작
    current->running_file = NULL;
#endif
}

/* 첫 번째 유저랜드 프로그램 "initd"를 FILE_NAME에서 로드하여 시작합니다.
 * - init 프로세스 생성의 진입점입니다.
 * - 스레드를 만들어 user 프로그램 로딩 루틴(initd)로 진입하게 합니다.
 * 새 스레드는 스케줄링될 수 있으며(심지어 종료될 수도 있음),
 * process_create_initd()가 반환되기 전에 실행될 수 있습니다.
 * initd의 스레드 id를 반환하고, 생성에 실패하면 TID_ERROR를 반환합니다.
 * 이 함수는 반드시 한 번만 호출되어야 합니다. */
tid_t process_create_initd(const char *file_name) {
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

    /* FILE_NAME의 복사본을 만듭니다.
     * - 호출자가 넘긴 문자열 버퍼의 라이프사이클과 load()의 사용 시점이 겹치지 않도록
     *   별도의 페이지에 안전하게 복사합니다. (경쟁 상태 방지) */
    fn_copy = palloc_get_page(0);
    if (fn_copy == NULL)
        return TID_ERROR;
    strlcpy(fn_copy, file_name, PGSIZE);

    /* FILE_NAME을 실행할 새 스레드를 생성합니다.
     * - 스레드의 시작 함수로 initd를 지정합니다. */
    // 이름이 first_token인 새 스레드를 기본 우선순위로 만들어 initd 함수로부터 실행하되
    // 그 함수의 인자로 준비해 둔 프로그램 문자열 복사본을 넘기는 호출
    // 반환값은 새 스레드의 ID임
    tid = thread_create(first_token, PRI_DEFAULT, initd, fn_copy);
    if (tid == TID_ERROR)
        palloc_free_page(fn_copy);
    return tid;
}

/* 첫 번째 사용자 프로세스를 실행하는 스레드 함수
 * - VM 사용 시 보조 페이지 테이블 초기화
 * - process_exec()를 호출하여 실제 ELF 로드 및 유저 모드 진입 수행 */
static void initd(void *f_name) {
#ifdef VM
    supplemental_page_table_init(&thread_current()->spt);
#endif

    process_init();

    if (process_exec(f_name) < 0)
        PANIC("Fail to launch initd\n");
    NOT_REACHED();
}

/* 현재 프로세스를 `name`으로 복제합니다. 새 프로세스의 스레드 id를 반환하고,
 * 스레드 생성에 실패하면 TID_ERROR를 반환합니다.
 * - 부모의 실행 상태를 참고하여 자식 스레드를 만들고, __do_fork()에서
 *   주소 공간/리소스 복제를 수행합니다. */
tid_t process_fork(const char *name, struct intr_frame *if_ UNUSED) {
    /* 현재 스레드를 새 스레드로 복제합니다.
     * - __do_fork()의 인자로 부모 thread_current()를 넘겨 후속 복제에 사용합니다. */
    return thread_create(name, PRI_DEFAULT, __do_fork, thread_current());
}

#ifndef VM
/* 부모의 주소 공간을 pml4_for_each에 이 함수를 전달하여 복제합니다.
 * 이 함수는 Project 2에서만 사용됩니다.
 * - PML4를 순회하면서 각 유저 페이지를 자식에게 새로 할당/복사하여
 *   동일한 유저 가상 주소 레이아웃을 구성합니다. */
static bool duplicate_pte(uint64_t *pte, void *va, void *aux) {
    struct thread *current = thread_current();
    struct thread *parent = (struct thread *)aux;
    void *parent_page;
    void *newpage;
    bool writable;

    /* 1. TODO: parent_page가 커널 페이지라면 즉시 반환하세요.
     *    - 커널 주소 영역은 사용자 주소 공간 복제 대상이 아닙니다.
     *    - 커널 매핑은 전역(공유)로 유지되므로 별도 복제가 필요 없습니다. */

    /* 2. 부모의 PML4에서 VA를 해석합니다.
     *    - 부모가 VA에 매핑한 실제 커널 물리 프레임(커널 가상주소)을 얻어옵니다. */
    parent_page = pml4_get_page(parent->pml4, va);

    /* 3. TODO: 자식용 PAL_USER 페이지를 새로 할당하고
     *    TODO: 그 결과를 NEWPAGE에 설정하세요.
     *    - 자식에게 독립적인 페이지 프레임을 부여합니다. */

    /* 4. TODO: 부모의 페이지 내용을 새 페이지로 복제하고,
     *    TODO: 부모 페이지가 쓰기 가능한지 여부를 확인하여
     *    TODO: 그 결과에 따라 WRITABLE을 설정하세요.
     *    - memcpy 등으로 내용 전체를 복사합니다.
     *    - pte 비트 또는 보조 API로 쓰기 권한을 확인합니다. */

    /* 5. 자식의 페이지 테이블에 VA 주소로 NEWPAGE를 WRITABLE 권한으로 매핑합니다.
     *    - 자식의 PML4에 동일한 VA로 매핑되어야 부모와 동일한 주소 공간을 이룹니다. */
    if (!pml4_set_page(current->pml4, va, newpage, writable)) {
        /* 6. TODO: 매핑 삽입에 실패한 경우 오류 처리를 수행하세요.
         *    - 할당한 페이지를 해제하고 false를 반환하거나
         *      상위에서 롤백 루틴을 호출할 수 있도록 에러 경로를 구성합니다. */
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
static void __do_fork(void *aux) {
    struct intr_frame if_;
    struct thread *parent = (struct thread *)aux;
    struct thread *current = thread_current();
    /* TODO: parent_if를 전달하는 방법을 마련하세요. (예: process_fork()의 if_)
     * - 시스템 콜 핸들러에서 process_fork(name, &f) 형태로 넘어온 f를
     *   자식이 접근할 수 있게 저장/전달해야 합니다. */
    struct intr_frame *parent_if;
    bool succ = true;

    /* 1. CPU 컨텍스트를 로컬 스택으로 읽어옵니다.
     * - 유저 모드로 복귀할 때 사용할 레지스터 상태 사본입니다.
     * - 자식은 여기를 기반으로 do_iret()을 통해 유저 모드로 돌아갑니다. */
    memcpy(&if_, parent_if, sizeof(struct intr_frame));

    /* 2. 페이지 테이블 복제
     * - 자식의 최상위 PML4 생성 후 활성화
     * - VM 미사용(Project 2)에서는 pml4_for_each + duplicate_pte로 전 페이지 복제
     * - VM 사용(Project 3~)에서는 SPT를 이용해 지연 적재 기반 복제 */
    current->pml4 = pml4_create();
    if (current->pml4 == NULL)
        goto error;

    process_activate(current);
#ifdef VM
    supplemental_page_table_init(&current->spt);
    if (!supplemental_page_table_copy(&current->spt, &parent->spt))
        goto error;
#else
    if (!pml4_for_each(parent->pml4, duplicate_pte, parent))
        goto error;
#endif

    /* TODO: 여기에 코드를 작성하세요.
     * TODO: 힌트) 파일 객체를 복제하기 위해 include/filesys/file.h의
     * TODO:       `file_duplicate`를 사용하세요.
     * TODO:       부모의 자원 복제가 성공하기 전까지 부모는 fork()에서
     * TODO:       반환되면 안 됩니다.
     * - 각 FD에 대해 동일한 파일을 가리키도록 복제(오프셋 공유 여부 주의)
     * - 실행 파일에 대한 deny_write 상태 유지 등도 신경 써야 합니다. */

    process_init();

    /* 마지막으로, 새로 생성된 프로세스로 전환합니다.
     * - if_에 담긴 레지스터 상태로 유저 모드 복귀
     * - 자식은 fork()의 반환값으로 0을 받도록 if_.rax를 조정하는 것이 일반적입니다. */
    if (succ)
        do_iret(&if_);
error:
    thread_exit();
}

/* 현재 실행 컨텍스트를 f_name으로 전환합니다.
 * 실패 시 -1을 반환합니다.
 * - 현재 프로세스의 주소 공간을 파괴하고
 *   새 ELF를 로드하여 같은 스레드가 새 유저 프로그램으로 실행되도록 합니다(exec). */
int process_exec(void *f_name) {
    char *file_name = f_name;
    bool success;
    char *argv[MAX_ARGS];  // 최대 인자 개수만큼 문자열 포인터를 담을 배열
    int argc = 0;          // 현재까지 파싱한 인재 개수 카운트
    char *save_ptr = NULL; // strtok_r이 다음 토큰 위치를 기억할 때 쓰는 상태 변수
    char *token;           // 방금 잘라낸 토큰의 주소를 담을 포인터 변수

    // file_name에서 공백을 기준으로 차례차례 잘라가며 토큰을 꺼냄. 더 이상 토큰이 없으면 루프가 끝남
    for (token = strtok_r(file_name, " ", &save_ptr); token != NULL; token = strtok_r(NULL, " ", &save_ptr)) {
        // 최대 인자를 초과하면 동적 할당했던 커맨드 문자열 페이지를 해제
        if (argc >= MAX_ARGS) {
            palloc_free_page(file_name);
            return -1; // 실패 반환
        }
        // 초과가 아니라면 지금 토큰의 시작 주소를 배열에 저장하고 argc를 1 증가
        argv[argc++] = token;
    }
    // 한 개의 토큰도 나오지 않았을 경우 -> 해제, 실패
    if (argc == 0) {
        palloc_free_page(file_name);
        return -1;
    }

    /* 스레드 구조체에 있는 intr_frame은 사용할 수 없습니다.
     * 현재 스레드가 리스케줄될 때, 실행 정보가 그 멤버에 저장되기 때문입니다.
     * - 지역 intr_frame을 만들어 로딩 결과 레지스터 상태를 담습니다. */
    struct intr_frame _if;
    _if.ds = _if.es = _if.ss = SEL_UDSEG;
    _if.cs = SEL_UCSEG;
    _if.eflags = FLAG_IF | FLAG_MBS; /* 인터럽트 허용 + 반드시 1이어야 하는 비트 */

    thread_current()->exit_status = -1; // exec 직후에도 기본값을 -1로 재설정(이전 상태 잔재 제거)

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
        return -1;
    }

    // 파싱해둔 인자 목록을 바탕으로 사용자 스택과 레지스터를 세팅
    argument_stack(argv, argc, &_if);
    // 커맨드 문자열 버퍼는 더이상 쓸 일이 없음
    palloc_free_page(file_name);

    /* 전환된 프로세스를 시작합니다.
     * - do_iret()은 _if에 적힌 유저 레지스터로 복귀(유저 모드 점프)합니다. */
    do_iret(&_if);
    NOT_REACHED();
}

/* 스레드 TID가 종료될 때까지 기다리고 그 종료 상태를 반환합니다.
 * 커널에 의해 종료된 경우(예: 예외로 인해 kill됨) -1을 반환합니다.
 * TID가 유효하지 않거나 호출 프로세스의 자식이 아니거나,
 * 주어진 TID에 대해 process_wait()가 이미 성공적으로 호출된 경우,
 * 즉시 -1을 반환하고 기다리지 않습니다.
 *
 * 이 함수는 문제 2-2에서 구현합니다. 현재는 아무 동작도 하지 않습니다.
 * - 채점 스크립트가 initd에 대해 wait을 호출할 때 커널이 바로 종료되지 않도록
 *   초기에는 바쁜 대기나 적절한 동기화 객체를 통한 대기가 필요할 수 있습니다. */
int process_wait(tid_t child_tid UNUSED) {
    /* XXX: 힌트) pintos는 process_wait (initd)에서 종료됩니다.
     * XXX:       process_wait을 구현하기 전에는 여기 무한 루프를 넣는 것을 권장합니다. */
    return -1;
}

/* 프로세스를 종료합니다. 이 함수는 thread_exit()에 의해 호출됩니다.
 * - 종료 메시지 출력(format 엄수)
 * - 열린 파일/FD/자식 관계/세마포어 등 정리
 * - 주소 공간 해제(process_cleanup 호출) */
void process_exit(void) {
    struct thread *curr = thread_current();
    if (curr->pml4 != NULL)
        printf("%s: exit(%d)\n", curr->name, curr->exit_status); // 사용자 프로세스 종료 메시지 출력
    process_cleanup();
}

/* 현재 프로세스의 자원을 해제합니다.
 * - 보조 페이지 테이블, PML4, 유저 공간 매핑 제거
 * - 커널 전용 페이지 디렉터리로 안전하게 전환 후 파괴 */
static void process_cleanup(void) {
    struct thread *curr = thread_current();

    /* 실행 중이던 ELF 파일 핸들을 닫아 재사용/삭제 시 충돌을 방지한다. */
    if (curr->running_file != NULL) {
        file_close(curr->running_file);
        curr->running_file = NULL;
    }

#ifdef VM
    supplemental_page_table_kill(&curr->spt);
#endif

    uint64_t *pml4;
    /* 현재 프로세스의 페이지 디렉터리를 파괴하고
     * 커널 전용 페이지 디렉터리로 다시 전환합니다. */
    pml4 = curr->pml4;
    if (pml4 != NULL) {
        /* 아래 순서는 매우 중요합니다.
         * - 스위치 순서가 틀리면 인터럽트 시 다시 해제된 페이지 디렉터리를
         *   활성화하려는 문제가 생길 수 있습니다.
         * 1) 현재 스레드의 pml4 포인터를 NULL로
         * 2) 커널 전용 페이지 테이블을 활성화
         * 3) 마지막으로 이전 pml4를 파괴 */
        curr->pml4 = NULL;
        pml4_activate(NULL);
        pml4_destroy(pml4);
    }
}

/* 다음 스레드에서 유저 코드를 실행할 수 있도록 CPU를 설정합니다.
 * - 문맥 전환 시 호출되며, 해당 스레드의 페이지 테이블 활성화와
 *   TSS(커널 스택 포인터 등) 갱신을 수행합니다. */
void process_activate(struct thread *next) {
    /* 스레드의 페이지 테이블을 활성화합니다. */
    pml4_activate(next->pml4);

    /* 인터럽트 처리를 위한 스레드의 커널 스택을 설정합니다.
     * - TSS: 인터럽트/예외 발생 시 사용할 커널 스택 포인터 저장 */
    tss_update(next);
}

/* 우리는 ELF 바이너리를 로드합니다.
 * 아래 정의들은 [ELF1] 사양에서 거의 그대로 가져왔습니다.
 * - ELF 헤더/프로그램 헤더를 해석해 로드 가능한 세그먼트를 매핑합니다. */

/* ELF 타입. [ELF1] 1-2 참고 */
#define EI_NIDENT 16

#define PT_NULL 0           /* 무시 */
#define PT_LOAD 1           /* 로드 가능한 세그먼트 */
#define PT_DYNAMIC 2        /* 동적 링킹 정보 */
#define PT_INTERP 3         /* 동적 로더의 이름 */
#define PT_NOTE 4           /* 부가 정보 */
#define PT_SHLIB 5          /* 예약됨 */
#define PT_PHDR 6           /* 프로그램 헤더 테이블 */
#define PT_STACK 0x6474e551 /* 스택 세그먼트 */

#define PF_X 1 /* 실행 가능 */
#define PF_W 2 /* 쓰기 가능 */
#define PF_R 4 /* 읽기 가능 */

/* 실행 파일 헤더. [ELF1] 1-4 ~ 1-8 참고
 * ELF 바이너리의 가장 앞부분에 위치합니다. */
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

/* 약어 */
#define ELF ELF64_hdr
#define Phdr ELF64_PHDR

static bool setup_stack(struct intr_frame *if_);
static bool validate_segment(const struct Phdr *, struct file *);
static bool load_segment(struct file *file, off_t ofs, uint8_t *upage, uint32_t read_bytes, uint32_t zero_bytes,
                         bool writable);

/* FILE_NAME에서 ELF 실행 파일을 현재 스레드로 로드합니다.
 * 실행 파일의 진입점은 *RIP에,
 * 초기 스택 포인터는 *RSP에 저장합니다.
 * 성공 시 true, 실패 시 false를 반환합니다.
 * - 주요 단계:
 *   1) 새 PML4 생성 및 활성화
 *   2) ELF 헤더 검증
 *   3) 각 LOAD 세그먼트 매핑(파일에서 읽기 + 0 채우기)
 *   4) 유저 스택(setup_stack) 구성
 *   5) 엔트리 포인트를 rip에 기록
 *   6) (과제) 인자 전달(argument passing) */
static bool load(const char *file_name, struct intr_frame *if_) {
    struct thread *t = thread_current();
    struct ELF ehdr;
    struct file *file = NULL;
    off_t file_ofs;
    bool success = false;
    int i;

    /* 페이지 디렉터리를 할당하고 활성화합니다. */
    t->pml4 = pml4_create();
    if (t->pml4 == NULL)
        goto done;
    process_activate(thread_current());

    /* 실행 파일을 엽니다. */
    file = filesys_open(file_name);
    if (file == NULL) {
        printf("load: %s: open failed\n", file_name);
        goto done;
    }

    /* 실행 파일 헤더를 읽고 검증합니다.
     * - 매직/클래스/머신타입/버전/프로그램 헤더 크기 등을 확인합니다.
     * - 실패 시 잘못된 ELF로 간주하고 로딩을 중단합니다. */
    if (file_read(file, &ehdr, sizeof ehdr) != sizeof ehdr || memcmp(ehdr.e_ident, "\177ELF\2\1\1", 7) ||
        ehdr.e_type != 2 || ehdr.e_machine != 0x3E // amd64
        || ehdr.e_version != 1 || ehdr.e_phentsize != sizeof(struct Phdr) || ehdr.e_phnum > 1024) {
        printf("load: %s: error loading executable\n", file_name);
        goto done;
    }

    /* 프로그램 헤더들을 읽습니다.
     * - 각 프로그램 헤더(세그먼트 설명)를 순회하며 LOAD 타입만 매핑합니다. */
    file_ofs = ehdr.e_phoff;
    for (i = 0; i < ehdr.e_phnum; i++) {
        struct Phdr phdr;

        if (file_ofs < 0 || file_ofs > file_length(file))
            goto done;
        file_seek(file, file_ofs);

        if (file_read(file, &phdr, sizeof phdr) != sizeof phdr)
            goto done;
        file_ofs += sizeof phdr;
        switch (phdr.p_type) {
        case PT_NULL:
        case PT_NOTE:
        case PT_PHDR:
        case PT_STACK:
        default:
            /* 이 세그먼트는 무시합니다. */
            break;
        case PT_DYNAMIC:
        case PT_INTERP:
        case PT_SHLIB:
            /* 동적 링킹/인터프리터/공유 라이브러리 등은 Pintos 범위를 벗어나므로 거부 */
            goto done;
        case PT_LOAD:
            if (validate_segment(&phdr, file)) {
                bool writable = (phdr.p_flags & PF_W) != 0;
                uint64_t file_page = phdr.p_offset & ~PGMASK;
                uint64_t mem_page = phdr.p_vaddr & ~PGMASK;
                uint64_t page_offset = phdr.p_vaddr & PGMASK;
                uint32_t read_bytes, zero_bytes;
                if (phdr.p_filesz > 0) {
                    /* 일반 세그먼트.
                     * - 파일에서 읽어올 바이트 + 나머지 0 채움(초기화 BSS)
                     * - 페이지 경계에 맞춰 read/zero 크기를 계산합니다. */
                    read_bytes = page_offset + phdr.p_filesz;
                    zero_bytes = (ROUND_UP(page_offset + phdr.p_memsz, PGSIZE) - read_bytes);
                } else {
                    /* 전부 0으로 채워야 하는 세그먼트(BSS 등).
                     * - 파일에서 읽을 바이트는 0, 나머지는 모두 0 초기화 */
                    read_bytes = 0;
                    zero_bytes = ROUND_UP(page_offset + phdr.p_memsz, PGSIZE);
                }
                if (!load_segment(file, file_page, (void *)mem_page, read_bytes, zero_bytes, writable))
                    goto done;
            } else
                goto done;
            break;
        }
    }

    /* 스택을 설정합니다.
     * - 최소 1페이지를 USER_STACK 최상단에 매핑하고, rsp를 정상 진입점으로 설정합니다. */
    if (!setup_stack(if_))
        goto done;

    /* 시작 주소를 설정합니다.
     * - ELF 엔트리 포인트를 rip에 적어 유저 모드가 시작될 위치를 지정합니다. */
    if_->rip = ehdr.e_entry;

    success = true;

done:
    if (!success) {
        if (file != NULL)
            file_close(file);
    } else {
        struct thread *current = thread_current();
        /* 성공적으로 적재한 실행 파일은 종료 시까지 열어 두어야 하므로 스레드에 보관한다. */
        current->running_file = file;
    }
    return success;
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

/* PHDR이 FILE 안에서 유효하고 로드 가능한 세그먼트를 기술하는지 검사합니다.
 * 그렇다면 true, 아니면 false를 반환합니다.
 * - 오프셋/주소 정렬, 파일 범위, 크기 관계, 유저 주소 범위 등을 점검합니다. */
static bool validate_segment(const struct Phdr *phdr, struct file *file) {
    /* p_offset과 p_vaddr는 같은 페이지 오프셋을 가져야 합니다.
     * - 파일 오프셋과 메모리 주소가 페이지 단위로 정렬되어 있어야
     *   페이지 매핑 시 일관성을 유지할 수 있습니다. */
    if ((phdr->p_offset & PGMASK) != (phdr->p_vaddr & PGMASK))
        return false;

    /* p_offset은 FILE 내부를 가리켜야 합니다. */
    if (phdr->p_offset > (uint64_t)file_length(file))
        return false;

    /* p_memsz는 p_filesz 이상이어야 합니다.
     * - 메모리에 올릴 크기가 파일에서 읽을 크기보다 작을 수는 없습니다. */
    if (phdr->p_memsz < phdr->p_filesz)
        return false;

    /* 세그먼트는 비어 있으면 안 됩니다. */
    if (phdr->p_memsz == 0)
        return false;

    /* 가상 메모리 영역의 시작과 끝이 모두 사용자 주소 공간 범위에 있어야 합니다. */
    if (!is_user_vaddr((void *)phdr->p_vaddr))
        return false;
    if (!is_user_vaddr((void *)(phdr->p_vaddr + phdr->p_memsz)))
        return false;

    /* 영역이 커널 가상 주소 공간을 넘나들며 래핑되면 안 됩니다. */
    if (phdr->p_vaddr + phdr->p_memsz < phdr->p_vaddr)
        return false;

    /* 페이지 0 매핑은 금지합니다.
       - NULL 포인터 역참조를 조기에 탐지하기 위한 관례적 보호입니다. */
    if (phdr->p_vaddr < PGSIZE)
        return false;

    /* 문제없습니다. */
    return true;
}

#ifndef VM
/* 이 블록의 코드는 Project 2 동안에만 사용됩니다.
 * Project 2 전체를 위해 함수를 구현하려면, #ifndef 매크로 바깥에 구현하세요. */

/* load() 보조 함수
 * - 커널 페이지(kpage)에 파일 내용을 읽어 채우고
 *   해당 페이지를 유저 주소 upage에 매핑합니다. */
static bool install_page(void *upage, void *kpage, bool writable);

/* FILE의 OFS 오프셋에서 시작하는 세그먼트를 주소 UPAGE에 로드합니다.
 * 전체적으로 READ_BYTES + ZERO_BYTES 바이트의 가상 메모리가 초기화되며, 규칙은 다음과 같습니다:
 *
 * - UPAGE에서 시작하는 READ_BYTES 바이트는 FILE의 OFS에서 읽어옵니다.
 * - UPAGE + READ_BYTES 이후의 ZERO_BYTES 바이트는 0으로 채웁니다.
 *
 * WRITABLE이 true이면 사용자 프로세스가 이 페이지들을 수정할 수 있어야 하고,
 * 그렇지 않으면 읽기 전용이어야 합니다.
 *
 * - 페이지 단위로 반복하여 파일을 읽고 남는 부분을 0으로 채운 뒤 매핑합니다. */
static bool load_segment(struct file *file, off_t ofs, uint8_t *upage, uint32_t read_bytes, uint32_t zero_bytes,
                         bool writable) {
    ASSERT((read_bytes + zero_bytes) % PGSIZE == 0);
    ASSERT(pg_ofs(upage) == 0);
    ASSERT(ofs % PGSIZE == 0);

    file_seek(file, ofs);
    while (read_bytes > 0 || zero_bytes > 0) {
        /* 이번 페이지를 어떻게 채울지 계산합니다.
         * FILE에서 PAGE_READ_BYTES 바이트를 읽고,
         * 남은 PAGE_ZERO_BYTES 바이트는 0으로 채웁니다. */
        size_t page_read_bytes = read_bytes < PGSIZE ? read_bytes : PGSIZE;
        size_t page_zero_bytes = PGSIZE - page_read_bytes;

        /* 메모리에서 페이지 하나를 가져옵니다. */
        uint8_t *kpage = palloc_get_page(PAL_USER);
        if (kpage == NULL)
            return false;

        /* 이 페이지를 로드합니다.
         * - 실제 파일 내용 복사 + 남은 공간 0 초기화 */
        if (file_read(file, kpage, page_read_bytes) != (int)page_read_bytes) {
            palloc_free_page(kpage);
            return false;
        }
        memset(kpage + page_read_bytes, 0, page_zero_bytes);

        /* 프로세스의 주소 공간에 페이지를 추가합니다.
         * - 실패 시 kpage를 반드시 해제하여 누수 방지 */
        if (!install_page(upage, kpage, writable)) {
            printf("fail\n");
            palloc_free_page(kpage);
            return false;
        }

        /* 다음으로 진행합니다. */
        read_bytes -= page_read_bytes;
        zero_bytes -= page_zero_bytes;
        upage += PGSIZE;
    }
    return true;
}

/* USER_STACK에 0으로 채워진 페이지를 매핑하여 최소한의 스택을 만듭니다.
 * - 성공 시 rsp를 사용자 스택 최상단으로 설정합니다. */
static bool setup_stack(struct intr_frame *if_) {
    uint8_t *kpage;
    bool success = false;

    kpage = palloc_get_page(PAL_USER | PAL_ZERO);
    if (kpage != NULL) {
        success = install_page(((uint8_t *)USER_STACK) - PGSIZE, kpage, true);
        if (success)
            if_->rsp = USER_STACK;
        else
            palloc_free_page(kpage);
    }
    return success;
}

/* 사용자 가상 주소 UPAGE를 커널 가상 주소 KPAGE에 매핑을 추가합니다.
 * WRITABLE이 true이면 사용자 프로세스가 이 페이지를 수정할 수 있고,
 * 아니면 읽기 전용입니다.
 * UPAGE는 이미 매핑되어 있으면 안 됩니다.
 * KPAGE는 palloc_get_page()로 사용자 풀에서 가져온 페이지여야 합니다.
 * 성공 시 true, UPAGE가 이미 매핑되어 있거나 메모리 할당 실패 시 false를 반환합니다. */
static bool install_page(void *upage, void *kpage, bool writable) {
    struct thread *t = thread_current();

    /* 해당 가상 주소에 이미 페이지가 없는지 확인한 뒤, 페이지를 매핑합니다. */
    return (pml4_get_page(t->pml4, upage) == NULL && pml4_set_page(t->pml4, upage, kpage, writable));
}
#else
/* 여기서부터의 코드는 Project 3 이후에 사용됩니다.
 * Project 2에서만 사용할 함수를 구현하려면 위쪽 블록에 구현하세요.
 * - VM이 켜진 환경에서는 lazy loading(지연 적재)을 통해
 *   페이지 폴트 시 파일 내용을 읽어오는 방식을 사용합니다. */

static bool lazy_load_segment(struct page *page, void *aux) {
    /* TODO: 파일에서 세그먼트를 로드하세요.
     * - aux에는 파일 포인터/오프셋/읽을 길이/0 채울 길이 등 메타데이터를 담아
     *   첫 접근(page fault) 시 실제 디스크 I/O를 수행합니다. */
    /* TODO: 이 함수는 주소 VA에서 첫 페이지 폴트가 발생할 때 호출됩니다. */
    /* TODO: 이 함수를 호출할 때 VA는 유효합니다. */
}

/* FILE의 OFS 오프셋에서 시작하는 세그먼트를 주소 UPAGE에 로드합니다.
 * 전체적으로 READ_BYTES + ZERO_BYTES 바이트의 가상 메모리가 초기화되며, 규칙은 다음과 같습니다:
 *
 * - UPAGE에서 시작하는 READ_BYTES 바이트는 FILE의 OFS에서 읽어옵니다.
 * - UPAGE + READ_BYTES 이후의 ZERO_BYTES 바이트는 0으로 채웁니다.
 *
 * WRITABLE이 true이면 사용자 프로세스가 이 페이지들을 수정할 수 있어야 하고,
 * 그렇지 않으면 읽기 전용이어야 합니다.
 *
 * - VM 경로에서는 실제 디스크 읽기를 즉시 하지 않고,
 *   vm_alloc_page_with_initializer를 통해 lazy_load_segment를 등록합니다. */
static bool load_segment(struct file *file, off_t ofs, uint8_t *upage, uint32_t read_bytes, uint32_t zero_bytes,
                         bool writable) {
    ASSERT((read_bytes + zero_bytes) % PGSIZE == 0);
    ASSERT(pg_ofs(upage) == 0);
    ASSERT(ofs % PGSIZE == 0);

    while (read_bytes > 0 || zero_bytes > 0) {
        /* 이번 페이지를 어떻게 채울지 계산합니다.
         * FILE에서 PAGE_READ_BYTES 바이트를 읽고,
         * 남은 PAGE_ZERO_BYTES 바이트는 0으로 채웁니다. */
        size_t page_read_bytes = read_bytes < PGSIZE ? read_bytes : PGSIZE;
        size_t page_zero_bytes = PGSIZE - page_read_bytes;

        /* TODO: lazy_load_segment에 정보를 전달할 aux를 설정하세요.
         * - 파일 포인터, 현재 오프셋, page_read_bytes, page_zero_bytes, writability 등을
         *   구조체로 묶어 넘기는 것이 일반적입니다. */
        void *aux = NULL;
        if (!vm_alloc_page_with_initializer(VM_ANON, upage, writable, lazy_load_segment, aux))
            return false;

        /* 다음으로 진행합니다. */
        read_bytes -= page_read_bytes;
        zero_bytes -= page_zero_bytes;
        upage += PGSIZE;
    }
    return true;
}

/* USER_STACK에 스택 페이지를 생성합니다. 성공 시 true를 반환합니다.
 * - VM 환경에서는 스택 확장 정책/마커 등을 고려해 스택 페이지를 생성/클레임합니다. */
static bool setup_stack(struct intr_frame *if_) {
    bool success = false;
    void *stack_bottom = (void *)(((uint8_t *)USER_STACK) - PGSIZE);

    /* TODO: stack_bottom에 스택을 매핑하고 즉시 페이지를 확보(claim)하세요.
     * TODO: 성공 시 rsp를 적절히 설정하세요.
     * TODO: 이 페이지가 스택임을 표시해야 합니다.
     * - vm_alloc_page_with_initializer + vm_claim_page 조합 등을 사용합니다. */
    /* TODO: 여기에 코드를 작성하세요 */

    return success;
}
#endif /* VM */
