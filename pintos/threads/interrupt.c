#include "threads/interrupt.h"
#include "devices/timer.h"
#include "intrinsic.h"
#include "threads/flags.h"
#include "threads/intr-stubs.h"
#include "threads/io.h"
#include "threads/mmu.h"
#include "threads/thread.h"
#include "threads/vaddr.h"
#include <debug.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#ifdef USERPROG
#include "userprog/gdt.h"
#endif

/* x86_64에서 사용 가능한 인터럽트 벡터 개수 */
#define INTR_CNT 256

/* FUNCTION을 호출하는 게이트 디스크립터를 생성한다.

   게이트는 DPL(Descriptor Privilege Level)을 가지며, CPU가 그 링(또는 더 높은 권한의 링)에
   있을 때 의도적으로 호출될 수 있음을 의미한다. 실무적으로 DPL==3이면 유저 모드에서 호출 가능,
   DPL==0이면 유저 모드에서 직접 호출은 불가하다. 다만, 유저 모드에서 발생한 Fault/Exception은
   DPL==0 게이트로도 진입한다.

   TYPE은 14(인터럽트 게이트) 또는 15(트랩 게이트)여야 한다. 차이는 인터럽트 게이트는 진입 시
   IF(인터럽트 플래그)를 클리어하여 인터럽트를 비활성화하고, 트랩 게이트는 비활성화하지 않는다.
   자세한 내용은 [IA32-v3a] 5.12.1.2를 참고. */

struct gate {
    unsigned off_15_0 : 16;  // 핸들러 주소 오프셋 하위 16비트
    unsigned ss : 16;        // 세그먼트 셀렉터(CS)
    unsigned ist : 3;        // IST 인덱스(보통 0)
    unsigned rsv1 : 5;       // 예약(0)
    unsigned type : 4;       // 14: interrupt gate, 15: trap gate
    unsigned s : 1;          // 시스템 디스크립터(반드시 0)
    unsigned dpl : 2;        // 권한 레벨(DPL)
    unsigned p : 1;          // Present 비트
    unsigned off_31_16 : 16; // 핸들러 주소 오프셋 상위(31:16)
    uint32_t off_32_63;
    uint32_t rsv2;
};

/* 인터럽트 디스크립터 테이블(IDT).
   형식은 CPU에 의해 고정되어 있음. 참고: [IA32-v3a] 5.10, 5.11, 5.12.1.2 */
static struct gate idt[INTR_CNT];

static struct desc_ptr idt_desc = {
    .size = sizeof(idt) - 1, // IDT 바이트 크기 - 1
    .address = (uint64_t)idt // IDT 베이스 주소
};

#define make_gate(g, function, d, t)                                                                                   \
    {                                                                                                                  \
        ASSERT((function) != NULL);                                                                                    \
        ASSERT((d) >= 0 && (d) <= 3);                                                                                  \
        ASSERT((t) >= 0 && (t) <= 15);                                                                                 \
        *(g) = (struct gate){                                                                                          \
            .off_15_0 = (uint64_t)(function) & 0xffff,                                                                 \
            .ss = SEL_KCSEG,                                                                                           \
            .ist = 0,                                                                                                  \
            .rsv1 = 0,                                                                                                 \
            .type = (t),                                                                                               \
            .s = 0,                                                                                                    \
            .dpl = (d),                                                                                                \
            .p = 1,                                                                                                    \
            .off_31_16 = ((uint64_t)(function) >> 16) & 0xffff,                                                        \
            .off_32_63 = ((uint64_t)(function) >> 32) & 0xffffffff,                                                    \
            .rsv2 = 0,                                                                                                 \
        };                                                                                                             \
    }

/* 주어진 DPL로 FUNCTION을 호출하는 인터럽트 게이트 생성 */
#define make_intr_gate(g, function, dpl) make_gate((g), (function), (dpl), 14)

/* 주어진 DPL로 FUNCTION을 호출하는 트랩 게이트 생성 */
#define make_trap_gate(g, function, dpl) make_gate((g), (function), (dpl), 15)

/* 각 인터럽트 벡터에 대한 C 레벨 핸들러 함수 포인터 테이블 */
static intr_handler_func *intr_handlers[INTR_CNT];

/* 디버깅용 인터럽트 이름 테이블 */
static const char *intr_names[INTR_CNT];

/* 외부 인터럽트(타이머 등 장치에서 발생)는 인터럽트 비활성 상태에서 처리되며 중첩되지 않는다.
   외부 인터럽트 핸들러는 수면할 수 없고, 필요 시 intr_yield_on_return()으로 리턴 직전 양보를
   요청할 수 있다. */
static bool in_external_intr; /* 현재 외부 인터럽트 처리 중인지 */
static bool yield_on_return;  /* 리턴 직전에 스케줄러 양보할지 */

/* 8259A PIC 관련 헬퍼 */
static void pic_init(void);
static void pic_end_of_interrupt(int irq);

/* 공용 인터럽트 핸들러(어셈블리 스텁에서 호출) */
void intr_handler(struct intr_frame *args);

/*
 * intr_get_level: 현재 인터럽트 상태를 반환한다(INTR_ON 또는 INTR_OFF).
 * - RFLAGS의 IF 비트를 읽어 판정한다.
 */
enum intr_level intr_get_level(void) {
    uint64_t flags;

    /* PUSHFQ/POPQ로 RFLAGS를 읽는다. 참고: [IA32-v2b] PUSHF/POP, [IA32-v3a] 5.8.1 */
    asm volatile("pushfq; popq %0" : "=g"(flags));

    return flags & FLAG_IF ? INTR_ON : INTR_OFF;
}

/*
 * intr_set_level: LEVEL에 따라 인터럽트를 켜거나 끄고, 이전 상태를 반환한다.
 */
enum intr_level intr_set_level(enum intr_level level) { return level == INTR_ON ? intr_enable() : intr_disable(); }

/*
 * intr_enable: 인터럽트를 허용(STI)하고, 이전 인터럽트 상태를 반환한다.
 * - 외부 인터럽트 처리 컨텍스트에서는 호출하면 안 된다.
 */
enum intr_level intr_enable(void) {
    enum intr_level old_level = intr_get_level();
    ASSERT(!intr_context());

    /* STI: IF 비트 세트 → 인터럽트 허용. 참고: [IA32-v2b] STI, [IA32-v3a] 5.8.1 */
    asm volatile("sti");

    return old_level;
}

/*
 * intr_disable: 인터럽트를 비허용(CLI)하고, 이전 인터럽트 상태를 반환한다.
 */
enum intr_level intr_disable(void) {
    enum intr_level old_level = intr_get_level();

    /* CLI: IF 비트 클리어 → 인터럽트 비활성. 참고: [IA32-v2b] CLI */
    asm volatile("cli" : : : "memory");

    return old_level;
}

/*
 * intr_init: 인터럽트 서브시스템 초기화.
 * - PIC 초기화 및 리맵(0x20~0x2f)
 * - IDT 엔트리 초기화 및 로드(lidt)
 * - (USERPROG) TSS 로드
 * - 잘 알려진 예외 이름 설정
 */
void intr_init(void) {
    int i;

    /* Initialize interrupt controller. */
    pic_init();

    /* Initialize IDT. */
    for (i = 0; i < INTR_CNT; i++) {
        make_intr_gate(&idt[i], intr_stubs[i], 0);
        intr_names[i] = "unknown";
    }

#ifdef USERPROG
    /* Load TSS. */
    ltr(SEL_TSS);
#endif

    /* Load IDT register. */
    lidt(&idt_desc);

    /* Initialize intr_names. */
    intr_names[0] = "#DE Divide Error";
    intr_names[1] = "#DB Debug Exception";
    intr_names[2] = "NMI Interrupt";
    intr_names[3] = "#BP Breakpoint Exception";
    intr_names[4] = "#OF Overflow Exception";
    intr_names[5] = "#BR BOUND Range Exceeded Exception";
    intr_names[6] = "#UD Invalid Opcode Exception";
    intr_names[7] = "#NM Device Not Available Exception";
    intr_names[8] = "#DF Double Fault Exception";
    intr_names[9] = "Coprocessor Segment Overrun";
    intr_names[10] = "#TS Invalid TSS Exception";
    intr_names[11] = "#NP Segment Not Present";
    intr_names[12] = "#SS Stack Fault Exception";
    intr_names[13] = "#GP General Protection Exception";
    intr_names[14] = "#PF Page-Fault Exception";
    intr_names[16] = "#MF x87 FPU Floating-Point Error";
    intr_names[17] = "#AC Alignment Check Exception";
    intr_names[18] = "#MC Machine-Check Exception";
    intr_names[19] = "#XF SIMD Floating-Point Exception";
}

/*
 * register_handler: 인터럽트 벡터에 핸들러를 등록(내부 헬퍼).
 * - vec_no: 벡터 번호
 * - dpl: 게이트 권한 레벨(0~3)
 * - level: 핸들러 진입 시 인터럽트 상태(INTR_ON/OFF)
 * - handler: C 핸들러 함수
 * - name: 디버깅용 이름
 */
static void register_handler(uint8_t vec_no, int dpl, enum intr_level level, intr_handler_func *handler,
                             const char *name) {
    ASSERT(intr_handlers[vec_no] == NULL);
    if (level == INTR_ON) {
        make_trap_gate(&idt[vec_no], intr_stubs[vec_no], dpl);
    } else {
        make_intr_gate(&idt[vec_no], intr_stubs[vec_no], dpl);
    }
    intr_handlers[vec_no] = handler;
    intr_names[vec_no] = name;
}

/*
 * intr_register_ext: 외부 인터럽트(0x20~0x2f) 핸들러 등록.
 * - 인터럽트 비활성 상태(INTR_OFF)로 실행되도록 설정한다.
 */
void intr_register_ext(uint8_t vec_no, intr_handler_func *handler, const char *name) {
    ASSERT(vec_no >= 0x20 && vec_no <= 0x2f);
    register_handler(vec_no, 0, INTR_OFF, handler, name);
}

/*
 * intr_register_int: 내부 인터럽트(예외/트랩) 핸들러 등록.
 * - level: 핸들러 진입 시 인터럽트 상태(INTR_ON/OFF)
 * - dpl: 게이트 권한(3이면 유저 모드에서 의도적 호출 가능)
 */
void intr_register_int(uint8_t vec_no, int dpl, enum intr_level level, intr_handler_func *handler, const char *name) {
    ASSERT(vec_no < 0x20 || vec_no > 0x2f);
    register_handler(vec_no, dpl, level, handler, name);
}

/*
 * intr_context: 외부 인터럽트 처리 중이면 true, 그렇지 않으면 false.
 */
bool intr_context(void) { return in_external_intr; }

/*
 * intr_yield_on_return: 외부 인터럽트 처리 중에만 사용.
 * - 리턴 직전에 스케줄러에 양보하도록 플래그를 세운다.
 */
void intr_yield_on_return(void) {
    ASSERT(intr_context());
    yield_on_return = true;
}

/* 8259A 프로그래머블 인터럽트 컨트롤러(PIC). */

/* 모든 PC에는 두 개의 8259A PIC가 있다. 하나는 0x20/0x21 포트로 접근 가능한 마스터,
   다른 하나는 마스터의 IRQ2에 캐스케이드된 슬레이브로 0xa0/0xa1 포트로 접근한다.
   0x20 포트 접근은 A0 라인을 0으로, 0x21 포트 접근은 A1 라인을 1로 설정한다. 슬레이브도 유사하다.

   기본값으로 PIC가 전달하는 인터럽트 0..15는 벡터 0..15로 매핑되는데, 이는 CPU 트랩/예외와 충돌한다.
   따라서 PIC를 재설정하여 인터럽트 0..15가 벡터 32..47(0x20..0x2f)로 전달되도록 리맵한다. */

/* PIC 초기화. 상세 동작은 [8259A] 문서 참조. */
static void pic_init(void) {
    /* 두 PIC의 모든 IRQ를 일단 마스킹 */
    outb(0x21, 0xff);
    outb(0xa1, 0xff);

    /* 마스터 초기화 */
    outb(0x20, 0x11); /* ICW1: 단일 모드, 엣지 트리거, ICW4 예상 */
    outb(0x21, 0x20); /* ICW2: IR0..7 -> 벡터 0x20..0x27 */
    outb(0x21, 0x04); /* ICW3: 슬레이브 PIC는 IR2에 연결 */
    outb(0x21, 0x01); /* ICW4: 8086 모드, 일반 EOI, 비버퍼드 */

    /* 슬레이브 초기화 */
    outb(0xa0, 0x11); /* ICW1: 단일 모드, 엣지 트리거, ICW4 예상 */
    outb(0xa1, 0x28); /* ICW2: IR0..7 -> 벡터 0x28..0x2f */
    outb(0xa1, 0x02); /* ICW3: 슬레이브 ID = 2 */
    outb(0xa1, 0x01); /* ICW4: 8086 모드, 일반 EOI, 비버퍼드 */

    /* 모든 IRQ 언마스킹 */
    outb(0x21, 0x00);
    outb(0xa1, 0x00);
}

/*
 * pic_end_of_interrupt: 주어진 IRQ에 대해 PIC에 EOI(End Of Interrupt) 신호 전송.
 * - ACK하지 않으면 동일 IRQ가 다시 전달되지 않으므로 반드시 호출해야 한다.
 */
static void pic_end_of_interrupt(int irq) {
    ASSERT(irq >= 0x20 && irq < 0x30);

    /* 마스터 PIC 승인(EOI) */
    outb(0x20, 0x20);

    /* 슬레이브 IRQ이면 슬레이브 PIC 승인 */
    if (irq >= 0x28)
        outb(0xa0, 0x20);
}
/* 인터럽트 핸들러들 */

/*
 * intr_handler: 모든 인터럽트/예외의 공용 진입점.
 * - intr-stubs.S가 호출하며, frame에는 벡터/에러코드/레지스터 스냅샷이 담긴다.
 * - 외부 인터럽트는 중첩 금지이며, PIC에 EOI를 전송하고 필요 시 양보한다.
 */
void intr_handler(struct intr_frame *frame) {
    bool external;
    intr_handler_func *handler;

    /* 외부 인터럽트는 특별 취급:
       - 한 번에 하나만 처리(인터럽트 비활성 상태 유지)
       - PIC에 승인(EOI) 필요
       - 외부 인터럽트 핸들러는 수면 불가 */
    external = frame->vec_no >= 0x20 && frame->vec_no < 0x30;
    if (external) {
        ASSERT(intr_get_level() == INTR_OFF);
        ASSERT(!intr_context());

        in_external_intr = true;
        yield_on_return = false;
    }

    /* 인터럽트 핸들러 호출 */
    handler = intr_handlers[frame->vec_no];
    if (handler != NULL)
        handler(frame);
    else if (frame->vec_no == 0x27 || frame->vec_no == 0x2f) {
        /* 핸들러 없음. 하드웨어 결함/레이스 등으로 스퍼리어스 가능 → 무시 */
    } else {
        /* 핸들러도 없고 스퍼리어스도 아님 → 예기치 않은 인터럽트 처리(패닉) */
        intr_dump_frame(frame);
        PANIC("Unexpected interrupt");
    }

    /* 외부 인터럽트 처리 마무리 */
    if (external) {
        ASSERT(intr_get_level() == INTR_OFF);
        ASSERT(intr_context());

        in_external_intr = false;
        pic_end_of_interrupt(frame->vec_no);

        if (yield_on_return)
            thread_yield();
    }
}

/*
 * intr_dump_frame: 인터럽트 프레임을 콘솔로 출력(디버깅용).
 * - CR2(마지막 페이지 폴트의 선형 주소)와 레지스터들을 표시한다.
 */
void intr_dump_frame(const struct intr_frame *f) {
	/* CR2는 마지막 페이지 폴트의 선형 주소.
	   참고: [IA32-v2a] MOV 제어레지스터, [IA32-v3a] 5.14 (#PF) */
    uint64_t cr2 = rcr2();
    printf("Interrupt %#04llx (%s) at rip=%llx\n", f->vec_no, intr_names[f->vec_no], f->rip);
    printf(" cr2=%016llx error=%16llx\n", cr2, f->error_code);
    printf("rax %016llx rbx %016llx rcx %016llx rdx %016llx\n", f->R.rax, f->R.rbx, f->R.rcx, f->R.rdx);
    printf("rsp %016llx rbp %016llx rsi %016llx rdi %016llx\n", f->rsp, f->R.rbp, f->R.rsi, f->R.rdi);
    printf("rip %016llx r8 %016llx  r9 %016llx r10 %016llx\n", f->rip, f->R.r8, f->R.r9, f->R.r10);
    printf("r11 %016llx r12 %016llx r13 %016llx r14 %016llx\n", f->R.r11, f->R.r12, f->R.r13, f->R.r14);
    printf("r15 %016llx rflags %08llx\n", f->R.r15, f->eflags);
    printf("es: %04x ds: %04x cs: %04x ss: %04x\n", f->es, f->ds, f->cs, f->ss);
}

/*
 * intr_name: 인터럽트 벡터 VEC의 이름을 반환.
 */
const char *intr_name(uint8_t vec) { return intr_names[vec]; }
