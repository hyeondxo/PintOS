#ifndef THREADS_INTERRUPT_H
#define THREADS_INTERRUPT_H

#include <stdbool.h>
#include <stdint.h>

/* 인터럽트 허용 상태 표현
   - INTR_ON: 인터럽트 허용(FLAG_IF=1)
   - INTR_OFF: 인터럽트 비허용(FLAG_IF=0) */
enum intr_level {
	INTR_OFF,             /* 인터럽트 비활성 */
	INTR_ON               /* 인터럽트 활성 */
};

/* 현재 RFLAGS의 IF 비트를 읽어 인터럽트 상태를 반환 */
enum intr_level intr_get_level (void);
/* LEVEL(INTR_ON/OFF)에 따라 인터럽트 상태를 설정하고, 이전 상태를 반환 */
enum intr_level intr_set_level (enum intr_level);
/* STI: 인터럽트 활성, 이전 상태 반환 */
enum intr_level intr_enable (void);
/* CLI: 인터럽트 비활성, 이전 상태 반환 */
enum intr_level intr_disable (void);

/* 인터럽트 스택 프레임(일반 레지스터 묶음)
   - intr-stubs.S에서 저장/복원되는 레이아웃과 일치해야 함
   - packed로 컴파일러 패딩을 금지하여 정확한 메모리 배치를 보장 */
struct gp_registers {
	uint64_t r15;
	uint64_t r14;
	uint64_t r13;
	uint64_t r12;
	uint64_t r11;
	uint64_t r10;
	uint64_t r9;
	uint64_t r8;
	uint64_t rsi;
	uint64_t rdi;
	uint64_t rbp;
	uint64_t rdx;
	uint64_t rcx;
	uint64_t rbx;
	uint64_t rax;
} __attribute__((packed));

struct intr_frame {
	/* intr-stubs.S의 intr_entry에서 스택에 저장되는 영역
	   - 인터럽트 진입 시 보존된 일반 레지스터 값들 */
	struct gp_registers R;
	uint16_t es;
	uint16_t __pad1;
	uint32_t __pad2;
	uint16_t ds;
	uint16_t __pad3;
	uint32_t __pad4;
	/* intr-stubs.S의 각 벡터 스텁(intrNN_stub)이 저장 */
	uint64_t vec_no; /* 인터럽트 벡터 번호 */
/* 에러코드: 일부 예외에서만 CPU가 자동 푸시(그 외엔 스텁이 0을 푸시)
   - CPU는 본래 eip(여기서는 rip) 바로 아래에 두지만, 편의상 구조 내로 옮겨 보관 */
	uint64_t error_code;
/* CPU가 자동으로 푸시하는 영역
   - RIP/CS/RFLAGS(/RSP/SS): 인터럽트 직전의 실행 컨텍스트 */
	uintptr_t rip;
	uint16_t cs;
	uint16_t __pad5;
	uint32_t __pad6;
	uint64_t eflags;
	uintptr_t rsp;
	uint16_t ss;
	uint16_t __pad7;
	uint32_t __pad8;
} __attribute__((packed));

typedef void intr_handler_func (struct intr_frame *);

/* 인터럽트 서브시스템 초기화(PIC/IDT 설정 등) */
void intr_init (void);
/* 외부 인터럽트(0x20~0x2f) 핸들러 등록: 항상 인터럽트 비활성 상태로 진입 */
void intr_register_ext (uint8_t vec, intr_handler_func *, const char *name);
/* 내부 인터럽트/예외 핸들러 등록: DPL/진입 인터럽트 상태(level) 지정 가능 */
void intr_register_int (uint8_t vec, int dpl, enum intr_level,
                        intr_handler_func *, const char *name);
/* 외부 인터럽트 처리 중 여부(true/false) */
bool intr_context (void);
/* 외부 인터럽트 리턴 직전에 스케줄러에 양보하도록 표시 */
void intr_yield_on_return (void);

/* intr_frame 내용을 콘솔로 덤프(디버깅) */
void intr_dump_frame (const struct intr_frame *);
/* 벡터 번호 → 이름 */
const char *intr_name (uint8_t vec);

#endif /* threads/interrupt.h */
