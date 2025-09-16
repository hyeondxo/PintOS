#ifndef THREADS_VADDR_H
#define THREADS_VADDR_H

#include <debug.h>
#include <stdbool.h>
#include <stdint.h>

#include "threads/loader.h"

/* 가상 주소(Virtual Address)를 다루는 함수와 매크로 정의
 *
 * 하드웨어 페이지 테이블(x86의 PTE 관련 매크로/함수)은 pte.h에 있습니다.
 * 여기서는 주로 가상 주소를 페이지 단위로 처리하기 위한
 * 공통적인 계산 도구들을 제공합니다.
 */

/* 특정 비트 구간만 1로 만든 마스크 생성 매크로
 * - SHIFT: 시작 비트 위치
 * - CNT  : 1로 채울 비트 개수
 * 예) BITMASK(3, 5) = 0001 1111 000 (비트 3~7이 1)
 */
#define BITMASK(SHIFT, CNT) (((1ul << (CNT)) - 1) << (SHIFT))

/* 페이지 오프셋 관련 상수들 (하위 12비트) */
#define PGSHIFT 0                       /* 페이지 오프셋의 첫 번째 비트 인덱스 */
#define PGBITS 12                       /* 오프셋으로 사용하는 비트 수 (2^12 = 4096) */
#define PGSIZE (1 << PGBITS)            /* 한 페이지의 크기 (4096바이트 = 4KB) */
#define PGMASK BITMASK(PGSHIFT, PGBITS) /* 오프셋 부분만 추출하기 위한 마스크 */

/* 주어진 가상 주소 va의 페이지 내부 오프셋 추출
 * - va & 0xFFF (즉, 하위 12비트) */
#define pg_ofs(va) ((uint64_t)(va) & PGMASK)

/* 주어진 가상 주소 va가 전체 메모리 공간에서 몇 번째 페이지에 속하는지 계산
 * - va >> 12 (페이지 크기 4096 단위로 나눈 몫) */
#define pg_no(va) ((uint64_t)(va) >> PGBITS)

/* 가상 주소 va를 다음 페이지 경계로 올림
 * - 예) va = 0x401234 → 0x402000 */
#define pg_round_up(va) ((void *)(((uint64_t)(va) + PGSIZE - 1) & ~PGMASK))

/* 가상 주소 va를 현재 페이지 경계로 내림
 * - 예) va = 0x401234 → 0x401000 */
#define pg_round_down(va) (void *)((uint64_t)(va) & ~PGMASK)

/* 커널 가상 메모리 시작 주소
 * - LOADER_KERN_BASE는 loader.h에서 정의됨
 * - Pintos의 커널 가상 메모리 영역은 이 주소 이상에서 시작함 */
#define KERN_BASE LOADER_KERN_BASE

/* 사용자 스택의 시작 주소 (최상단 주소)
 * - 유저 프로그램의 rsp 초기값으로 사용됨
 * - 스택은 "높은 주소 → 낮은 주소" 방향으로 자라납니다. */
#define USER_STACK 0x47480000

/* VADDR이 유저 가상 주소 공간에 속하면 true 반환
 * - 커널 영역보다 낮은 주소이면 유저 영역 */
#define is_user_vaddr(vaddr) (!is_kernel_vaddr((vaddr)))

/* VADDR이 커널 가상 주소 공간에 속하면 true 반환
 * - KERN_BASE 이상이면 커널 주소 */
#define is_kernel_vaddr(vaddr) ((uint64_t)(vaddr) >= KERN_BASE)

// FIXME: 추가적인 검증 로직 필요
/* 물리 주소 PADDR을 커널 가상 주소로 변환
 * - Pintos는 커널 주소 공간에서 물리 메모리를 1:1로 매핑하지만,
 *   KERN_BASE를 기준으로 offset을 둠
 * - 즉, 커널 가상 주소 = 물리 주소 + KERN_BASE */
#define ptov(paddr) ((void *)(((uint64_t)paddr) + KERN_BASE))

/* 커널 가상 주소 VADDR을 대응되는 물리 주소로 변환
 * - 반드시 커널 주소임을 ASSERT로 확인
 * - 물리 주소 = 커널 가상 주소 - KERN_BASE
 * - 커널에서만 사용 가능 (유저 주소 변환 불가) */
#define vtop(vaddr)                                                                                                    \
    ({                                                                                                                 \
        ASSERT(is_kernel_vaddr(vaddr));                                                                                \
        ((uint64_t)(vaddr) - (uint64_t)KERN_BASE);                                                                     \
    })

#endif /* threads/vaddr.h */