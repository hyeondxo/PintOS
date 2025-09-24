#ifndef USERPROG_SYSCALL_H
#define USERPROG_SYSCALL_H

void syscall_init (void);

#include "threads/synch.h"
extern struct lock filesys_lock;    // 파일 시스템 API 호출 직렬화를 위한 전역 락

void sys_exit(int status);

#endif /* userprog/syscall.h */
