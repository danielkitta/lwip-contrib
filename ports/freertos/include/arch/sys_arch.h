/*
 * Copyright (c) 2017 Daniel Elstner <daniel.kitta@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 */
/**
 * @file
 * @brief lwIP abstraction layer for FreeRTOS.
 */

#ifndef LWIP_ARCH_SYS_ARCH_H_INCLUDED
#define LWIP_ARCH_SYS_ARCH_H_INCLUDED

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include <errno.h>

typedef SemaphoreHandle_t sys_sem_t;
typedef SemaphoreHandle_t sys_mutex_t;
typedef QueueHandle_t     sys_mbox_t;
typedef TaskHandle_t      sys_thread_t;

#define SYS_MBOX_NULL (QueueHandle_t)NULL
#define SYS_SEM_NULL  (SemaphoreHandle_t)NULL

#define sys_mbox_valid(mbox)        (*(mbox) != NULL)
#define sys_mbox_set_invalid(mbox)  (*(mbox) = NULL)

#define sys_sem_valid(sem)          (*(sem) != NULL)
#define sys_sem_set_invalid(sem)    (*(sem) = NULL)

#define sys_mutex_valid(mutex)        (*(mutex) != NULL)
#define sys_mutex_set_invalid(mutex)  (*(mutex) = NULL)

#define sys_msleep(ms)  vTaskDelay(TASK_DELAY_MS(ms))

#define SYS_ARCH_DECL_PROTECT(lev)  (void)0
#define SYS_ARCH_PROTECT(lev)       taskENTER_CRITICAL()
#define SYS_ARCH_UNPROTECT(lev)     taskEXIT_CRITICAL()

#endif /* !LWIP_ARCH_SYS_ARCH_H_INCLUDED */
