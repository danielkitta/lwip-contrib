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

#include <lwip/opt.h>
#include <arch/sys_arch.h>

#include <lwip/debug.h>
#include <lwip/stats.h>
#include <lwip/sys.h>

/* Sanity-check FreeRTOS configuration.
 */
#if !configSUPPORT_DYNAMIC_ALLOCATION
# error "lwIP FreeRTOS port requires configSUPPORT_DYNAMIC_ALLOCATION"
#endif
#if !configUSE_MUTEXES
# error "lwIP FreeRTOS port requires configUSE_MUTEXES"
#endif
#if !INCLUDE_vTaskDelay
# error "lwIP FreeRTOS port requires INCLUDE_vTaskDelay"
#endif
#if !INCLUDE_vTaskSuspend
# error "lwIP FreeRTOS port requires INCLUDE_vTaskSuspend"
#endif

/*-- Init -------------------------------------------------------------------*/

void sys_init(void)
{
}

/*-- Time -------------------------------------------------------------------*/

u32_t sys_now(void)
{
  return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

/*-- Threads ----------------------------------------------------------------*/

sys_thread_t sys_thread_new(const char *name, lwip_thread_fn function,
                            void *arg, int stacksize, int prio)
{
  BaseType_t ret;
  TaskHandle_t task = NULL;

  ret = xTaskCreate(function, name, stacksize, arg, prio, &task);

  /* Assert, in accordance with the lwIP documentation. */
  LWIP_ASSERT("task created", ret == pdPASS);
  LWIP_UNUSED_ARG(ret);

  return task;
}

/*-- Mailbox ----------------------------------------------------------------*/

err_t sys_mbox_new(sys_mbox_t *mbox, int size)
{
  QueueHandle_t queue;

  queue = xQueueCreate(size, sizeof(void *));
  *mbox = queue;

  if (!queue)
    return ERR_MEM;

  SYS_STATS_INC_USED(mbox);
  return ERR_OK;
}

void sys_mbox_free(sys_mbox_t *mbox)
{
  QueueHandle_t queue;

  queue = *mbox;
  if (!queue)
    return;
#if SYS_DEBUG
  {
    unsigned int n_waiting;
    n_waiting = uxQueueMessagesWaiting(queue);
    if (n_waiting > 0) {
      LWIP_DEBUGF(SYS_DEBUG | LWIP_DBG_TRACE | LWIP_DBG_LEVEL_SERIOUS,
                  ("sys_mbox_free: %u messages still waiting\n", n_waiting));
    }
  }
#endif
  vQueueDelete(queue);
  SYS_STATS_DEC(mbox.used);
}

void sys_mbox_post(sys_mbox_t *mbox, void *msg)
{
  QueueHandle_t queue;

  queue = *mbox;
  LWIP_ASSERT("mbox valid", queue);

  if (xQueueSendToBack(queue, &msg, portMAX_DELAY) != pdTRUE) {
    SYS_STATS_INC(mbox.err);
    LWIP_DEBUGF(SYS_DEBUG | LWIP_DBG_TRACE | LWIP_DBG_LEVEL_SERIOUS,
                ("sys_mbox_post: failed to queue message\n"));
  }
}

err_t sys_mbox_trypost(sys_mbox_t *mbox, void *msg)
{
  QueueHandle_t queue;

  queue = *mbox;
  LWIP_ASSERT("mbox valid", queue);

  if (xQueueSendToBack(queue, &msg, 0) != pdTRUE) {
    SYS_STATS_INC(mbox.err);
    return ERR_MEM;
  }
  return ERR_OK;
}

u32_t sys_arch_mbox_fetch(sys_mbox_t *mbox, void **msg, u32_t timeout)
{
  QueueHandle_t queue;
  TickType_t ticks, start, stop;
  BaseType_t ret;
  void *buf = NULL;

  queue = *mbox;
  LWIP_ASSERT("mbox valid", queue);

  ticks = (timeout > 0) ? timeout / portTICK_PERIOD_MS : portMAX_DELAY;
  start = xTaskGetTickCount();

  ret = xQueueReceive(queue, &buf, ticks);

  if (msg)
    *msg = buf;
  if (ret != pdTRUE)
    return SYS_ARCH_TIMEOUT;

  stop = xTaskGetTickCount();
  return (stop - start) * portTICK_PERIOD_MS;
}

u32_t sys_arch_mbox_tryfetch(sys_mbox_t *mbox, void **msg)
{
  QueueHandle_t queue;
  BaseType_t ret;
  void *buf = NULL;

  queue = *mbox;
  LWIP_ASSERT("mbox valid", queue);

  ret = xQueueReceive(queue, &buf, 0);

  if (msg)
    *msg = buf;

  return (ret == pdTRUE) ? 0 : SYS_MBOX_EMPTY;
}

/*-- Semaphore --------------------------------------------------------------*/

err_t sys_sem_new(sys_sem_t *sem, u8_t count)
{
  SemaphoreHandle_t handle;

  /* The initial count is de facto always zero with current lwIP. */
  LWIP_ASSERT("count <= 1", count <= 1);

  handle = xSemaphoreCreateBinary();
  *sem = handle;

  if (!handle)
    return ERR_MEM;
  if (count != 0)
    xSemaphoreGive(handle);

  SYS_STATS_INC_USED(sem);
  return ERR_OK;
}

void sys_sem_free(sys_sem_t *sem)
{
  SemaphoreHandle_t handle;

  handle = *sem;
  if (!handle)
    return;

  vSemaphoreDelete(handle);
  SYS_STATS_DEC(sem.used);
}

void sys_sem_signal(sys_sem_t *sem)
{
  SemaphoreHandle_t handle;

  handle = *sem;
  LWIP_ASSERT("semaphore valid", handle);

  xSemaphoreGive(handle);
}

u32_t sys_arch_sem_wait(sys_sem_t *sem, u32_t timeout)
{
  SemaphoreHandle_t handle;
  TickType_t ticks, start, stop;

  handle = *sem;
  LWIP_ASSERT("semaphore valid", handle);

  ticks = (timeout > 0) ? timeout / portTICK_PERIOD_MS : portMAX_DELAY;
  start = xTaskGetTickCount();

  if (xSemaphoreTake(handle, ticks) != pdTRUE)
    return SYS_ARCH_TIMEOUT;

  stop = xTaskGetTickCount();
  return (stop - start) * portTICK_PERIOD_MS;
}

/*-- Mutex ------------------------------------------------------------------*/

err_t sys_mutex_new(sys_mutex_t *mutex)
{
  SemaphoreHandle_t handle;

  handle = xSemaphoreCreateMutex();
  *mutex = handle;

  if (!handle)
    return ERR_MEM;

  SYS_STATS_INC_USED(mutex);
  return ERR_OK;
}

void sys_mutex_lock(sys_mutex_t *mutex)
{
  SemaphoreHandle_t handle;

  handle = *mutex;
  LWIP_ASSERT("mutex valid", handle);

  xSemaphoreTake(handle, portMAX_DELAY);
}

void sys_mutex_unlock(sys_mutex_t *mutex)
{
  SemaphoreHandle_t handle;

  handle = *mutex;
  LWIP_ASSERT("mutex valid", handle);

  xSemaphoreGive(handle);
}

void sys_mutex_free(sys_mutex_t *mutex)
{
  SemaphoreHandle_t handle;

  handle = *mutex;
  if (!handle)
    return;

  vSemaphoreDelete(handle);
  SYS_STATS_DEC(mutex.used);
}
