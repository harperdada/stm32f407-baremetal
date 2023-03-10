#include <string.h>
#include <zephyr.h>

#include "lib_rtos/lib_rtos.h"

extern struct k_mem_pool decoder_mem_pool;


#define DECODER_THREAD_STACK_SIZE (16384 * 2)

K_THREAD_STACK_ARRAY_DEFINE(decoder_thread_stacks, CONFIG_DEC_NUM_THREADS,
          DECODER_THREAD_STACK_SIZE);

typedef struct {
  k_tid_t tid;
  struct k_thread thread;
  int index;
} thread_info_t;

static thread_info_t thread_table[CONFIG_DEC_NUM_THREADS];

static thread_info_t *get_thread_entry(void)
{
  for (size_t i = 0; i < CONFIG_DEC_NUM_THREADS; i++) {
    if (thread_table[i].tid == NULL) {
      thread_table[i].index = i;
      return &thread_table[i];
    }
  }
  printk("%s: returning NULL\n", __FUNCTION__);
  return NULL;
}

void *Rtos_Malloc(size_t zSize)
{
        void *ptr = k_mem_pool_malloc(&decoder_mem_pool, zSize);
        if (ptr == NULL) {
                printk("Rtos_Malloc : Failed to allocate %ld bytes of memory.\n",
                       zSize);
        }
        return ptr;
}

/****************************************************************************/
void Rtos_Free(void *pMem)
{
        k_free(pMem);
}

/****************************************************************************/
void *Rtos_Memcpy(void *pDst, void const *pSrc, size_t zSize)
{
        return memcpy(pDst, pSrc, zSize);
}

/****************************************************************************/
void *Rtos_Memmove(void *pDst, void const *pSrc, size_t zSize)
{
        return memmove(pDst, pSrc, zSize);
}

/****************************************************************************/
void *Rtos_Memset(void *pDst, int iVal, size_t zSize)
{
        return memset(pDst, iVal, zSize);
}

/****************************************************************************/
int Rtos_Memcmp(void const *pBuf1, void const *pBuf2, size_t zSize)
{
        return memcmp(pBuf1, pBuf2, zSize);
}

void Rtos_Sleep(uint32_t uMillisecond)
{
        k_sleep(uMillisecond);
}

AL_MUTEX Rtos_CreateMutex()
{
        struct k_mutex *mutex;
        mutex = k_mem_pool_malloc(&decoder_mem_pool, sizeof(struct k_mutex));
        if (mutex) {
                k_mutex_init(mutex);
        } else {
                printk("Failed to allocate memory for mutex.\n");
        }
        return mutex;
}
/****************************************************************************/
void Rtos_DeleteMutex(AL_MUTEX Mutex)
{
        k_free(Mutex);
}

/****************************************************************************/
bool Rtos_GetMutex(AL_MUTEX Mutex)
{
        return (k_mutex_lock((struct k_mutex *)Mutex, K_FOREVER) == 0);
}

/****************************************************************************/
bool Rtos_ReleaseMutex(AL_MUTEX Mutex)
{
        return (k_mutex_unlock((struct k_mutex *)Mutex) == 0);
}

AL_SEMAPHORE Rtos_CreateSemaphore(int iInitialCount)
{
        struct k_sem *sem;
        sem = k_mem_pool_malloc(&decoder_mem_pool, sizeof(struct k_sem));
        if (sem) {
                k_sem_init(sem, iInitialCount, UINT32_MAX);
        } else {
                printk("Failed to allocate memory for mutex.\n");
        }
        return (AL_SEMAPHORE)sem;
}
/****************************************************************************/
void Rtos_DeleteSemaphore(AL_SEMAPHORE Semaphore)
{
        k_free(Semaphore);
}

/****************************************************************************/

static s32_t convert_time(uint32_t Wait)
{
        s32_t wait_time;

        if (Wait == AL_NO_WAIT) {
                wait_time = K_NO_WAIT;
        } else if (Wait == AL_WAIT_FOREVER) {
                wait_time = K_FOREVER;
        } else {
                wait_time = Wait;
        }

        return wait_time;
}
bool Rtos_GetSemaphore(AL_SEMAPHORE Semaphore, uint32_t Wait)
{
        int ret = 0;
        s32_t wait_time = convert_time(Wait);

        if (wait_time == K_NO_WAIT) {
                ret = k_sem_take((struct k_sem *)Semaphore, K_NO_WAIT);
        } else if (wait_time == K_FOREVER) {
                do {
                        ret = k_sem_take((struct k_sem *)Semaphore, K_FOREVER);
                } while ((ret == -EBUSY) || (ret == -EAGAIN));
        } else {
                s64_t start_time = k_uptime_get();
                s64_t elapsed_time = 0;
                do {
                        ret = k_sem_take((struct k_sem *)Semaphore, wait_time);
                        elapsed_time = (s32_t)k_uptime_delta(&start_time);
                        if (elapsed_time >= wait_time) {
                                break;
                        } else {
                                wait_time -= (s32_t)elapsed_time;
                        }
                } while (ret == -EBUSY);
        }

        return ret == 0 ? true : false;
}
/****************************************************************************/
bool Rtos_ReleaseSemaphore(AL_SEMAPHORE Semaphore)
{
        k_sem_give((struct k_sem *)Semaphore);
        return true;
}

/****************************************************************************/
AL_EVENT Rtos_CreateEvent(bool bInitialState)
{
        struct k_poll_signal *signal;

        signal = k_mem_pool_malloc(&decoder_mem_pool,
                                   sizeof(struct k_poll_signal));
        if (signal) {
                k_poll_signal_init(signal);
                if (bInitialState) {
                        k_poll_signal_raise(signal, 0);
                }
        } else {
                printk("Failed to allocate memory for signal.\n");
        }

        return (AL_EVENT)signal;
}
/****************************************************************************/
void Rtos_DeleteEvent(AL_EVENT Event)
{
        k_free(Event);
}

/****************************************************************************/
bool Rtos_WaitEvent(AL_EVENT Event, uint32_t Wait)
{
        int ret = 0;
        struct k_poll_event poll_event;
        s32_t wait_time = convert_time(Wait);

        k_poll_event_init(&poll_event, K_POLL_TYPE_SIGNAL,
                          K_POLL_MODE_NOTIFY_ONLY, Event);

        if (wait_time == K_NO_WAIT) {
                ret = k_poll(&poll_event, 1, K_NO_WAIT);
        } else if (wait_time == K_FOREVER) {
                do {
                        ret = k_poll(&poll_event, 1, K_FOREVER);
                } while ((ret == -EINTR) || (ret == -EAGAIN));
        } else {
                s64_t start_time = k_uptime_get();
                s64_t elapsed_time = 0;
                do {
                        ret = k_poll(&poll_event, 1, wait_time);
                        elapsed_time = (s32_t)k_uptime_delta(&start_time);
                        if (elapsed_time >= wait_time) {
                                break;
                        } else {
                                wait_time -= (s32_t)elapsed_time;
                        }
                } while (ret == -EINTR);
        }
        if (ret == 0) {
                k_poll_signal_reset((struct k_poll_signal *)Event);
                return true;
        }

        return false;
}

/****************************************************************************/
bool Rtos_SetEvent(AL_EVENT Event)
{
        k_poll_signal_raise((struct k_poll_signal *)Event, 0);
        return true;
}

/****************************************************************************/
static void entry_point_wrapper(void *param_1, void *param_2, void *param_3)
{
  void (*func_ptr)(void *pParam) = param_1;
  (*func_ptr)(param_2);
}


AL_THREAD Rtos_CreateThread(void *(*pFunc)(void *pParam), void *pParam)
{
  thread_info_t *thread_entry;
  printk("%s: Called\n", __FUNCTION__);
  // Find a free thread.
  thread_entry = get_thread_entry();
  printk("%s: Got thread_info pointer %x\n", __FUNCTION__, thread_entry);

  thread_entry->tid = k_thread_create(
    &thread_entry->thread,
    decoder_thread_stacks[thread_entry->index],
    K_THREAD_STACK_SIZEOF(
      decoder_thread_stacks[thread_entry->index]),
    // Entry point to the wrapper which will call the actual
    // entry point for this thread. We need to do this as
    // the signatures of the functions required by Zephyr and the
    // ones provided by Allegro are different.
    entry_point_wrapper,
    // Input args to the wrapper
    pFunc, pParam, NULL,
    // Priority, options and delay in starting the thread
    CONFIG_DEC_PRIORITY, 0, K_NO_WAIT);

  return &thread_entry->tid;
}
/****************************************************************************/
void Rtos_SetCurrentThreadName(const char *pThreadName)
{
  k_thread_name_set(k_current_get(), pThreadName);
}

/****************************************************************************/
bool Rtos_JoinThread(AL_THREAD Thread)
{
  thread_info_t *info =
    CONTAINER_OF((k_tid_t *)Thread, thread_info_t, tid);
  // k_thread_join doesn't exist in Zephyr 2.2.1
  k_thread_abort(*(k_tid_t *)Thread);
  printk("%s:  thread aborted\n", __FUNCTION__);
  printk("%s: Got thread_info pointer %x\n", __FUNCTION__, info);
  memset(&info->thread, 0, sizeof(struct k_thread));
  info->tid = NULL;
  return true;
}

/****************************************************************************/
void Rtos_DeleteThread(AL_THREAD Thread)
{
  // Memory for the threads is statically allocated, so delete
  // doesn't have too much relevance in this case.
}

/****************************************************************************/
int32_t Rtos_AtomicIncrement(int32_t *iVal)
{
  return __sync_add_and_fetch(iVal, 1);
}

/****************************************************************************/
int32_t Rtos_AtomicDecrement(int32_t *iVal)
{
  return __sync_sub_and_fetch(iVal, 1);
}

