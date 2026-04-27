#if !defined(_WIN32) && !defined(_POSIX_C_SOURCE)
#  define _POSIX_C_SOURCE 200809L
#endif

#include "apos.h"

#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#  define WIN32_LEAN_AND_MEAN
#  include <windows.h>
#else
#  include <errno.h>
#  include <pthread.h>
#  include <time.h>
#endif

typedef struct apos_message_t {
  unsigned char*         data;
  int                    size;
  struct apos_message_t* next;
} apos_message_t;

typedef struct apos_channel_t {
  int                    lvc;
  apos_message_t*        head;
  apos_message_t*        tail;
  struct apos_channel_t* next;
} apos_channel_t;

static apos_channel_t* g_channels = 0;
static DelayCallback   g_delay_callback = 0;

#ifdef _WIN32
static CRITICAL_SECTION g_lock;
static CONDITION_VARIABLE g_cv;
static INIT_ONCE g_init_once = INIT_ONCE_STATIC_INIT;
__declspec(thread) static unsigned int g_process_id = 0;

static BOOL CALLBACK apos_init_once(PINIT_ONCE once,
                                    PVOID      parameter,
                                    PVOID*     context) {
  (void)once;
  (void)parameter;
  (void)context;
  InitializeCriticalSection(&g_lock);
  InitializeConditionVariable(&g_cv);
  return TRUE;
}

static void apos_init(void) {
  InitOnceExecuteOnce(&g_init_once, apos_init_once, 0, 0);
}
#else
static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  g_cv = PTHREAD_COND_INITIALIZER;
static pthread_once_t  g_init_once = PTHREAD_ONCE_INIT;
static _Thread_local unsigned int g_process_id = 0;

static void apos_init_once(void) {
}

static void apos_init(void) {
  pthread_once(&g_init_once, apos_init_once);
}
#endif

static unsigned int apos_timeout_to_ms(const CTimeout* timeout) {
  unsigned long long ms;
  if (!timeout) return 0xFFFFFFFFu;
  ms = (unsigned long long)timeout->uiSeconds * 1000ull;
  ms += ((unsigned long long)timeout->uiNanoSeconds + 999999ull) / 1000000ull;
  return ms > 0xFFFFFFFEull ? 0xFFFFFFFEu : (unsigned int)ms;
}

static apos_channel_t* apos_find_channel_locked(int lvc, int create) {
  apos_channel_t* ch;
  for (ch = g_channels; ch; ch = ch->next) {
    if (ch->lvc == lvc) return ch;
  }
  if (!create) return 0;
  ch = (apos_channel_t*)calloc(1u, sizeof(*ch));
  if (!ch) return 0;
  ch->lvc = lvc;
  ch->next = g_channels;
  g_channels = ch;
  return ch;
}

static int apos_channel_has_data_locked(int lvc) {
  apos_channel_t* ch = apos_find_channel_locked(lvc, 0);
  return ch && ch->head;
}

static int apos_any_channel_has_data_locked(const int* channels, int count) {
  int i;
  if (!channels || count <= 0) return 0;
  for (i = 0; i < count; ++i) {
    if (apos_channel_has_data_locked(channels[i])) return 1;
  }
  return 0;
}

static void apos_fill_ready_locked(const int* channels, int count, int* out) {
  int i;
  int out_index = 0;
  if (!out || count <= 0) return;
  for (i = 0; i < count; ++i) out[i] = -1;
  if (!channels) return;
  for (i = 0; i < count; ++i) {
    if (apos_channel_has_data_locked(channels[i])) {
      out[out_index++] = channels[i];
    }
  }
}

static void apos_lock(void) {
#ifdef _WIN32
  EnterCriticalSection(&g_lock);
#else
  pthread_mutex_lock(&g_lock);
#endif
}

static void apos_unlock(void) {
#ifdef _WIN32
  LeaveCriticalSection(&g_lock);
#else
  pthread_mutex_unlock(&g_lock);
#endif
}

static void apos_notify_all(void) {
#ifdef _WIN32
  WakeAllConditionVariable(&g_cv);
#else
  pthread_cond_broadcast(&g_cv);
#endif
}

static int apos_wait_locked(const int* channels, int count, unsigned int ms) {
#ifdef _WIN32
  if (ms == 0u) return 0;
  if (ms == 0xFFFFFFFFu) {
    while (!apos_any_channel_has_data_locked(channels, count)) {
      SleepConditionVariableCS(&g_cv, &g_lock, INFINITE);
    }
    return 1;
  }
  return SleepConditionVariableCS(&g_cv, &g_lock, ms) != 0;
#else
  struct timespec ts;
  int rc;
  if (ms == 0u) return 0;
  if (ms == 0xFFFFFFFFu) {
    while (!apos_any_channel_has_data_locked(channels, count)) {
      pthread_cond_wait(&g_cv, &g_lock);
    }
    return 1;
  }
  clock_gettime(CLOCK_REALTIME, &ts);
  ts.tv_sec += (time_t)(ms / 1000u);
  ts.tv_nsec += (long)((ms % 1000u) * 1000000u);
  if (ts.tv_nsec >= 1000000000L) {
    ts.tv_sec += 1;
    ts.tv_nsec -= 1000000000L;
  }
  rc = pthread_cond_timedwait(&g_cv, &g_lock, &ts);
  return rc == 0;
#endif
}

void setupAPOS(unsigned int uiProcessNo) {
  apos_init();
  g_process_id = uiProcessNo;
}

void setupApos(unsigned int uiProcessNo) {
  setupAPOS(uiProcessNo);
}

void setDelayFunction(DelayCallback pDelay) {
  apos_init();
  apos_lock();
  g_delay_callback = pDelay;
  apos_unlock();
}

void sendMessage(int            iLVC,
                 unsigned char* pucData,
                 int            iDataSize,
                 CTimeout*      pTimeoutLength,
                 TM_Status*     pStatus) {
  RS_Status status = RS_ERROR;
  (void)pTimeoutLength;
  sendMessageNonBlocking(iLVC, pucData, iDataSize, &status);
  if (pStatus) *pStatus = (status == RS_SUCCESS) ? TM_SUCCESS : TM_ERROR;
}

void sendMessageNonBlocking(int            iLVC,
                            unsigned char* pucData,
                            int            iDataSize,
                            RS_Status*     pStatus) {
  apos_channel_t* ch;
  apos_message_t* msg;
  (void)g_process_id;
  apos_init();

  if (iDataSize < 0 || (iDataSize > 0 && !pucData)) {
    if (pStatus) *pStatus = RS_ERROR;
    return;
  }

  msg = (apos_message_t*)calloc(1u, sizeof(*msg));
  if (!msg) {
    if (pStatus) *pStatus = RS_ERROR;
    return;
  }
  if (iDataSize > 0) {
    msg->data = (unsigned char*)malloc((size_t)iDataSize);
    if (!msg->data) {
      free(msg);
      if (pStatus) *pStatus = RS_ERROR;
      return;
    }
    memcpy(msg->data, pucData, (size_t)iDataSize);
  }
  msg->size = iDataSize;

  apos_lock();
  ch = apos_find_channel_locked(iLVC, 1);
  if (!ch) {
    apos_unlock();
    free(msg->data);
    free(msg);
    if (pStatus) *pStatus = RS_ERROR;
    return;
  }
  if (ch->tail) {
    ch->tail->next = msg;
  } else {
    ch->head = msg;
  }
  ch->tail = msg;
  apos_notify_all();
  apos_unlock();

  if (g_delay_callback) g_delay_callback(0);
  if (pStatus) *pStatus = RS_SUCCESS;
}

void receiveMessage(int            iLVC,
                    unsigned char* pucData,
                    int            iMaxSize,
                    int*           piDataSize,
                    CTimeout*      pTimeoutLength,
                    TM_Status*     pStatus) {
  int channels[1];
  int ready[1];
  unsigned int timeout_ms;

  if (pStatus) *pStatus = TM_ERROR;
  if (piDataSize) *piDataSize = 0;
  if (iMaxSize < 0 || (iMaxSize > 0 && !pucData)) return;

  channels[0] = iLVC;
  ready[0] = -1;
  timeout_ms = apos_timeout_to_ms(pTimeoutLength);

  apos_init();
  apos_lock();
  while (!apos_any_channel_has_data_locked(channels, 1)) {
    if (!apos_wait_locked(channels, 1, timeout_ms)) {
      apos_unlock();
      if (pStatus) *pStatus = TM_TIMEOUT;
      return;
    }
  }
  apos_fill_ready_locked(channels, 1, ready);
  apos_unlock();

  (void)ready;
  {
    RS_Status rs = RS_ERROR;
    receiveMessageNonBlocking(iLVC, pucData, iMaxSize, piDataSize, &rs);
    if (pStatus) {
      *pStatus = (rs == RS_SUCCESS) ? TM_SUCCESS :
                 (rs == RS_RESOURCE ? TM_TIMEOUT : TM_ERROR);
    }
  }
}

void receiveMessageNonBlocking(int            iLVC,
                               unsigned char* pucData,
                               int            iMaxSize,
                               int*           piDataSize,
                               RS_Status*     pStatus) {
  apos_channel_t* ch;
  apos_message_t* msg;

  apos_init();
  if (pStatus) *pStatus = RS_ERROR;
  if (piDataSize) *piDataSize = 0;
  if (iMaxSize < 0 || (iMaxSize > 0 && !pucData)) return;

  apos_lock();
  ch = apos_find_channel_locked(iLVC, 0);
  if (!ch || !ch->head) {
    apos_unlock();
    if (pStatus) *pStatus = RS_RESOURCE;
    return;
  }

  msg = ch->head;
  if (piDataSize) *piDataSize = msg->size;
  if (msg->size > iMaxSize) {
    apos_unlock();
    if (pStatus) *pStatus = RS_ERROR;
    return;
  }

  ch->head = msg->next;
  if (!ch->head) ch->tail = 0;
  apos_unlock();

  if (msg->size > 0) memcpy(pucData, msg->data, (size_t)msg->size);
  free(msg->data);
  free(msg);
  if (pStatus) *pStatus = RS_SUCCESS;
}

void waitOnMultiChannel(int*       piVCSetIn,
                        int        iMinVcNo,
                        int*       piVCSetOut,
                        CTimeout*  pTimeoutLength,
                        TM_Status* pStatus) {
  unsigned int timeout_ms;

  apos_init();
  if (pStatus) *pStatus = TM_ERROR;
  if (!piVCSetIn || iMinVcNo <= 0 || !piVCSetOut) return;

  timeout_ms = apos_timeout_to_ms(pTimeoutLength);
  apos_lock();
  while (!apos_any_channel_has_data_locked(piVCSetIn, iMinVcNo)) {
    if (!apos_wait_locked(piVCSetIn, iMinVcNo, timeout_ms)) {
      apos_fill_ready_locked(piVCSetIn, iMinVcNo, piVCSetOut);
      apos_unlock();
      if (pStatus) *pStatus = TM_TIMEOUT;
      return;
    }
  }
  apos_fill_ready_locked(piVCSetIn, iMinVcNo, piVCSetOut);
  apos_unlock();
  if (pStatus) *pStatus = TM_SUCCESS;
}

void logEvent(char* pcLogEntry) {
  (void)pcLogEntry;
}
