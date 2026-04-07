#include "pcl/pcl_container.h"
#include "pcl/pcl_executor.h"
#include "pcl/pcl_log.h"

#include <stdio.h>
#include <string.h>

#ifdef _WIN32
#  define WIN32_LEAN_AND_MEAN
#  include <windows.h>
#else
#  include <pthread.h>
#  include <unistd.h>
#endif

typedef struct {
  pcl_executor_t* exec;
  int             updates_received;
  int             last_value;
} SensorData;

static void sensor_update_callback(pcl_container_t* c,
                                   const pcl_msg_t* msg,
                                   void*            user_data) {
  (void)c;
  SensorData* data = (SensorData*)user_data;
  data->updates_received++;
  data->last_value = *(const int*)msg->data;
}

static pcl_status_t sensor_on_configure(pcl_container_t* c, void* user_data) {
  SensorData* data = (SensorData*)user_data;
  if (!pcl_container_add_subscriber(c, "sensor_updates", "SensorMsg",
                                    sensor_update_callback, data)) {
    return PCL_ERR_CALLBACK;
  }
  return PCL_OK;
}

static pcl_status_t sensor_on_tick(pcl_container_t* c, double dt, void* user_data) {
  SensorData* data = (SensorData*)user_data;

  if (data->updates_received > 0) {
    pcl_log(c, PCL_LOG_INFO,
            "tick observed %d update(s), last_value=%d, dt=%.3fs",
            data->updates_received, data->last_value, dt);
  }

  if (data->updates_received >= 3) {
    pcl_log(c, PCL_LOG_INFO, "received all sensor updates, stopping executor");
    pcl_executor_request_shutdown(data->exec);
  }

  return PCL_OK;
}

#ifdef _WIN32
static DWORD WINAPI sensor_thread_main(LPVOID arg) {
#else
static void* sensor_thread_main(void* arg) {
#endif
  SensorData* data = (SensorData*)arg;
  int         i;

  for (i = 1; i <= 3; ++i) {
    int payload = i * 10;
    pcl_msg_t msg;

    memset(&msg, 0, sizeof(msg));
    msg.data = &payload;
    msg.size = sizeof(payload);
    msg.type_name = "SensorMsg";

    if (pcl_executor_post_incoming(data->exec, "sensor_updates", &msg) != PCL_OK) {
      return 0;
    }

    // Safe to reuse the stack buffer immediately because post_incoming copies it.
    payload = -1;

#ifdef _WIN32
    Sleep(20);
#else
    usleep(20000);
#endif
  }

#ifdef _WIN32
  return 0;
#else
  return NULL;
#endif
}

int main(void) {
  pcl_callbacks_t callbacks;
  pcl_container_t* sensor_container;
  pcl_executor_t*  exec;
  SensorData       data;

  memset(&callbacks, 0, sizeof(callbacks));
  memset(&data, 0, sizeof(data));

  callbacks.on_configure = sensor_on_configure;
  callbacks.on_tick = sensor_on_tick;

  sensor_container = pcl_container_create("sensor_consumer", &callbacks, &data);
  exec = pcl_executor_create();
  data.exec = exec;

  if (!sensor_container || !exec) {
    fprintf(stderr, "failed to create PCL objects\n");
    return 1;
  }

  pcl_container_set_tick_rate_hz(sensor_container, 50.0);
  pcl_container_configure(sensor_container);
  pcl_container_activate(sensor_container);
  pcl_executor_add(exec, sensor_container);

  printf("Starting external I/O bridge example...\n");
  printf("A background thread will inject messages into the executor queue.\n");

#ifdef _WIN32
  HANDLE producer = CreateThread(NULL, 0, sensor_thread_main, &data, 0, NULL);
  if (producer == NULL) {
    fprintf(stderr, "failed to create producer thread\n");
    return 1;
  }
#else
  pthread_t producer;
  if (pthread_create(&producer, NULL, sensor_thread_main, &data) != 0) {
    fprintf(stderr, "failed to create producer thread\n");
    return 1;
  }
#endif

  pcl_executor_spin(exec);

#ifdef _WIN32
  WaitForSingleObject(producer, INFINITE);
  CloseHandle(producer);
#else
  pthread_join(producer, NULL);
#endif

  pcl_executor_shutdown_graceful(exec, 5000);
  pcl_executor_destroy(exec);
  pcl_container_destroy(sensor_container);

  printf("Done.\n");
  return 0;
}
