#include <mujoco/mujoco.h>
#include <stdbool.h>
#include <stddef.h>
#include <threads.h>

#include "ui/ui.h"
#define SIM_MAX_ERR_MSG_SIZE 1000
#define SIM_MAX_THREADS 128

enum sim_errors { SIM_EOK = 0, SIM_EMJMDL, SIM_ERLMDL };
enum { SIM_KF_HOME = 0 };

typedef struct {
  char* mjcf_filepath;
  char* urdf_filepath;
  uint8_t n_threads;
  bool render;
} SimConf;

typedef struct {
  mjModel* model;
  mjData* data[SIM_MAX_THREADS];
  thrd_t physics_threads[SIM_MAX_THREADS];
  mtx_t physics_locks[SIM_MAX_THREADS];
  thrd_t render_thread;
  int error;
  char error_message[SIM_MAX_ERR_MSG_SIZE];
  uint8_t n_threads;
} Sim;

Sim* start_sim(SimConf);
