#include <mujoco/mujoco.h>
#include <stdio.h>
#include <threads.h>
#include "mujoco/mjdata.h"
#include "sim.h"
#include "ik/ik.h"

typedef struct {
  Sim* sim;
  size_t n_obs;
  size_t n_act;
} Env;

typedef struct {
  size_t size;
  double* state;
} Obs;

typedef struct {
  double translation[3];
  double rotation[9];
} Action;

Env* init(SimConf cfg) {
  Env* env = malloc(sizeof(Env));
  if (!env) {
    perror("malloc");
    return NULL;
  }
  env->sim = start_sim(cfg);
  env->n_act = 12;
  env->n_obs = mj_stateSize(env->sim->model, mjSTATE_PHYSICS);
  return env;
}

double* reset(Env* env) {
  double* ret = malloc(sizeof(env->sim->n_threads) * env->n_obs);
  for (size_t i = 0; i < env->sim->n_threads; ++i) {
    mj_resetDataKeyframe(env->sim->model, env->sim->data[i], SIM_KF_HOME);
    mj_getState(env->sim->model, env->sim->data[i], ret + i * env->n_obs, mjSTATE_PHYSICS);
  }
  return ret;
}

void step(Env env, Action act, Obs* ret[]) {
  mjtNum quat[4];
  mju_mat2Quat(quat, act.rotation);
};
