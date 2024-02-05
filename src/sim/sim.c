#include "sim.h"
#include "ik/ik.h"
#include <mujoco/mujoco.h>
#include <stdio.h>
#include <stdlib.h>
#include <threads.h>

#define MAX_GEOMS 10000

struct physics_args {
  mjModel* model;
  mjData* data;
};

static int physics_loop(void* thrd_data) {
  struct physics_args args = *(struct physics_args*) thrd_data;
  free(thrd_data);
  while (true)
    mj_step(args.model, args.data);
  return EXIT_SUCCESS;
}

Sim* start_sim(SimConf args) {
  Sim* sim = malloc(sizeof(Sim));
  sim->n_threads = args.n_threads;
  sim->model = mj_loadXML(args.mjcf_filepath, NULL, sim->error_message, MAX_GEOMS);
  if (!sim->model) {
    sim->error = SIM_EMJMDL;
    return sim;
  }
  IK ik_model = ik_load(args.urdf_filepath);
  if (!ik_model) {
    sim->error = SIM_ERLMDL;
    return sim;
  }
  for (size_t i = 0; i < args.n_threads; ++i) {
    sim->data[i] = mj_makeData(sim->model);
    mj_resetDataKeyframe(sim->model, sim->data[i], SIM_KF_HOME);
    struct physics_args* phys_args = malloc(sizeof(struct physics_args));
    phys_args->model = sim->model;
    phys_args->data = sim->data[i];
    thrd_create(&sim->physics_threads[i], physics_loop, phys_args);
  }
  struct render_args* render_args = malloc(sizeof(struct render_args));
  render_args->data = sim->data;
  render_args->model = sim->model;
  render_args->n_threads = sim->n_threads;
  thrd_create(&sim->render_thread, render_thread, render_args);
  return sim;
}
