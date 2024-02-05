#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <threads.h>
#include <unistd.h>

#include "sim.h"
#include "plugin.h"

char USAGE[] = "Usage: %s [OPTION...] [FILE]\n";
char HELP[] =
"Franka Research 3 simulation for Reinforcement Learning.\n"
"\n"
"  -r           Render the simulation\n"
"  -n N_THREADS Start N_THREADS simulations in different threads\n"
"  -m PATH      Path to the mjcf model file\n"
"  -u PATH      Path to the corresponding URDF file\n"
"  -h           Print this help\n";

enum Options { OPT_N = 'n', OPT_R = 'r', OPT_H = 'h', OPT_M = 'm', OPT_U = 'u', OPT_ERR = '?'};
char OPTSTRING[] = "rhm:u:n:";

SimConf parse_args(int argc, char *argv[], SimConf defaults) {
  int opt;
  while ((opt = getopt(argc, argv, OPTSTRING)) != -1) {
    switch ((enum Options)opt) {
    case OPT_N:
      defaults.n_threads = atoi(optarg);
      break;
    case OPT_R:
      defaults.render = true;
      break;
    case OPT_M:
      defaults.mjcf_filepath = strndup(optarg, FILENAME_MAX);
      break;
    case OPT_U:
      defaults.urdf_filepath = strndup(optarg, FILENAME_MAX);
      break;
    case OPT_H:
      printf(USAGE, argv[0]);
      puts(HELP);
      exit(EXIT_SUCCESS);
    case OPT_ERR:
      exit(EXIT_FAILURE);
    }
  }
  return defaults;
}

bool validate_args(SimConf args) {
  bool ret = true;
  {
    FILE *model_file = fopen(args.mjcf_filepath, "r");
    if (!model_file) {
      fprintf(stderr, "Could not open file '%s': ", args.mjcf_filepath);
      perror("");
      ret = false;
    } else {
      fclose(model_file);
    }
  }
  {
    FILE *model_file = fopen(args.urdf_filepath, "r");
    if (!model_file) {
      fprintf(stderr, "Could not open file '%s': ", args.urdf_filepath);
      perror("");
      ret = false;
    } else {
      fclose(model_file);
    }
  }
  {
    if (args.n_threads > SIM_MAX_THREADS) {
      fprintf(stderr, "Cannot start more than %d threads\n", SIM_MAX_THREADS);
      ret = false;
    }
  }
  return ret;
}

int main(int argc, char *argv[]) {
  SimConf args = {
    .n_threads = 1,
    .mjcf_filepath = MODEL_DIR "/mjcf/scene.xml",
    .urdf_filepath = MODEL_DIR "/urdf/fr3.urdf",
  };
  args = parse_args(argc, argv, args);
  if (!validate_args(args))
    return EXIT_FAILURE;
  Sim* sim = start_sim(args);
  switch ((enum sim_errors) sim->error) {
    case SIM_EOK:
      break;
    case SIM_EMJMDL:
      fprintf(stderr, "Error loading MuJoCo model %s:\n%s", args.mjcf_filepath, sim->error_message);
      exit(EXIT_FAILURE);
    case SIM_ERLMDL:
      fprintf(stderr, "Error loading URDF model %s\n", args.urdf_filepath);
      exit(EXIT_FAILURE);
  }
  thrd_join(sim->render_thread, NULL);
  return EXIT_SUCCESS;
}
