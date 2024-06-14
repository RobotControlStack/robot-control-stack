#include <mujoco/mujoco.h>
#include <cstdlib>
#include "sim.h"
#include "config.h"

std::pair<mjModel*, mjData*> create_model_and_data() {
  mjModel* m;
  mjData* d;
  mj_loadXML(MODEL_DIR "/mjcf/scene.xml", NULL, NULL, 0);
  return std::pair(m, d);
}

int test_sim_class() {
  return EXIT_SUCCESS;
}

int main() {
  return test_sim_class();
}
