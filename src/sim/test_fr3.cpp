#include <unistd.h>

#include <cstdlib>

#include "sim/FR3.h"
#include "sim/plugin.h"

int main() {
  auto robot = rcs::sim::FR3(MODEL_DIR "/mjcf/scene.xml",
                             MODEL_DIR "/urdf/fr3_from_panda.urdf");
  sleep(10);
  return EXIT_SUCCESS;
}
