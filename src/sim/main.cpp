#include <Eigen/Eigen>
#include <chrono>
#include <iostream>

#include "mujoco/mjmodel.h"
#include "rl/math/Matrix.h"
#include "sim.h"

static const size_t NUM_THREADS = 30;
static const Eigen::Matrix<double, 1, 3, Eigen::RowMajor> iso_cube_center(
    0.498, 0.0, 0.226);

int main(void) {
  std::cout << "Starting simulation." << std::endl;
  std::shared_ptr<mjModel> m = std::shared_ptr<mjModel>(
      mj_loadXML(MODEL_DIR "/mjcf/scene.xml", NULL, NULL, 0));
  Simulation sim = Simulation(m, true, NUM_THREADS);
  sim.reset();
  Eigen::Matrix<rl::math::Real, NUM_THREADS, 6, Eigen::RowMajor> action(
      NUM_THREADS, 6);
  action(Eigen::all, Eigen::seqN(0, 3)) =
      Eigen::Replicate<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>,
                       NUM_THREADS, 1>(iso_cube_center);
  action(Eigen::all, Eigen::seqN(Eigen::fix<3>, Eigen::fix<3>)) =
      Eigen::Matrix<double, NUM_THREADS, 3>::Zero();  // set rotation to zero
  // run every thread for 2 minutes. With 30 threads this is one hour of
  // experience.
  std::chrono::steady_clock::time_point begin =
      std::chrono::steady_clock::now();
  for (size_t i = 0; i < 120; ++i) {
    sim.step(action);
  }
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  auto elapsed =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - begin)
          .count();
  std::cout << "Simulated 1 hour in " << elapsed << " [ms]" << std::endl;
  std::cout << "Speed up factor: " << 3.6e6 / elapsed << std::endl;
  sim.close();
  return EXIT_SUCCESS;
}
