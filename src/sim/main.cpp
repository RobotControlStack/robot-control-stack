#include <Eigen/Eigen>
#include <chrono>
#include <iostream>
#include <memory>
#include <random>

#include "mujoco/mjmodel.h"
#include "mujoco/mujoco.h"
#include "rl/math/Matrix.h"
#include "rl/math/Rotation.h"
#include "rl/math/Transform.h"
#include "rl/math/Vector.h"
#include "rl/mdl/Dynamic.h"
#include "rl/mdl/Exception.h"
#include "rl/mdl/JacobianInverseKinematics.h"
#include "rl/mdl/UrdfFactory.h"
#include "sim.h"

static const size_t NUM_THREADS = 30;
static const Eigen::Matrix<double, 1, 3, Eigen::RowMajor> iso_cube_center(
    0.498, 0.0, 0.226);
static const float iso_cube_size = 0.4;

float measure_speed() {
  std::shared_ptr<mjModel> m = std::shared_ptr<mjModel>(
      mj_loadXML(MODEL_DIR "/mjcf/scene.xml", NULL, NULL, 0));
  std::vector<std::shared_ptr<rl::mdl::Model>> fr3_rlmdls;
  fr3_rlmdls.reserve(NUM_THREADS);
  rl::mdl::UrdfFactory factory{};
  for (size_t i = 0; i < NUM_THREADS; ++i) {
    fr3_rlmdls.emplace_back(
        factory.create(MODEL_DIR "/urdf/fr3_from_panda.urdf"));
  }
  Simulation sim = Simulation(m, fr3_rlmdls, true, NUM_THREADS);
  sim.reset();
  std::vector<rl::math::Transform> action(NUM_THREADS);
  // run every thread for 2 minutes. With 30 threads this is one hour of
  // experience.
  std::chrono::steady_clock::time_point begin =
      std::chrono::steady_clock::now();
  for (size_t i = 0; i < 120; ++i) sim.step(action);
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  return 3.6e6 / (end - begin).count();
}

bool test_ik() {
  rl::mdl::UrdfFactory factory{};
  rl::mdl::Dynamic model;
  factory.load(MODEL_DIR "/urdf/fr3_from_panda.urdf", &model);
  std::unique_ptr<rl::mdl::JacobianInverseKinematics> ik;
  ik = std::make_unique<rl::mdl::JacobianInverseKinematics>(
      static_cast<rl::mdl::Kinematic *>(&model));
  rl::math::Transform t;
  t.translation() << 0.2, 0.2, 0.2;
  ik->addGoal(t, 0);
  bool success = ik->solve();
  if (success) {
    rl::math::Vector q(7);
    q = model.getPosition();
    std::cout << q << std::endl;
  } else {
    throw rl::mdl::Exception("IK failed");
  }
  return EXIT_SUCCESS;
}

std::array<double, 6> random_point_in_iso_cube() {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::array<double, 6> ret;
  std::uniform_real_distribution<double> distr_x(
      iso_cube_center[0] - iso_cube_size / 2,
      iso_cube_center[0] + iso_cube_size / 2);
  std::uniform_real_distribution<double> distr_y(
      iso_cube_center[1] - iso_cube_size / 2,
      iso_cube_center[1] + iso_cube_size / 2);
  std::uniform_real_distribution<double> distr_z(
      iso_cube_center[2] - iso_cube_size / 2,
      iso_cube_center[2] + iso_cube_size / 2);
  std::uniform_real_distribution<double> distr_angle(-std::numbers::pi,
                                                     std::numbers::pi);
  ret[0] = distr_x(gen);
  ret[1] = distr_y(gen);
  ret[2] = distr_z(gen);
  ret[3] = distr_angle(gen);
  ret[4] = distr_angle(gen);
  ret[5] = distr_angle(gen);
  return ret;
}
void test_convergence() {
  rl::mdl::UrdfFactory factory{};
  std::vector<std::shared_ptr<rl::mdl::Model>> fr3_rlmdls(0);
  fr3_rlmdls.reserve(NUM_THREADS);
  for (size_t i = 0; i < NUM_THREADS; ++i) {
    fr3_rlmdls.emplace_back(
        factory.create(MODEL_DIR "/urdf/fr3_from_panda.urdf"));
  }
  std::shared_ptr<mjModel> fr3_mjmdl = std::shared_ptr<mjModel>(
      mj_loadXML(MODEL_DIR "/mjcf/scene.xml", NULL, NULL, 0));
  Simulation sim = Simulation(fr3_mjmdl, fr3_rlmdls, true, NUM_THREADS);
  sim.reset();
  std::vector<rl::math::Transform> action(NUM_THREADS);
  for (size_t i = 0; i < 100; ++i) {
    for (size_t j = 0; j < NUM_THREADS; ++j) {
      std::array<double, 6> random_action = random_point_in_iso_cube();
      action[j].setIdentity();
      action[j].translation() << random_action[0], random_action[1],
          random_action[2];
      action[j].rotate(
          rl::math::AngleAxis(random_action[3], rl::math::Vector3::UnitX()));
      action[j].rotate(
          rl::math::AngleAxis(random_action[4], rl::math::Vector3::UnitY()));
      action[j].rotate(
          rl::math::AngleAxis(random_action[5], rl::math::Vector3::UnitZ()));
    }
    sim.step(action);
  }
  sim.close();
}

int main(void) {
  test_convergence();
  return EXIT_SUCCESS;
}
