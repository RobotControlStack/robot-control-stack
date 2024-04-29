#include <Eigen/Eigen>
#include <chrono>
#include <iostream>
#include <memory>
#include <random>
#include <string>

#include "common/Pose.h"
#include "mujoco/mjdata.h"
#include "mujoco/mujoco.h"
#include "plugin.h"
#include "rl/math/Transform.h"
#include "rl/math/Vector.h"
#include "rl/mdl/Dynamic.h"
#include "rl/mdl/Exception.h"
#include "rl/mdl/JacobianInverseKinematics.h"
#include "rl/mdl/UrdfFactory.h"
#include "sim.h"
#include "sim/FR3.h"

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
  std::vector<rcs::common::Pose> action(NUM_THREADS);
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
      static_cast<rl::mdl::Kinematic*>(&model));
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

rcs::common::RPY random_rpy() {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> distr_angle(-std::numbers::pi,
                                                     std::numbers::pi);
  return rcs::common::RPY(0, 180 * std::numbers::pi / 180, distr_angle(gen));
}

Eigen::Vector3d random_point_in_iso_cube() {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> distr_x(
      iso_cube_center[0] - iso_cube_size / 2,
      iso_cube_center[0] + iso_cube_size / 2);
  std::uniform_real_distribution<double> distr_y(
      iso_cube_center[1] - iso_cube_size / 2,
      iso_cube_center[1] + iso_cube_size / 2);
  std::uniform_real_distribution<double> distr_z(
      iso_cube_center[2] - iso_cube_size / 2,
      iso_cube_center[2] + iso_cube_size / 2);
  return Eigen::Vector3d(distr_x(gen), distr_y(gen), distr_z(gen));
}

rcs::common::Pose random_pose_in_iso_cube() {
  return rcs::common::Pose(random_rpy(), random_point_in_iso_cube());
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
  std::vector<rcs::common::Pose> action(NUM_THREADS);
  for (size_t i = 0; i < 100; ++i) {
    for (size_t j = 0; j < NUM_THREADS; ++j) {
      action[j] = random_pose_in_iso_cube();
    }
    sim.step(action);
  }
  sim.close();
}

void test_fr3() {
  const std::string mjcf = MODEL_DIR "/mjcf/scene.xml";
  const std::string urdf = MODEL_DIR "/urdf/fr3_from_panda.urdf";
  mjModel* m = mj_loadXML(mjcf.c_str(), NULL, NULL, 0);
  mjData* d = mj_makeData(m);
  rcs::sim::FR3 robot(m, d, rl::mdl::UrdfFactory().create(urdf));
  rcs::sim::FR3Config cfg{};
  cfg.ik_duration = 300;
  cfg.realtime = true;
  cfg.trajectory_trace = true;
  robot.set_parameters(cfg);

  double marker_size = 0.015;
  float marker_col[4] = {1, 0, 0, 1};

  for (size_t i = 0; i < 100; ++i) {
    std::cout << "Setting cartesian position... " << std::endl;
    rcs::common::Pose target = random_pose_in_iso_cube();
    robot.add_sphere(target, marker_size, marker_col);
    robot.set_cartesian_position(target);
    std::cout << "Done!" << std::endl;
    std::cout << "Moving home... ";
    robot.move_home();
    std::cout << "Done!" << std::endl;
    robot.clear_markers();
  }
  robot.move_home();
  std::cout << "Exiting..." << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(10000));
}

int main(void) {
  test_fr3();
  return EXIT_SUCCESS;
}
