#include <GLFW/glfw3.h>
#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>

#include <cstdlib>
#include <iostream>
#include <memory>
#include <thread>

#include "common/Pose.h"
#include "sim.h"
#include "sim/FR3.h"

// const std::string mjcf = MODEL_DIR "/mjcf/fr3_modular/scene.xml";
// const std::string urdf = MODEL_DIR "/fr3/urdf/fr3_from_panda.urdf";
const std::string mjcf = "build/_deps/scenes-src/scenes/lab/scene.xml";
const std::string urdf = "build/_deps/scenes-src/scenes/lab/assets/fr3.urdf";
static const Eigen::Matrix<double, 1, 3, Eigen::RowMajor> iso_cube_center(
    0.498, 0.0, 0.226);
static const float iso_cube_size = 0.4;
static rcs::common::Pose target;
static rcs::common::Pose actual_cartesian_pose;

void rendering_loop(mjModel* m, mjData* d) {
  mjvCamera cam;   // abstract camera
  mjvOption opt;   // visualization options
  mjvScene scn;    // abstract scene
  mjrContext con;  // custom GPU context
  mjvPerturb pert;

  // init GLFW, create window, make OpenGL context current, request v-sync
  glfwInit();
  GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // initialize visualization data structures
  mjv_defaultCamera(&cam);
  mjv_defaultPerturb(&pert);
  mjv_defaultOption(&opt);
  mjr_defaultContext(&con);

  // create scene and context
  mjv_makeScene(m, &scn, 1000);
  mjr_makeContext(m, &con, mjFONTSCALE_100);

  while (!glfwWindowShouldClose(window)) {
    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // update scene and render
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjvGeom newgeom;
    Eigen::Vector3d pos = target.translation();
    float marker_col[4] = {1, 0, 0, 1};
    mjtNum size = 0.015;
    mjv_initGeom(&newgeom, mjGEOM_SPHERE, &size, pos.data(), NULL, marker_col);
    mjvGeom* thisgeom = scn.geoms + scn.ngeom;
    *thisgeom = newgeom;
    thisgeom->segid = scn.ngeom;
    scn.ngeom++;

    mjr_render(viewport, &scn, &con);

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
  }

  // close GLFW, free visualization storage
  glfwTerminate();
  mjv_freeScene(&scn);
  mjr_freeContext(&con);
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
auto create_mj_datastructures() {
  char error[1000];
  mjModel* m = mj_loadXML(mjcf.c_str(), NULL, error, 1000);
  if (not m) {
    std::cout << error << std::endl;
    exit(EXIT_FAILURE);
  }
  mjData* d = mj_makeData(m);
  if (not d) {
    std::cout << "Failed to create mjData datastructure" << std::endl;
  }
  return std::pair(m, d);
}

int test_sim() {
  auto [m, d] = create_mj_datastructures();
  auto sim = std::make_shared<rcs::sim::Sim>(m, d);
  auto cfg = sim->get_config();
  cfg.realtime = true;
  cfg.async = false;
  sim->set_config(cfg);
  std::string id = "0";

  auto ik = std::make_shared<rcs::common::RL>(urdf);
  auto fr3 = rcs::sim::FR3(sim, id, ik);
  auto tcp_offset = rcs::common::Pose(rcs::common::FrankaHandTCPOffset());
  rcs::sim::FR3Config fr3_config = *fr3.get_parameters();
  fr3_config.tcp_offset = tcp_offset;
  fr3_config.seconds_between_callbacks = 0.05;  // 20hz
  fr3.set_parameters(fr3_config);
  std::jthread t(rendering_loop, m, d);
  sim->step(1);
  for (size_t i = 0; i < 100; ++i) {
    auto desired_pose = random_pose_in_iso_cube();
    target = desired_pose;
    fr3.set_cartesian_position(desired_pose);
    sim->step_until_convergence();
    auto state = fr3.get_state();
    if (state->ik_success) {
      if (state->is_moving) {
        throw std::runtime_error(
            "FR3 should not be moving at the end of a step");
      }
      if (not state->is_arrived) {
        throw std::runtime_error("FR3 should be arrived at the end of a step");
      }
      /* According to fact sheet, pose repeatability within iso cube is 0.1
       * millimeters, i.e. 0.0001 meters.
       * We don't quite get there (IK / floating point accuracy?)
       * I Just tested the IK, here is a sample computed / desired position
       * IK solution
       *    -0.507646     0.861566   4.7215e-11     0.500292
       *     0.861566     0.507646 -3.47652e-11   -0.0312635
       *  -5.3921e-11  2.30303e-11           -1     0.461741
       * Desired position
       *    -0.507646     0.861566  1.22465e-16     0.500292
       *     0.861566     0.507646            0   -0.0312635
       * -6.21687e-17  1.05511e-16           -1     0.461741
       * So the problem seems to come from the mujoco part. Could be either
       * the PID controllers, or the calculated position of the end effector.
       * Maybe also when we take the inverse of the tcp offset?
       * Nope:
       * TCP offset:
       *     1      0      0      0
       *     0      1      0      0
       *     0      0      1 0.1034
       *     0      0      0      1
       * roll: -0        pitch: 0        yaw: -0
       * Inverse tcp offset:
       *      1       0       0       0
       *      0       1       0       0
       *      0       0       1 -0.1034
       *      0       0       0       1
       * Next test: compare the actual angles with the computed angles
       *
       * Actual angles: -0.394268
       * -0.617676
       *  0.480793
       *   -2.4452
       *  0.282081
       *   1.86158
       *   2.34344
       * Difference:  -0.00112583
       *  -0.00125228
       * -0.000798704
       *    0.0063457
       *   0.00154612
       *   0.00233189
       *    0.0188265
       * That seems to be the problem... So the PID controllers?
       * Verification: bypass the PID controllers by setting the angles
       * directly. Yep, when setting the angles directly we can get the 1mm
       * precision and the rtol can be set to .01 degrees.
       *
       * Note: mujoco actually does not implement a PID controller.
       */
      auto current_pose = fr3.get_cartesian_position();
      long double rtol = 3 * (std::numbers::pi / 180.0);  // 3 degrees
      long double ttol =
          1.875 / 100.0;  // 1.875 cm found after short bisection search
      if (not desired_pose.is_close(current_pose, rtol, ttol)) {
        std::cout << desired_pose.str() << std::endl;
        std::cout << current_pose.str() << std::endl;
        std::cout
            << "Translation distance: "
            << (desired_pose.translation() - current_pose.translation()).norm()
            << std::endl;
        std::cout << "Translation tolerance: " << ttol << std::endl;
        std::cout << "Rotational distance: "
                  << desired_pose.quaternion().normalized().angularDistance(
                         current_pose.quaternion().normalized())
                  << std::endl;
        std::cout << "Rotational tolerance: " << rtol << std::endl;
        std::cout << "Computed angles: " << fr3.get_state()->target_angles
                  << std::endl;
        std::cout << "Actual angles: " << fr3.get_joint_position() << std::endl;
        std::cout << "Difference: "
                  << fr3.get_state()->target_angles - fr3.get_joint_position()
                  << std::endl;
        throw std::runtime_error(
            "FR3 should be close to the desired cartesian pose");
      }
    }
  }
  return EXIT_SUCCESS;
}

int main() { return test_sim(); }
