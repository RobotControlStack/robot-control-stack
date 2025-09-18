#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/robot.h>
#include <rcs/IK.h>

#include <iostream>
#include <memory>
#include <optional>
#include <string>

#include "FR3.h"

using namespace std;

const string ip = "192.168.101.1";
const string urdf_path = "models/urdf/fr3.urdf";

int main() {
  try {
    auto ik = make_shared<rcs::common::RL>(urdf_path);
    rcs::hw::FR3 robot(ip, ik);
    robot.automatic_error_recovery();
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!"
              << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.move_home();

    auto rs = robot.get_cartesian_position();
    rs.translation() -= Eigen::Vector3d(0, 0, 0.1);

    robot.set_cartesian_position_internal(rs, 5.0, std::nullopt);

    // robot.automatic_error_recovery();
  } catch (const franka::Exception &e) {
    cout << e.what() << endl;
    return -1;
  }
  return 0;
}