#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/robot.h>
#include <rcs/Kinematics.h>

#include <iostream>
#include <memory>
#include <optional>
#include <string>

#include "Franka.h"

using namespace std;

const string ip = "192.168.101.1";
const string mjcf_path = "assets/fr3/mjcf/fr3_0.xml";

int main() {
  try {
    auto ik =
        make_shared<rcs::common::Pin>(mjcf_path, "attachment_site_0", false);
    rcs::hw::Franka robot(ip, ik);
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
  } catch (const franka::Exception& e) {
    cout << e.what() << endl;
    return -1;
  }
  return 0;
}