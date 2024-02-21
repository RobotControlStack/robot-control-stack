#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/robot.h>

#include <iostream>
#include <optional>
#include <string>

#include "FR3.h"

using namespace std;

const string ip = "192.168.101.1";
const string fn = "models/urdf/fr3.urdf";

int main() {
  try {
    FR3 robot(ip, fn);
    robot.automatic_error_recovery();
    // robot.move_home();

    auto rs = robot.getCartesianPosition();
    rs.translation() += Eigen::Vector3d(0, 0, 0.1);

    robot.move_cartesian(rs, 5.0, std::nullopt);

    // robot.automatic_error_recovery();
  } catch (const franka::Exception &e) {
    cout << e.what() << endl;
    return -1;
  }
  return 0;
}