
#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/robot.h>

#include <chrono>
#include <iostream>
#include <string>
#include <thread>

using namespace std;

const string ip2mirror = "192.168.100.1";
const string ip2control = "192.168.100.1";

int main() {
  try {
    franka::Robot robot2mirror(ip2mirror);
    franka::Gripper gripper2mirror(ip2mirror);

    franka::Robot robot2control(ip2control);
    franka::Gripper gripper2control(ip2control);

    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!"
              << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    std::array<double, 7> initial_position;
    double time = 0.0;

    robot2control.control(
        [&initial_position, &time, &robot2control, &robot2mirror](
            const franka::RobotState& robot_state,
            franka::Duration period) -> franka::JointPositions {
          time += period.toSec();

          if (time == 0.0) {
            initial_position = robot_state.q_d;
          }

          // read the mirror robot state

          franka::RobotState state = robot2mirror.readOnce();
          franka::JointPositions output(state.q);

          // catch ctrl-c and return franka::MotionFinished(output)?
          return output;
        });

  } catch (const franka::Exception& e) {
    cout << e.what() << endl;
    return -1;
  }
  return 0;
}