#include <iostream>
#include <string>
#include <franka/robot.h>
#include <franka/gripper.h>
#include <franka/exception.h>

using namespace std;

const string ip = "192.168.100.1";
std::function<franka::JointVelocities(const franka::RobotState &, franka::Duration)> my_external_motion_generator_callback;

// // Define my_external_motion_generator_callback
// franka::JointVelocities my_external_motion_generator_callback(const franka::RobotState &robot_state, franka::Duration period)
// {
//     // do something
// }

int main()
{
    try
    {
        franka::Robot robot(ip);
        franka::Gripper gripper(ip);
        cout << "Initialized" << endl;

        // only motion generation for the beginning
        robot.control(my_external_motion_generator_callback);
    }catch (const franka::Exception &e){
        cout << e.what() << endl;
        return -1;
    }
    return 0;
}