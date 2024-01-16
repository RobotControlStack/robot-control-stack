#include <iostream>
#include <string>
#include <franka/robot.h>
#include <franka/gripper.h>
#include <franka/exception.h>
#include "FR3.h"

using namespace std;

const string ip = "192.168.100.1";
const string fn = "robot.urdf";

int main()
{
    try
    {
        FR3 robot(ip, fn);
        // robot.automatic_error_recovery();
        // std::cout << (robot.get_ee_state()) << std::endl;
        // std::cout << (robot.get_joint_state()) << std::endl;

        // std::cout << std::get<0>(robot.get_ee_pose()) << std::endl;
        // std::cout << std::get<1>(robot.get_ee_pose()) << std::endl;

        // std::cout << "WARNING: This example will move the robot! "
        //           << "Please make sure to have the user stop button at hand!" << std::endl
        //           << "Press Enter to continue..." << std::endl;
        // std::cin.ignore();
        // // robot.move_extend_arm(0.1);
        // robot.move_home(0.1);
    }
    catch (const franka::Exception &e)
    {
        cout << e.what() << endl;
        return -1;
    }
    return 0;
}