#include <thread>

#include "quadruped/ros/qr_telekeyboard.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "keyboard_control");
    ros::NodeHandle nh;
    qrTeleKeyboard *keyboard = new qrTeleKeyboard(nh);
    std::cout << "---------Keyboard start receving---------" << std::endl;
    std::cout << "You can use:\n"
    "'K'             to switch control mode\n"
    "'J'             to change the gait\n"
    "'L'             to stop trot\n"
    "'I'             to sit down and exit\n\n"

    "'W' 'A' 'S' 'D' to control the robot's movement\n"
    "'Q' 'E'         to control the robot's rotation\n\n"

    "'Ctrl+C'        to exit keyboard_control" << std::endl;
    std::thread keyboardTh(&qrTeleKeyboard::run, keyboard);
    std::thread keyboardTh_default(&qrTeleKeyboard::run_default, keyboard);
    keyboardTh.join();
    keyboardTh_default.join();
    ros::shutdown();
    return 0;
}