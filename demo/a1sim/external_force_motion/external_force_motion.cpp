#include "quadruped/ros/qr_external_force.h"
using namespace std;

int main(int argc, char **argv)
{

    ros::init(argc, argv, "external_force_motion");
    qrTeleForceCmd remote;
    signal(SIGINT,quit);
    remote.keyLoop();
    return(0);
}
