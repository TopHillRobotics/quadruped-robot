/*!
    \file examples/dr_example/dr_example.cc
    \brief Domain randomization example client
    \author João Borrego : jsbruglie
*/

#include "dr_example.hh"

// Reduce verbosity
using namespace ignition::math;

int main(int _argc, char **_argv)
{
    // Load gazebo as a client
    gazebo::client::setup(_argc, _argv);

    DRInterface interface;
    DRRequest msg;

    // Shadowhand parameters
    std::string model("shadowhand");
    std::string joint("rh_FFJ3");
    std::string joint_scoped = model + "::" + joint;
    std::string link("rh_forearm");
    std::vector<std::string> visuals {
        "rh_forearm","rh_wrist","rh_palm","rh_imu",
        "rh_ffknuckle","rh_ffproximal","rh_ffmiddle","rh_ffdistal","rh_fftip",
        "rh_mfknuckle","rh_mfproximal","rh_mfmiddle","rh_mfdistal","rh_mftip",
        "rh_rfknuckle","rh_rfproximal","rh_rfmiddle","rh_rfdistal","rh_rftip",
        "rh_lfmetacarpal",
        "rh_lfknuckle","rh_lfproximal","rh_lfmiddle","rh_lfdistal","rh_lftip",
        "rh_thbase","rh_thproximal","rh_thhub",
        "rh_thmiddle","rh_thdistal","rh_thtip"};
    for (int i = 0; i < visuals.size(); i++) {
        // Obtain full names
        visuals.at(i) = model + "::" + visuals.at(i); 
    }
    std::string parent("shadowhand");
    int pid_type = DRInterface::POSITION;
    double inf = INFINITY;

    // Disable gravity
    interface.addGravity(msg, Vector3d(0,0,0));
    // Rescale model
    interface.addModelScale(msg, model, Vector3d(2,2,2) );
    // Update link mass
    interface.addLinkMass(msg, model, link, 0.1);
    // Update joint properties
    interface.addJoint(msg, model, joint, -0.01, 0.7, 10, 10, inf, 200); 
    // Update PID controller
    interface.addModelCmd(msg, model, joint_scoped, pid_type, 100, inf, inf);
    // Blocking publish call 
    interface.publish(msg, true);
    
    // Update link colors
    for (int i = 0; i < 20; i++)
    {
        for (auto const visual : visuals)
        {
            gazebo::msgs::Visual color_msg;
            double r = getRandomDouble(0,1);
            double g = getRandomDouble(0,1);
            double b = getRandomDouble(0,1);
            interface.addColors(color_msg, visual, parent,
                Color(r, g, b),
                Color(r, g, b),
                Color(0.1, 0,    0.1),
                Color(0.2, 0.2,  0.2, 64));
            interface.publish(color_msg);
        }
        waitMs(500);
    }

    // Shut down
    gazebo::client::shutdown();
    return 0;
}

//////////////////////////////////////////////////

// Random integer generator
std::random_device rng;
std::mt19937 mt_rng(rng());
std::uniform_int_distribution<int> uniform_dist;

//////////////////////////////////////////////////
int getRandomInt(int min, int max)
{
    int aux = uniform_dist(mt_rng);
    return aux % (max - min + 1) + min;;
}

//////////////////////////////////////////////////
double getRandomDouble(double min, double max)
{
    double aux = ((double) uniform_dist(mt_rng)) / (double) RAND_MAX;
    return aux * (max - min) + min;
}

/////////////////////////////////////////////////
void inline waitMs(int delay)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(delay));
}
