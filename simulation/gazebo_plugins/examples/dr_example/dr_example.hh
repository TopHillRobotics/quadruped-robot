/*!
    \file examples/dr_example/dr_example.hh
    \brief Domain randomization example client headers
    \author João Borrego : jsbruglie
*/

// Gazebo

// Gazebo
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

// Custom messages
#include "dr_request.pb.h"
// Domain randomization plugin interface
#include "DRInterface.hh"

// Required fields workaround
#include <limits>
// Sleep
#include <chrono>
#include <thread>

/// Declaration for request message type
typedef gap::msgs::DRRequest DRRequest;

// TODO - Migrate to utils

/// \brief Get a random integer in a given interval
///
/// Value is sampled from uniform distribution
///
/// \param min Interval lower bound
/// \param max Interval upper bound
/// \return Random integer
int getRandomInt(int min, int max);

/// \brief Get a random double in a given interval
///
/// Value is sampled from uniform distribution
///
/// \param min Interval lower bound
/// \param max Interval upper bound
/// \return Random double
double getRandomDouble(double min, double max);


/// \brief Waits a given number of ms
/// \param delay Amount of ms to wait
void inline waitMs(int delay);
