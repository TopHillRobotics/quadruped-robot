/*
 * Copyright (C) 2017 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifndef EXAMPLES_PLUGINS_GAITPARAMETERGUIPLUGIN_HH_
#define EXAMPLES_PLUGINS_GAITPARAMETERGUIPLUGIN_HH_

#include <ros/ros.h>
#include <ros/advertise_options.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo/transport/transport.hh>
#include "unitree_legged_msgs/GaitParameter.h"

struct Gait
  {
    QString gait_name;
    QList<double> stance_duration;
    QList<double> duty_factor;
    QList<double> init_full_cycle;
    QList<int> init_leg_state;
    double contact_detection_phase_threshold;
  };
  
  Q_DECLARE_METATYPE(Gait)

namespace gazebo
{
    
    class GAZEBO_VISIBLE GaitParameterGuiPlugin : public GUIPlugin
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Parent widget
      public: GaitParameterGuiPlugin();

      /// \brief Destructor
      public: virtual ~GaitParameterGuiPlugin();

      // Documentation inherited
      public: virtual void Load(sdf::ElementPtr _elem) override;

      ros::NodeHandle* rosnode;

      ros::Publisher gait_param_pub;
    };
}
#endif
