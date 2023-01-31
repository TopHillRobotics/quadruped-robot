/*
 *  Copyright (C) 2018 João Borrego
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *  
 *      http://www.apache.org/licenses/LICENSE-2.0
 *      
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

/*!
    \file world_utils/MoveObject.hh
    \brief Move Object class

    Class for object with a pending move operation

    \author João Borrego : jsbruglie
    \author Rui Figueiredo : ruipimentelfigueiredo
*/

// Gazebo
#include "gazebo/common/Plugin.hh"

#include <string>

/// \brief Object with a pending move operation
class MoveObject
{
    /// \brief Object name
    public: std::string name;
    /// \brief Whether object is light
    public: bool is_light;
    /// \brief Object new pose
    public: ignition::math::Pose3d pose;

    /// \brief Constructs the object
    /// \param _name        Object name
    /// \param _is_light    Whether the object is a light
    /// \param _pose        Object pose
    public: MoveObject(
        std::string & _name,
        bool _is_light,
        ignition::math::Pose3d & _pose);
};
