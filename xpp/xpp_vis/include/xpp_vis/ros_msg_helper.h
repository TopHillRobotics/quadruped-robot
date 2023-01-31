/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once

#include <array>
#include <deque>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/MarkerArray.h>

namespace xpp {
enum class Color { blue, orange, yellow, purple, green, red, black };

geometry_msgs::Vector3 getVectorMsg(const Eigen::Vector3d& vec);

geometry_msgs::Point getPointMsg(const Eigen::Vector3d& point);

geometry_msgs::Quaternion getOrientationMsg(const Eigen::Quaterniond& orientation);

std_msgs::Header getHeaderMsg(const std::string& frame_id, const ros::Time& timeStamp);

visualization_msgs::Marker getLineMsg(std::vector<geometry_msgs::Point>&& points, std::array<double, 3> color, double lineWidth);

std_msgs::ColorRGBA getColor(std::array<double, 3> rgb, double alpha = 1.0);

std_msgs::ColorRGBA getColor(Color color, double alpha = 1.0);

visualization_msgs::Marker getLineMsg(std::vector<geometry_msgs::Point>&& points, Color color, double lineWidth);

visualization_msgs::Marker getLineMsg(std::deque<geometry_msgs::Point>& points, Color color, double lineWidth);


std::array<double, 3> getRGB(Color color);



}  // namespace zpp