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

#include <xpp_vis/ros_msg_helper.h>

namespace xpp {

std::array<double, 3> getRGB(Color color) {
  switch (color) {
    case Color::blue:
      return {0, 0.4470, 0.7410};
    case Color::orange:
      return {0.8500, 0.3250, 0.0980};
    case Color::yellow:
      return {0.9290, 0.6940, 0.1250};
    case Color::purple:
      return {0.4940, 0.1840, 0.5560};
    case Color::green:
      return {0.4660, 0.6740, 0.1880};
    case Color::red:
      return {0.6350, 0.0780, 0.1840};
    case Color::black:
      return {0.25, 0.25, 0.25};
    default:
      return {0.0, 0.0, 0.0};
  }
}

geometry_msgs::Point getPointMsg(const Eigen::Vector3d& point) {
  geometry_msgs::Point pointMsg;
  pointMsg.x = point.x();
  pointMsg.y = point.y();
  pointMsg.z = point.z();
  return pointMsg;
}

geometry_msgs::Vector3 getVectorMsg(const Eigen::Vector3d& vec) {
  geometry_msgs::Vector3 vecMsg;
  vecMsg.x = vec.x();
  vecMsg.y = vec.y();
  vecMsg.z = vec.z();
  return vecMsg;
}

geometry_msgs::Quaternion getOrientationMsg(const Eigen::Quaterniond& orientation) {
  geometry_msgs::Quaternion orientationMsg;
  orientationMsg.x = orientation.x();
  orientationMsg.y = orientation.y();
  orientationMsg.z = orientation.z();
  orientationMsg.w = orientation.w();
  return orientationMsg;
}

std_msgs::Header getHeaderMsg(const std::string& frame_id, const ros::Time& timeStamp) {
  std_msgs::Header header;
  header.frame_id = frame_id;
  header.stamp = timeStamp;
  return header;
}

visualization_msgs::Marker getLineMsg(std::vector<geometry_msgs::Point>&& points, std::array<double, 3> color, double lineWidth) {
  visualization_msgs::Marker line;
  line.type = visualization_msgs::Marker::LINE_STRIP;
  line.scale.x = lineWidth;
  line.color = getColor(color);
  line.points = std::move(points);
  line.pose.orientation = getOrientationMsg({1., 0., 0., 0.});
  return line;
}

std_msgs::ColorRGBA getColor(std::array<double, 3> rgb, double alpha /* = 1.0*/) {
  std_msgs::ColorRGBA colorMsg;
  colorMsg.r = rgb[0];
  colorMsg.g = rgb[1];
  colorMsg.b = rgb[2];
  colorMsg.a = alpha;
  return colorMsg;
}


std_msgs::ColorRGBA getColor(Color color, double alpha) {
  const auto rgb = getRGB(color);
  std_msgs::ColorRGBA colorMsg;
  colorMsg.r = rgb[0];
  colorMsg.g = rgb[1];
  colorMsg.b = rgb[2];
  colorMsg.a = alpha;
  return colorMsg;
}


visualization_msgs::Marker getLineMsg(std::vector<geometry_msgs::Point>&& points, Color color, double lineWidth) {
  visualization_msgs::Marker line;
  line.type = visualization_msgs::Marker::LINE_STRIP;
  line.scale.x = lineWidth;
  line.color = getColor(color);
  line.points = std::move(points);
  line.pose.orientation = getOrientationMsg({1., 0., 0., 0.});
  return line;
}

visualization_msgs::Marker getLineMsg(std::deque<geometry_msgs::Point>& points, Color color, double lineWidth) {
  visualization_msgs::Marker line;
  line.type = visualization_msgs::Marker::LINE_STRIP;
  line.scale.x = lineWidth;
  line.color = getColor(color);
  std::copy(points.begin(), points.end(), std::back_inserter(line.points));
  line.pose.orientation = getOrientationMsg({1., 0., 0., 0.});
  return line;
}

}  // namespace xpp
