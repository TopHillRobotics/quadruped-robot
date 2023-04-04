// The MIT License

// Copyright (c) 2022
// Robot Motion and Vision Laboratory at East China Normal University
// Contact: tophill.robotics@gmail.com

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "controllers/qr_stance_leg_controller_interface.h"

namespace Quadruped {

qrStanceLegControllerInterface::qrStanceLegControllerInterface(
    qrRobot *robot,
    qrGaitGenerator *gait_generator,
    qrStateEstimatorContainer* state_estimators,
    qrComAdjuster *com_adjuster,
    qrPosePlanner *pose_planner,
    qrFootholdPlanner *foothold_planner,
    qrUserParameters& user_parameters,
    std::string config_filepath)
{
    c = nullptr;
    c1 = new TorqueStanceLegController(
             robot,
             gait_generator,
             state_estimators,
             com_adjuster,
             pose_planner,
             foothold_planner,
             user_parameters,
             config_filepath);

    c2 = new MPCStanceLegController(
             robot,
             gait_generator,
             state_estimators,
             com_adjuster,
             pose_planner,
             foothold_planner,
             user_parameters,
             config_filepath);

    if (robot->controlParams["mode"]==LocomotionMode::ADVANCED_TROT) {
        c = c2;
    } else {
        std::cout << "[STANCE CONTROLLER INTERFACE] use TorqueStanceLegController" <<std::endl;
        // TorqueStanceLegController, TorqueStanceLegControllerMPC
        c = c1;
    }

}


qrStanceLegControllerInterface::~qrStanceLegControllerInterface()
{
    if (c1) {
        delete c1;
    }
    if (c2) {
        delete c2;
    }
    c = nullptr;
    c1 = nullptr;
    c2 = nullptr;
}


void qrStanceLegControllerInterface::Reset(float current_time)
{
    if (c->robot->controlParams["mode"] != LocomotionMode::ADVANCED_TROT) {
        c = c1;
    } else {
        c = static_cast<MPCStanceLegController*>(c2);
    }
    c->Reset(current_time);
}


void qrStanceLegControllerInterface::Update(float current_time)
{
    c->Update(current_time);
}


std::tuple<std::map<int, qrMotorCommand>, Eigen::Matrix<float, 3, 4>> qrStanceLegControllerInterface::GetAction()
{
    return c->GetAction();
}

} // namespace Quadruped
