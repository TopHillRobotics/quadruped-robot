/*
* Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
* Description: Base class of quadruped robots.
* Author: Zhao Yao
* Create: 2021-11-3
* Notes: xx
* Modify: init the file. @ Zhao Yao 2021.11.19
*/

#ifndef ASCEND_QUADRUPED_CPP_ROBOT_H
#define ASCEND_QUADRUPED_CPP_ROBOT_H

#include <iostream>
#include <string>
#include <cmath>
#include <unordered_map>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "../../config/config.h"
#include "../../config/types.h"
#include "robots/timer.h"
#include "robots/motor.h"
#include "utils/se3.h"
#include "utils/tools.h"
#include "utils/print.hpp"
#include "estimators/filter.hpp"
#include "controllers/state_dataflow.h"
#include "dynamics/floating_base_model.hpp"

#include "unitree_legged_sdk/unitree_interface.h"

namespace Quadruped {
    /**
     * @brief base class of robot.
     */
    class Robot {

    public:
        Robot();

        virtual ~Robot() = default;

        virtual void Reset();

        std::string GetControlMode()
        {
            return modeMap[controlParams["mode"]];
        }

        /** @brief update observation in each loop. */
        virtual void ReceiveObservation() = 0;

        void UpdateDataFlow();
        void ComputeMoment();

        virtual void ApplyAction(const Eigen::MatrixXf &motorCommands, MotorMode motorControlMode) = 0;

        virtual void ApplyAction(const std::vector<MotorCommand> &motorCommands, MotorMode motorControlMode)
        {};

        virtual void Step(const Eigen::MatrixXf &action, MotorMode motorControlMode) = 0;

        virtual void Step(const std::vector<MotorCommand> &motorCommands,
                          MotorMode motorControlMode)
        {};

        inline Eigen::Matrix<float, 12, 1> GetMotorAngles()
        {
            return motorAngles;
        }

        inline Eigen::Matrix<float, 12, 1> GetMotorVelocities() const
        {
            return motorVelocities;
        }

        /**
         * @brief Get robot base position in world frame.
         * @return robot base position in world frame.
         */
        inline Eigen::Matrix<float, 3, 1> GetBasePosition() const
        {
            return basePosition;
        };

        /**
         * @brief Get robot base orientation in world frame.
         * @return robot base orientation in world frame.
         */
        inline Eigen::Matrix<float, 4, 1> GetBaseOrientation() const
        {
            return baseOrientation;
        };

        /**
         * @brief Get robot base rpy in world frame.
         * @return yaw calibrated robot rpy in world frame.
         */
        inline Eigen::Matrix<float, 3, 1> GetBaseRollPitchYaw() const
        {
            return baseRollPitchYaw;
        }

        /**
         * @brief Get robot base rpy rate in base frame.
         * @return robot rpy rate in base frame
         */
        inline Eigen::Matrix<float, 3, 1> GetBaseRollPitchYawRate() const
        {
            return baseRollPitchYawRate;
        }

        inline Eigen::Matrix<float, 4, 1> GetFootForce() const
        {
            return footForce;
        }

        inline Eigen::Matrix<bool, 4, 1> GetFootContacts() const
        {
            return footContact;
        }

        inline Eigen::Matrix<float, 12, 1> GetMotorPositionGains() const
        {
            return motorKps;
        }

        inline Eigen::Matrix<float, 12, 1> GetMotorVelocityGains() const
        {
            return motorKds;
        }

        inline float GetTimeStep()
        { return timeStep; }

        inline TimerInterface &GetTimer()
        { return timer; }

        void ResetTimer()
        {
            timer.ResetStartTime();
        }

        float GetTimeSinceReset()
        {
            return timer.GetTimeSinceReset();
        }

        Eigen::Matrix<float, 3, 1>
        FootPositionInHipFrameToJointAngle(Eigen::Matrix<float, 3, 1> &footPosition, int hipSign = 1);

        Eigen::Matrix<float, 3, 1>
        FootPositionInHipFrame(Eigen::Matrix<float, 3, 1> &angles, int hipSign = 1);

        Eigen::Matrix<float, 3, 3>
        AnalyticalLegJacobian(Eigen::Matrix<float, 3, 1> &legAngles, int legId);

        Eigen::Matrix<float, 3, 4> FootPositionsInBaseFrame(Eigen::Matrix<float, 12, 1> footAngles);

        Eigen::Matrix<float, 3, 4> ComputeFootVelocitiesInBaseFrame();
    
        Vec3<float> GetEstimatedVelocityInBaseFrame()
        {
            return baseVelocityInBaseFrame;
        }

        void ComputeMotorAnglesFromFootLocalPosition(int legId,
                                                     Eigen::Matrix<float, 3, 1> footLocalPosition,
                                                     Eigen::Matrix<int, 3, 1> &jointIdx,
                                                     Eigen::Matrix<float, 3, 1> &jointAngles);

        Eigen::Matrix<float, 3, 1> ComputeMotorVelocityFromFootLocalVelocity(int legId,
                                                    Eigen::Matrix<float, 3, 1> legAngles,
                                                    Eigen::Matrix<float, 3, 1> footLocalVelocity);
 
        virtual Eigen::Matrix<float, 3, 4> GetFootPositionsInBaseFrame();

        Eigen::Matrix<float, 3, 4> GetFootPositionsInWorldFrame(bool useInput=false, Vec3<float> basePositionIn={0.f,0.f,0.f}, Quat<float> baseOrientationIn={1.f,0.f,0.f,0.f});

        Eigen::Matrix<float, 3, 3> ComputeJacobian(int legId);

        Eigen::Matrix<float, 3, 4> GetHipPositionsInBaseFrame();

        Eigen::Matrix<float, 12, 1> GetMotorPositionGains();

        Eigen::Matrix<float, 12, 1> GetMotorVelocityGains();

        std::map<int, float> MapContactForceToJointTorques(int legId, Eigen::Matrix<float, 3, 1> contractForce);

        Vec3<float> withLegSigns(const Vec3<float>& v, int legID);

        virtual bool BuildDynamicModel() {return false;}

        //config file
        std::string configFilePath;
        YAML::Node robotConfig;

        // robot params
        std::string robotName;

        float totalMass;
        float bodyMass;
        Eigen::Matrix<float, 3, 3> totalInertia, bodyInertia;
        std::vector<Mat3<float>> linkInertias;
        std::vector<float> linkMasses; // 12 links on four legs
        std::vector<float> linkLength;
        std::vector<std::vector<float>> linksComPos; // relative to body geometry frame.
        float bodyHeight;
        Vec3<float> abadLocation;
        float hipLength;
        float upperLegLength;
        float lowerLegLength;
        float footHoldOffset = 0.1f;

        Eigen::Matrix<float, 3, 1> comOffset;
        Eigen::Matrix<float, 3, 4> hipOffset, hipOffset2Com;
        Eigen::Matrix<float, 3, 4> defaultHipPosition;

        // motor params
        Eigen::Matrix<float, 12, 1> motorKps;
        Eigen::Matrix<float, 12, 1> motorKds;
        Eigen::Matrix<float, 12, 1> jointDirection = Eigen::Matrix<float, 12, 1>::Ones();
        Eigen::Matrix<float, 12, 1> jointOffset = Eigen::Matrix<float, 12, 1>::Zero();
        Eigen::Matrix<float, 12, 1> standUpMotorAngles; // default motor angle when robot stands.
        Eigen::Matrix<float, 12, 1> sitDownMotorAngles;

        std::map<std::string, int> controlParams;

        bool useRosTime = true;
        TimerInterface timer;
        float timeStep;
        float lastResetTime;
        bool initComplete = false;
        bool isSim;

//protected:
        //robot states from observation
        MovingWindowFilter<float,3> accFilter;
        MovingWindowFilter<float,3> gyroFilter;
        MovingWindowFilter<float,3> rpyFilter;
        MovingWindowFilter<float,4> quatFilter;
        MovingWindowFilter<float,12> motorVFilter;

        Eigen::Matrix<float, 3, 1> basePosition = {0.f, 0.f, A1_BODY_HIGHT}; //robot base position in world frame
        float absolutHight = 0;
        Eigen::Matrix<float, 4, 1> baseOrientation; //robot base orientation in world frame
        Eigen::Matrix<float, 3, 1> baseRollPitchYaw; //yaw calibrated robot rpy in world frame
        Eigen::Matrix<float, 3, 1> baseRollPitchYawRate; //robot rpy rate in base frame
        Eigen::Matrix<float, 3, 1> baseVelocityInBaseFrame;
        
        Eigen::Matrix<float, 12, 1> motorAngles;
        Eigen::Matrix<float, 12, 1> motorVelocities;
        Eigen::Matrix<float, 12, 1> motorddq;
        Eigen::Matrix<float, 12, 1> motortorque;
        Eigen::Matrix<float, 4, 1> footForce;
        Eigen::Matrix<bool, 4, 1> footContact;

        StateDataFlow stateDataFlow;

        FloatingBaseModel<float> model; // dynamics model for whole body control

        RobotInterface robotInterface;
        LowState lowState; // low-level state including imu data encoder data.
        HighState highState; // high-level state used for unitree sdk high-level command. 
        float yawOffset = 0.f;
        bool stop = false;
        Eigen::Matrix<float, 3, 1> gazeboBasePosition = {0.f, 0.f, A1_BODY_HIGHT}; //robot base position in world frame
        Eigen::Matrix<float, 4, 1> gazeboBaseOrientation = {1.f,0.f,0.f,0.f}; //robot base orientation in world frame
        Vec3<float> gazeboBaseVInBaseFrame;
        Eigen::Matrix<float, 3, 4> gazeboFootPositionInWorldFrame;
        
        virtual void RecordData(int beginId) {};
        virtual void SaveData(std::string fileName) {};
        const long unsigned int NumX=20000;
        const long unsigned int NumY=54;
        std::vector<float> trainData;
        std::vector<uint8_t> trainDataLabel;
        
        std::unordered_map<int, std::string> modeMap = {{0, "velocity"},
                                                         {1, "position"},
                                                         {2, "walk"},
                                                         {3, "advanced_trot"}};
        int fsmMode = 4; // K_LOCOMOTION
        int count = 0;
    };
} // namespace Quadruped

#endif //ASCEND_QUADRUPED_CPP_ROBOT_H
