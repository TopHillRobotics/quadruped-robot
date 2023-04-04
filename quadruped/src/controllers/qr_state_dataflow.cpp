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

#include "controllers/qr_state_dataflow.h"


qrUserParameters::qrUserParameters(std::string filePath)
{
    YAML::Node userConfig = YAML::LoadFile(filePath);
    stairsTime = userConfig["stairsTime"].as<float>();
    stairsVel = userConfig["stairsVel"].as<float>(); 
    controlFrequency = userConfig["controlFrequency"].as<unsigned int>();
    
    filterWindowSize = userConfig["filterWindowSize"].as<unsigned int>();
    
    accelerometerVariance = userConfig["accelerometerVariance"].as<float>();
    
    sensorVariance = userConfig["sensorVariance"].as<float>();
    
    initialVariance = userConfig["initialVariance"].as<float>();
    
    movingWindowFilterSize = userConfig["movingWindowFilterSize"].as<int>();
    
    desiredHeight = userConfig["desiredHeight"].as<float>();
    
    std::vector<float> desiredSpeed_ = userConfig["desiredSpeed"].as<std::vector<float>>();
    desiredSpeed = Eigen::MatrixXf::Map(&desiredSpeed_[0], 3, 1);
    
    desiredTwistingSpeed = userConfig["desiredTwistingSpeed"].as<float>();

    footClearance = userConfig["footClearance"].as<float>();
    std::vector<float> frictionCoeffs_ = userConfig["frictionCoeffs"].as<std::vector<float>>();
    frictionCoeffs = Eigen::MatrixXf::Map(&frictionCoeffs_[0], 4, 1);
    
    swingKp = userConfig["swingKp"].as<std::map<std::string, std::vector<float>>>();
    
    computeForceInWorldFrame = userConfig["computeForceInWorldFrame"].as<bool>();
    
    useWBC = userConfig["useWBC"].as<bool>();
    
    std::cout << "init UserParameters finish\n" ;
}


Quadruped::qrStateDataFlow::qrStateDataFlow()
{
    footPositionsInBaseFrame.setZero();
    footVelocitiesInBaseFrame.setZero();

    baseVInWorldFrame.setZero();
    baseWInWorldFrame.setZero();
    baseLinearAcceleration.setZero();

    footJvs = std::vector<Mat3<float>>(4, Mat3<float>::Identity());
    estimatedFootForce.setZero();
    estimatedMoment.setZero();

    zmp.setZero();
    baseRMat.setIdentity();
    groundRMat.setIdentity();
    groundOrientation << 1.f, 0.f, 0.f, 0.f;
    baseRInControlFrame.setIdentity();
}
