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

#include "estimators/qr_anomaly_detection.h"

namespace Quadruped {

float CExp(float x, float lambda=5.0f)
{
    return 1.0f - std::exp(-lambda*x);
}


qrContactDetection::qrContactDetection(qrRobot* robotIn, qrGaitGenerator* gaitGeneratorIn, qrGroundSurfaceEstimator* groundEstimatorIn):
    robot(robotIn),
    gaitGenerator(gaitGeneratorIn),
    groundEstimator(groundEstimatorIn)
{
    /* init leg inertias  */
    for (int legId=0; legId< NumLeg; ++legId) {
        legInertias.push_back(robot->linkInertias[3*legId+2]);
        float m= robot->linkMasses[legId*3+2];
        Vec3<float> p;
        p<< 0.f,0.f, robot->lowerLegLength;
        legInertias[legId] = transformInertia(legInertias[legId], m, p);
    }
    // contact
    Eigen::Matrix<float,12,1> sensorVarianceEigen;
    sensorVarianceEigen << sigmaTorques.cwiseProduct(sigmaTorques), sigmaPz.cwiseProduct(sigmaPz), sigmaPz.cwiseProduct(sigmaPz);
    float sigmaPhase2 = sigmaPhase*sigmaPhase;
    float accelerometerVariance[4] = {sigmaPhase2,sigmaPhase2,sigmaPhase2,sigmaPhase2};
    float sensorVariance[12];
    Eigen::Map<Eigen::MatrixXf>( &sensorVariance[0], 12, 1) = sensorVarianceEigen;
    float x[4];
    pContact = Eigen::Matrix<float, 4, 4>::Ones() * 0.5;
    Eigen::Map<Eigen::MatrixXf>(&x[0], 1,4) = pContact.row(0);
    filterContact = new TinyEKF<4,12>(&x[0], 1.f, &accelerometerVariance[0], &sensorVariance[0]);

    /* init ekf matrix */
    for (int i=0; i<4; ++i) {
        for (int j=0; j < 4; ++j) {
            F[i][j] = 0.0;
            H[i][j] = 0.0;
            H[i+4][j] = 0.0;
            H[i+8][j] = 0.0;

            FSlip[i][j] = 0;
            HSlip[i][j] = 0;
        }
    }

    /* init moving window filter */
    for (int i=0; i<4; ++i) {
        windowFilter[i] = qrMovingWindowFilter<float, 3>(50);
        H[i][i] = 1.0; // torque
        H[i+4][i] = 1.0; // vz
        H[i+8][i] = 1.0; // pz

        // HSlip[i][i] = 1.0; // vx & vy in predict model
    }

    // slip
    pSlip = Eigen::Matrix<float, 2, 4>::Zero();
    Eigen::Map<Eigen::MatrixXf>(&x[0], 1,4) = pSlip.row(0);
    float obsVariance[4] = {0.01, 0.01, 0.01, 0.01};
    /* accVar and obsVar determine which variable should be more trusty */
    filterSlip = new TinyEKF<4,4>(&x[0], 1.f, &accelerometerVariance[0], &obsVariance[0]);

    std::cout << "init detection --- " <<std::endl;
}


void qrContactDetection::Update(float currentTime)
{
    Vec4<float> contactK;
    Vec4<float> swingK;
    Vec4<int> legState = gaitGenerator->legState;
    Vec4<int> desiredLegState = gaitGenerator->desiredLegState;
    Vec4<float> normalizedPhase = gaitGenerator->normalizedPhase;
    float& trueSwingEndPhaseInFullCycle = gaitGenerator->trueSwingEndPhaseInFullCycle;
    float& trueSwingStartPhaseInFullCycle = gaitGenerator->trueSwingStartPhaseInFullCycle;

    /* Compute prior contact probility and stance probility */
    for (int legId = 0; legId < NumLeg; ++legId) {
        if (robot->controlParams["mode"] == LocomotionMode::WALK_LOCOMOTION) {
            if (gaitGenerator->desiredLegState[legId]==SubLegState::TRUE_SWING) {
                normalizedPhase[legId] = (gaitGenerator->phaseInFullCycle[legId] - trueSwingStartPhaseInFullCycle) / (trueSwingEndPhaseInFullCycle - trueSwingStartPhaseInFullCycle);
            } else {
                normalizedPhase[legId] = fmod(gaitGenerator->phaseInFullCycle[legId] + (1-trueSwingEndPhaseInFullCycle),1.f) / (1-(trueSwingEndPhaseInFullCycle - trueSwingStartPhaseInFullCycle));
            }
            if (desiredLegState[legId] != SubLegState::TRUE_SWING) {
                contactK[legId] = 0.5 * (std::erf(normalizedPhase[legId] / (sigmaPhase * M_SQRT2))
                    + std::erf((1.0 - normalizedPhase[legId]) / (sigmaPhase * M_SQRT2))
                );
                swingK[legId] = 0.f;
            } else {
                swingK[legId] = 0.5 * (2.0 + std::erf(-normalizedPhase[legId] / (sigmaPhase * M_SQRT2))
                    + std::erf((normalizedPhase[legId] - 1.0) / (sigmaPhase * M_SQRT2))
                );
                contactK[legId] = 0.f;
            }
        } else {
            if (desiredLegState[legId] == LegState::STANCE) {
                contactK[legId] = 0.5 * (std::erf(normalizedPhase[legId] / (sigmaPhase * M_SQRT2))
                    + std::erf((1.0 - normalizedPhase[legId]) / (sigmaPhase * M_SQRT2))
                );
                swingK[legId] = 0.f;
            } else {
                swingK[legId] = 0.5 * (2.0 + std::erf(-normalizedPhase[legId] / (sigmaPhase * M_SQRT2))
                    + std::erf((normalizedPhase[legId] - 1.0) / (sigmaPhase * M_SQRT2))
                );
                contactK[legId] = 0.f;
            }
        }

        pContact(0, legId) = contactK[legId] + swingK[legId]; // probability of leg contact
    }

    // pz
    Eigen::Matrix<float,3,4> footPositionInBaseFrame = robot->GetFootPositionsInBaseFrame();
    Eigen::Matrix<float,3,4> footPositionInWorldFrame = robot->GetFootPositionsInWorldFrame();
    Vec4<float> controlFrameOrientationLastT = groundEstimator->GetControlFrameOrientation();
    Vec3<float> controlFramePLastT = groundEstimator->bodyPositionInWorldFrame;
    Eigen::Matrix<float,3,4> footPositionInLastControlFrame = robotics::math::RigidTransform<float, 4>(controlFramePLastT,controlFrameOrientationLastT,footPositionInWorldFrame);
    Vec4<float> zc, dz;
    Vec4<float> ppz;

    /* Compute prior contact probility and stance probility using another factor*/
    for (int legId=0; legId<NumLeg; ++legId) {
        /*
         * for trot
         * zc[legId] = groundEstimator->GetZInBaseFrame(footPositionInBaseFrame(0, legId),footPositionInBaseFrame(1, legId));
         * ppz[legId] = 0.5 *(1.0+std::erf((zc[legId]-footPositionInBaseFrame(2,legId))/(M_SQRT2 * sigmaPz[legId])));
         * ppz[legId] = std::exp(-20*std::abs(zc[legId]-footPositionInBaseFrame(2,legId)));
         */

        // for walk
        zc[legId] = groundEstimator->GetZInControlFrame(footPositionInLastControlFrame(0,legId),footPositionInLastControlFrame(1, legId));
        dz[legId] = footPositionInLastControlFrame(2,legId) - zc[legId];
        ppz[legId] = 1.0-std::erf(std::max(dz[legId],0.f)/(M_SQRT2 * sigmaPz[legId]/2));
        // ppz[legId] = std::exp(-20*std::abs(-dz[legId]));
        pContact(3,legId) = ppz[legId];
    }

    // vz in Control Frame
    float pStance[4];
    float pSwing[4];
    Eigen::Matrix<float,3,4> footVInBaseFrame = robot->ComputeFootVelocitiesInBaseFrame();
    Eigen::Matrix<float,3,4> filtedFootVInBaseFrame;
    /* compute contact probility using velocity in z axis in control frame */
    for (int legId=0; legId<NumLeg; ++legId) {
        filtedFootVInBaseFrame.col(legId) = windowFilter[legId].CalculateAverage(footVInBaseFrame.col(legId));
        float vz = filtedFootVInBaseFrame(2, legId);
        float vzUpper = std::max(vz, 0.f);
        float vzLower = std::min(vz, 0.f);
        float lastVz  = lastFootVzInControlFrame[legId];
        float lastVzUpper = std::max(lastVz, 0.f);
        float lastVzLower = std::min(lastVz, 0.f);

        pStance[legId]= - std::min(CExp(vzLower*vzLower)- CExp(lastVzLower*lastVzLower), 0.f);
        pStance[legId] = 0.5 *(1+ std::erf((pStance[legId]*20-0.3)/0.2));
        pSwing[legId] = std::max(CExp(vzUpper*vzUpper)- CExp(lastVzUpper*lastVzUpper), 0.f);
        pSwing[legId] = 0.5 *(1+ std::erf((pSwing[legId]*20-0.3)/0.2));

        // method 1
        pContact(2, legId) = std::exp(-5*std::abs(vz));// * ppz[legId];
        // pContact(2, legId) = (1-std::erf(std::abs(vz)/(M_SQRT2*0.3))) * ppz[legId];

         /*
          * method 2
          * // if (pStance[legId]*ppz[legId]*5>0.3) {
          * if (pStance[legId] > 0.8)
          * {
          *     pContact(2, legId) = 0.8;
          * } else if (pSwing[legId] > 0.8) {
          *     pContact(2, legId) = 0.2;
          * } else {
          *     // pContact(2, legId) = pStance[legId];
          * }
          *
          */

        lastFootVzInControlFrame[legId] = vz;
    }

    // torque/force
    Vec4<float> externalTorques = JointObserver(currentTime);
    auto forces = robot->GetFootForce();
    /* compute contact probility using terque */
    for (int legId=0; legId< NumLeg; ++legId) {
        pContact(1, legId) = 0.5 * (1.0+std::erf((-externalTorques[legId]-meanTorques[legId])/(M_SQRT2 * sigmaTorques[legId])));
    }

    /* use kalman filter to compute fused contact probility  */
    for (int i=0; i<NumLeg; ++i) {
        fx[i] = pContact(0, i);
        hx[i] = fx[i];
        hx[i+4] = fx[i+4];
        hx[i+8] = fx[i+8];
        z[i] = pContact(1,i);
        z[i+4] = pContact(2,i);
        z[i+8] = pContact(3,i);
    }

    filterContact->step(fx, F, hx, H, z);
    for (int i=0; i<NumLeg; ++i) {
        if (isContact[i]==true) { // estimated stance
            isContact[i] = (filterContact->getX(i) > thresold[1]);
        } else { // swing
            isContact[i] = (filterContact->getX(i) > thresold[0]);
        }
    }

    /* UpdateSlip */
    auto dequeLen = vQue.size();
    if (dequeLen >= 500) {
        // The left most value to be subtracted from the moving sum.
        Vec3<float> vPop = vQue.front();
        vQue.pop_front();
        vSum -= vPop;
    }
    vSum += robot->baseVelocityInBaseFrame;
    vQue.push_back(robot->baseVelocityInBaseFrame);
    vAvg = vSum / float(dequeLen);

    float accumX=0.0, accumY=0.0;
    for (auto& item: vQue) {
        accumX  += (item[0]-vAvg[0])*(item[0]-vAvg[0]);
        accumY  += (item[1]-vAvg[1])*(item[1]-vAvg[1]);
    }
    float stdevX = std::sqrt(accumX/(dequeLen-1));
    float stdevY = std::sqrt(accumY/(dequeLen-1));

    Vec4<float> pvx, pvy;
    Eigen::Matrix<float,3,4> vDetection;
    /* if the leg contact but the velocity is a little big,
     * then maybe there is a slip happend
     */
    for (int legId=0; legId<NumLeg; ++legId) {
        if (isContact[legId]) {
            vDetection.col(legId) = filtedFootVInBaseFrame.col(legId) + vAvg; // foot v in base frame
            vDetection.col(legId) = robot->stateDataFlow.baseRInControlFrame * vDetection.col(legId);
            pvx[legId] = 0.5 * (1+std::erf(vDetection(0,legId) / (M_SQRT2*0.05))); // 0.2 for trot
            pvy[legId] = 0.5 * (1+std::erf(vDetection(1, legId) / (M_SQRT2*0.05/3.0)));
            pSlip(0, legId) = (0.75*pvx[legId] +  0.25*pvy[legId]) * filterContact->getX(legId);
        } else {
            pSlip(0, legId) =0.f;
        }
    }

    /* KF filter for slip detection */
    for (int i=0; i<NumLeg ;++i) {
        fxSlip[i] = pSlip(0, i);
        hxSlip[i] = fxSlip[i];
        zSlip[i] = pSlip(1,i);
    }
    filterSlip->step(fxSlip, FSlip, hxSlip, HSlip, zSlip);
    for (int i=0; i<NumLeg; ++i) {
        isSlip[i] = (filterSlip->getX(i) > 0.6); // 0.6 for trot, 0.7 for walk
    }

    count++;
}


void qrContactDetection::UpdateSlip(float currentTime)
{
    // todo
}


void qrContactDetection::GMObserver(float currentTime)
{
    // todo
}


Vec4<float> qrContactDetection::JointObserver(float currentTime)
{
    Eigen::Matrix<float,12,1> jointVs = robot->GetMotorVelocities();
    Eigen::Matrix<float,12,1> ddq = robot->motorddq;//(jointVs - lastJointVs) / (currentTime - lastTime); //

    /* compute leg external torque on lower legs */
    for (int legId=0; legId < NumLeg; ++legId) {
        float mgl = robot->linkMasses[3*legId + 2] * 9.8 *robot->lowerLegLength; // todo
        Vec3<float> ddq_ =  ddq.block<3,1>(3*legId,0);
        float externalTorque = (legInertias[legId]*Vec3<float>(0.f, ddq_[2], 0)).array()[1] + mgl - robot->motortorque[3*legId+2];
        // externalForces.col(legId) = robot->ComputeJacobian(legId).transpose().inverse() * externalTorque;
        externalTorques[legId] = externalTorque;
    }

    lastJointVs = jointVs;
    lastTime = currentTime;
    return externalTorques;
}


Eigen::Matrix<float,3,4> WorkspaceDetection::Update()
{
    // Cohen-Sutherland 线裁剪算法
    Eigen::Matrix<float,3,4> footPositionsInBaseFrame = robot->GetFootPositionsInBaseFrame();
    Eigen::Matrix<float,3,4> clipedFootPositions;
    clipedFootPositions.setZero();
    Eigen::Matrix<float,3,4> hipPositionsInBaseFrame = robot->GetDefaultHipPosition();

    for (int legId=0; legId<NumLeg; ++legId) {
        Vec3<float> offset = {hipPositionsInBaseFrame(0,legId), hipPositionsInBaseFrame(1, legId), -robot->bodyHeight};
        Vec3<float> regularlizedP = footPositionsInBaseFrame.col(legId) - offset;
        // check regularlizedP vector locates in work_space
        Vec3<bool> absGreaterThenSpace =  regularlizedP.cwiseAbs().array()  > allowedWorkSpace.array();
        Vec3<bool> greaterThenSpace = regularlizedP.array() > allowedWorkSpace.array();
        uint8_t code = 0b000000;
        float ratio = 1.f;
        for (int posId=0;posId<3; posId++) {
            if (absGreaterThenSpace[posId]) {
                if (greaterThenSpace[posId]) { // axis+ = 1
                    code = code | (0b000001<<(posId*2+1));
                } else { // axis- = 1
                    code = code | (0b000001<<(posId*2));
                }
            }
        }
        Vec3<float> p = regularlizedP;
        if (code > 0) { // outside the box
            if (code & 0b000001) { // intersect with x-
                ratio = allowedWorkSpace[0]/(-p[0]);
                if (ratio<1.0) p *= ratio;
            } else if (code & 0b000010) { // intersect with x+
                ratio = allowedWorkSpace[0]/p[0];
                if (ratio<1.0) p *= ratio;
            }
            if (code & 0b000100) {  // intersect with y-
                ratio = allowedWorkSpace[1]/(-p[1]);
                if (ratio<1.0) p *= ratio;
            } else if (code & 0b001000) {  // intersect with y+
                ratio = allowedWorkSpace[1]/p[1];
                if (ratio<1.0) p *= ratio;
            }
            if (code & 0b010000) {  // intersect with z-
                ratio = allowedWorkSpace[2]/(-p[2]);
                if (ratio<1.0) p *= ratio;
            } else if (code & 0b100000) { // intersect with z+
                ratio = allowedWorkSpace[2]/p[2];
                if (ratio<1.0) p *= ratio;
            }
            printf("[warning] the leg %d, is outside the workspace!", legId);
        }
        clipedFootPositions.col(legId) = p + offset;
    }

    return clipedFootPositions; // foot displacement in base frame
}

} // Namespace Quadruped
