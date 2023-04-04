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

#include "planner/qr_pose_planner.h"
#include "QuadProg++.hh"
#include "Array.hh"

namespace Quadruped {

qrPosePlanner::qrPosePlanner(qrRobot *robotIn, qrGaitGenerator *gaitGeneratorIn, qrStateEstimatorContainer* stateEstimators):
    robot(robotIn),
    gaitGenerator(gaitGeneratorIn),
    robotEstimator(stateEstimators->GetRobotEstimator()),
    groundEstimator(stateEstimators->GetGroundEstimator())
{
    /*  leg contact status  */
    for (int legId = 0; legId < 4; legId++) {
        int legState = gaitGenerator->detectedLegState[legId];
        if (legState == LegState::STANCE || legState == LegState::EARLY_CONTACT) {
            contactK[legId] = true;
        } else {
            contactK[legId] = false;
        }
    }
    ToCounterClockOrder(contactK);
    contactLegNumber = 0;
    rBH << 0.18, 0.18, -0.18, -0.18,
           -0.047, 0.047, -0.047, 0.047,
           0,    0,    0,    0;  // robot->GetHipPositionsInBaseFrame();
    
    ToCounterClockOrder(rBH);
    
    validContactPointId.clear();
    Asp = Eigen::MatrixXf::Zero(N, 3);
    bsp = Eigen::MatrixXf::Zero(N, 1);
    G = Eigen::MatrixXf::Zero(3*N, 1);
    Lambda = Eigen::MatrixXf::Ones(3*N, 1)/10.0;
    so3Phi << 0,0,0;
    
    quat << 1,0,0,0;// 0.995, 0.0998,0,0; //roll=11.5du, 
    rIB = robotEstimator->GetEstimatedPosition();
    rIBSource << 0.f, 0.f, 0.f;
    rpy << 0, 0, 0;
    poseDest << rIB,rpy;
    twist = Vec6<float>::Zero();
    std::cout << "poseDest = " << poseDest.transpose() << std::endl;
    segment.Reset(poseDest, poseDest);
}


Vec6<float> qrPosePlanner::Update(float currentTime)
{
    /// leg contact status  ///
    for (int legId = 0; legId < 4; legId++) {
        int legState = gaitGenerator->desiredLegState[legId];
        if (legState == LegState::STANCE) {
            contactK[legId] = true;
        } else {
            contactK[legId] = false;
        }
    }
    printf("pose planner contact = %d, %d, %d, %d\n", contactK[0],contactK[1],contactK[2],contactK[3]);
    ToCounterClockOrder(contactK);

    quat = robot->GetBaseOrientation();
    Vec3<float> robotComRpy = robot->GetBaseRollPitchYaw(); // world frame
    Mat3<float> Rb = robotics::math::quaternionToRotationMatrix(quat).transpose();
    Quat<float> controlFrameOrientation = groundEstimator->GetControlFrameOrientation();
    Mat3<float> Rc = robotics::math::quaternionToRotationMatrix(controlFrameOrientation).transpose();
    Vec3<float> groundRPY = groundEstimator->GetControlFrameRPY();
    Mat3<float> Rcb = Rc.transpose() * Rb;
    Vec3<float> robotComRPYInControlFrame = robotics::math::rotationMatrixToRPY(Rcb.transpose());
        
    rIBSource = robotEstimator->GetEstimatedPosition(); // in world frame
    std::cout << "rIBSource = " <<rIBSource.transpose() << std::endl; 
    poseSource << rIBSource, robotComRpy; // todo
    // rIB << 0.f, 0.f, 0.f; // init at world frame 's origin
    rIB = robotEstimator->GetEstimatedPosition(); // in world frame
    rIB_ = ProjectV(rIB);
    rBF = robot->GetFootPositionsInBaseFrame();
    ToCounterClockOrder(rBF);
    rIF = robot->GetFootPositionsInWorldFrame();
    // rIF = rIF.colwise() - rIBSource; // world frame origin at body center
    std::cout << "rIF" << rIF << std::endl;
    // rIF = Rcb * rBF; // foot positions in source control frame
    ToCounterClockOrder(rIF);
    // rBCOM = robot->comOffset; // rICOM = rIB + Phi(rBCOM);
    rBCOM << 0., 0., 0.; // assume com is overlape with geometric center point.
    // rICOMoffset = robotics::math::TransformVecByQuat(quat, rBCOM);
    rICOMoffset = Rcb * rBCOM; // in control frame
    rICOMoffset_ = ProjectV(rICOMoffset);
    rSP_ << 0, 0, 0 ;
    supportPolygonVertices.clear();
    projectedSupportPolygonVertices.clear();
    g.clear();
    contactLegNumber = 0;
    validContactPointId.clear();
    for (int i=0; i<4 ;++i) { // i is counterwise order leg index (started from FR-leg).
        if (contactK[i]) {
            validContactPointId.insert(i);
            contactLegNumber++;
            Vec3<float> r = rIF.col(i); 
            supportPolygonVertices.push_back(r); // in world frame origin at body center
            rSP_ += r;
            Vec3<float> rIFB = rIB + robotics::math::TransformVecByQuat<float>(quat, rBH.col(i)) - r;
            g.push_back(rIFB);
            // g.push_back(rIB + Rcb * rBH.col(i) - r); // in control frame
        }
    }
    Vec3<float> centerOfQuadrangle = rIF.rowwise().mean();
    rSP_ = ProjectV(rSP_ / contactLegNumber);
    rSP_ = rSP_*2.f/3.f + centerOfQuadrangle/3.f; // todo, reduce the value of the center of sp to avoid large base-moving amplitude.
    // rSP_[2] = std::max(-0.03f, (std::min(0.03f, bodyHight - rIBSource[2] - rIBSource[2] - Rcb(0,2)*rSP_[0] - Rcb(1,2)*rSP_[1]) / Rcb(2,2) + rIBSource[2])); // Rcb^T * X = Y, where Y is in control frame, and Y[2] = bodyHeight 
    rSP_[2] = bodyHight;
    std::cout << "rSP_ =  " << rSP_.transpose() << std::endl;
    N = contactLegNumber;
    if (contactLegNumber==4) {
        /* check sp is or is not convex polygen; */
        int invalidId=-1;
        for (int sourceId=1; sourceId<=2; sourceId++) {
            int destId = (sourceId + 2) % 4;
            Vec3<float>& checkPostive = supportPolygonVertices[sourceId-1];
            Vec3<float>& checkNegative = supportPolygonVertices[sourceId+1];
            Vec3<float>& sourcePoint = supportPolygonVertices[sourceId];
            Vec3<float>& destPoint = supportPolygonVertices[destId];
            
            if ((destPoint[0] - sourcePoint[0])*(checkPostive[1] - sourcePoint[1]) - 
                    (destPoint[1] - sourcePoint[1])*(checkPostive[0] - sourcePoint[0]) > 0) {
                invalidId = sourceId -1;
                break;
            }
            
            if ((destPoint[0] - sourcePoint[0])*(checkNegative[1] - sourcePoint[1]) - 
                    (destPoint[1] - sourcePoint[1])*(checkNegative[0] - sourcePoint[0]) < 0) {
                invalidId = sourceId +1;
                break;
            }
        }
        //* IF NOT CONVEX, COMPUTE ITS CLOSEURE. */
        if (invalidId>=0) {
            printf("invalidId = %d \n", invalidId);
            supportPolygonVertices.erase(supportPolygonVertices.begin()+invalidId);
            g.erase(g.begin()+invalidId);
            N--;
            validContactPointId.erase(invalidId);
        }
    }

    for (int i=0; i< N; ++i) {
        Vec3<float>& r = supportPolygonVertices[i];
        projectedSupportPolygonVertices.push_back(ProjectV(r));
    }
    Lambda.conservativeResize(3*N, Eigen::NoChange);

    /* build a QP problem and solve it, get the desired pose and Lagrange factor */
    for (int loop=0; loop < 20; ++loop) {
        float f = ComputeF();
        Eigen::MatrixXf GValue = ComputeG();
        Mat6<float> hessF = ComputeHessianF();
        std::vector<Mat6<float>> hessG = ComputeHessianG();
        Vec6<float> gradientF = ComputeGradientF();
        Eigen::MatrixXf gradientG = ComputeGradientG();
        Mat6<float> hessGSum = Mat6<float>::Zero();
        if (hessG.size() != Lambda.size()) {
            throw std::domain_error("The size of hessG and Lambda does not match!");
        }
        for (int i=0; i<hessG.size(); ++i) {
            hessGSum += Lambda(i) * hessG[i];
        }
        // solve the min L, SQP. 
        auto res = QpSolver(hessF, hessGSum, gradientF, gradientG, GValue);
        // update state
        Vec6<float> p = std::get<0>(res);
        Eigen::MatrixXf u = std::get<1>(res);
        rIB += p.head(3); // in translated world frame
        so3Phi = p.tail(3);
        Quat<float> dQuat = robotics::math::so3ToQuat(so3Phi);
        quat = robotics::math::ConcatenationTwoQuats(dQuat, quat);
        rIB_ = ProjectV(rIB);
        rBF = robotics::math::RigidTransform<float,4>({0.f,0.f,0.f}, quat, rIF.colwise() - rIB); // in base frame
        rICOMoffset = robotics::math::TransformVecByQuat(quat, rBCOM);
        rICOMoffset_ = ProjectV(rICOMoffset);
        g.clear();
        for (int i : validContactPointId) { // i is counterwise order leg index (started from FR-leg).
            Vec3<float> r = rIF.col(i);
            g.push_back(rIB + robotics::math::TransformVecByQuat<float>(quat, rBH.col(i)) - r);
        }
        for (int i=0; i<3*N; ++i) {
            Lambda(i, 0) = u(i, 0);
        }
        if (Lambda.size()!=3*N) {
            throw std::domain_error("Lambda.size()!=3*N\n");
        }
    }

    rpy = robotics::math::quatToRPY(quat); // TODO
    // rpy[1] = groundRPY[1];
    rpy[1] = (rpy[1] + groundRPY[1]) / 2.0;
    Mat3<float> sourceR = robotics::math::rpyToRotMat(robot->GetBaseRollPitchYaw()).transpose();
    // poseDest << rIB + rIBSource, rpy; // in world frame
    poseDest << rIB, rpy;
    Mat3<float> destR = robotics::math::rpyToRotMat(rpy).transpose();
    std::cout <<"poseSource = "<< poseSource.transpose() 
              << "\n poseSDest = " << poseDest.transpose() <<std::endl; 
    
    segment.Reset(poseSource, poseDest);
    // for (int i=0; i < 3; ++i) { 
    //     tarjectoryR3[i] = robotics::math::CubicSpline(currentTime,gaitGenerator->moveBaseTime, poseSource[i], rIB[i]);
    // }
    // tarjectorySo3 = robotics::math::CubicSplineInSO3(sourceR, destR, currentTime, gaitGenerator->moveBaseTime, robot->bodyInertia);
    return poseDest;
}


std::tuple<Vec6<float>,Eigen::MatrixXf> qrPosePlanner::QpSolver(Mat6<float>& hessF,
                                                              Mat6<float>& hessGSum,
                                                              Vec6<float>& gradientF,
                                                              Eigen::MatrixXf& gradientG,
                                                              Eigen::MatrixXf& GValue)
{
    quadprogpp::Matrix<double> GG(6, 6);
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            GG[i][j] = double(hessF(j, i)- hessGSum(j,i));
        }
    }

    quadprogpp::Vector<double> aa(6);
    for (int i = 0; i < 6; i++) {
        aa[i] = double(gradientF(i));
    }
    quadprogpp::Matrix<double> CICI(6, 3*N);
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 3*N; j++) {
            CICI[i][j] = double(gradientG(j, i));
        }
    }
    quadprogpp::Vector<double> bb(3*N);
    for (int i = 0; i < 3*N; i++) {
        bb[i] = double(GValue(i));
    }
    quadprogpp::Matrix<double> CECE(6, 0);
    quadprogpp::Vector<double> ee(0);
    quadprogpp::Vector<double> x(6);

    quadprogpp::Vector<double> u(0.0, 3*N);
    
    quadprogpp::solve_quadprog_test(GG, aa, CECE, ee, CICI, bb, x, u);
    Vec6<float> p;
    for (int i=0; i<6; ++i) {
        p[i] = float(x[i]);
    }
    Eigen::MatrixXf larg = Eigen::MatrixXf::Zero(3*N,1);
    for (int i=0; i < 3*N; ++i) {
        larg(i, 0) = u[i];
    }
    return {p,larg};
}


Vec6<float> qrPosePlanner::ComputeGradientF()
{
    Vec6<float> gradient = Vec6<float>::Zero();
    for (int i : validContactPointId) {
        gradient.head(3)  +=  (rIB + robotics::math::TransformVecByQuat<float>(quat, rBF.col(i)) - rIF.col(i));
        Vec3<float> r = robotics::math::TransformVecByQuat<float>(quat, rBF.col(i));
        Mat3<float> rMAT = robotics::math::vectorToSkewMat(r);
        gradient.tail(3) += rMAT*(rIB - rIF.col(i));
    }
    gradient.head(3) += omega*(rIB_ - rSP_ + rICOMoffset_); 
    gradient.tail(3) += omega*(robotics::math::vectorToSkewMat(rICOMoffset_)*(rIB_-rSP_));
    gradient *= 2.0;
    return gradient;
}


Eigen::Matrix<float, 6, 6> qrPosePlanner::ComputeHessianF() 
{
    Eigen::Matrix<float, 6, 6> Hess = Eigen::Matrix<float, 6, 6>::Zero();
    for (int i : validContactPointId) {
        Hess.block<3,3>(0,0) += Eigen::Matrix<float,3,3>::Identity();
        Vec3<float> r = robotics::math::TransformVecByQuat<float>(quat, rBF.col(i));
        Mat3<float> rMat = robotics::math::vectorToSkewMat(r);
        Hess.block<3,3>(0,3) -= rMat;
        Hess.block<3,3>(3,0) += rMat;
        Mat3<float> rIBMat = robotics::math::vectorToSkewMat(rIB);
        Mat3<float> rIFMat = robotics::math::vectorToSkewMat(rIF.col(i));
        Mat3<float> Di = ((rIBMat - rIFMat) * rMat + rMat * (rIBMat - rIFMat)) / 2.;
        Hess.block<3,3>(3,3) += Di;
    }
    
    Mat3<float> I_ = Eigen::Matrix<float,3,3>::Identity();
    // I_(2,2) = 0;
    Hess.block<3,3>(0,0) += omega * I_;
    Mat3<float> rICOMoffsetMat_ = robotics::math::vectorToSkewMat(rICOMoffset_);
    Hess.block<3,3>(0,3) -= omega * rICOMoffsetMat_;
    Hess.block<3,3>(3,0) += omega * rICOMoffsetMat_;
    Mat3<float> rIBMat_ = robotics::math::vectorToSkewMat(rIB_);
    Mat3<float> rSPMat_ = robotics::math::vectorToSkewMat(rSP_);
    Mat3<float> E = ((rIBMat_ - rSPMat_) * rICOMoffsetMat_  +  rICOMoffsetMat_ * (rIBMat_ - rSPMat_)) /2.0;
    Hess.block<3,3>(3,3) += omega * E;
    
    Hess *= 2.f;
    return Hess;
}


Eigen::MatrixXf qrPosePlanner::ComputeGradientG()
{
    Eigen::MatrixXf gradient = Eigen::MatrixXf::Zero(3*N, 3+3);   
    gradient.block(0, 0, N, 3) = Asp;
    gradient.block(0, 3, N, 3) = -Asp * robotics::math::vectorToSkewMat(rICOMoffset);
    int n=0;
    for(int i : validContactPointId) {
        Vec3<float> gi = g[n];
        gi.normalize();
        gradient.block(n+N, 0, 1, 3) =  gi.transpose();
        Vec3<float> rIBH = robotics::math::TransformVecByQuat<float>(quat, rBH.col(i));
        gradient.block(n+N, 3, 1, 3) =  -gi.transpose() * robotics::math::vectorToSkewMat(rIBH);
        n++;
    }
    gradient.block(2*N,0,N,6) = -gradient.block(N,0,N,6);
    return gradient;
}


std::vector<Mat6<float>> qrPosePlanner::ComputeHessianG() 
{
    std::vector<Mat6<float>> hessVec;
    Eigen::MatrixXf AspT = Asp.transpose();
    Mat3<float> rICOMoffsetMat = robotics::math::vectorToSkewMat(rICOMoffset);
    for (int i=0; i<N; ++i) {
        Mat6<float> hess = Mat6<float>::Zero();
        Mat3<float> AspTiMat = robotics::math::vectorToSkewMat(Vec3<float>(AspT.col(i)));
        hess.block<3,3>(3,3) = (AspTiMat*rICOMoffsetMat + rICOMoffsetMat*AspTiMat) / 2;
        hessVec.push_back(hess);
    }
    
    int n=0;
    for (int i : validContactPointId) {
        Mat6<float> hess = Mat6<float>::Zero();
        Vec3<float> gi = g[n];
        float giNorm = gi.norm();
        Mat3<float> diffRMat = gi * gi.transpose();
        hess.block<3,3>(0,0) =  Mat3<float>::Identity() / giNorm -  diffRMat / pow(giNorm, 3);
        Vec3<float> rIBH = robotics::math::TransformVecByQuat<float>(quat, rBH.col(i));
        Mat3<float> rIBHMat = robotics::math::vectorToSkewMat(rIBH);
        hess.block<3,3>(0,3) = - rIBHMat / giNorm + diffRMat * rIBHMat / pow(giNorm, 3);
        hess.block<3,3>(3,0) = - (hess.block<3,3>(0,0) * rIBHMat).transpose();
        Mat3<float> rIBMat = robotics::math::vectorToSkewMat(rIB);
        Mat3<float> rIFMat = robotics::math::vectorToSkewMat(rIF.col(i));
        Mat3<float> DHi = ((rIBMat - rIFMat) * rIBHMat + rIBHMat * (rIBMat - rIFMat)) / 2.;
        Vec3<float> dGdPhi = - gi.transpose() * rIBHMat/giNorm;
        hess.block<3,3>(3,3) = (DHi/2.f - dGdPhi * dGdPhi.transpose())/giNorm;
        hessVec.push_back(hess);
        n++;
    }
    for (int i=0; i< N; ++i) {
        hessVec.push_back(-hessVec[i+N]);
    }
    return hessVec;
}


Eigen::MatrixXf qrPosePlanner::ComputeG()
{
    G.conservativeResize(3*N, Eigen::NoChange);
    Asp.conservativeResize(N, Eigen::NoChange);
    bsp.conservativeResize(N, Eigen::NoChange);
    // assuming this's a convex polygen
    if (N==3) {
        Vec3<float> A = projectedSupportPolygonVertices[0];
        Vec3<float> B = projectedSupportPolygonVertices[1];
        Vec3<float> C = projectedSupportPolygonVertices[2];
        Vec3<float> O = (A+B+C) / 3.0;
        A = O + (1-eps) *(A-O);
        B = O + (1-eps) *(B-O);
        C = O + (1-eps) *(C-O);

        Asp << B[1]-A[1], A[0]-B[0], 0,
               C[1]-B[1], B[0]-C[0], 0,
               A[1]-C[1], C[0]-A[0] , 0;
        
        bsp << A[0]*B[1]-B[0]*A[1],
               B[0]*C[1]-C[0]*B[1],
               C[0]*A[1]-A[0]*C[1];
    
    } else if (N==4) {
        Vec3<float> A = projectedSupportPolygonVertices[0];
        Vec3<float> B = projectedSupportPolygonVertices[1];
        Vec3<float> C = projectedSupportPolygonVertices[2];
        Vec3<float> D = projectedSupportPolygonVertices[3];
        Vec3<float> O = (A+B+C+D) / 4.0;
        A = O + (1-eps) *(A-O);
        B = O + (1-eps) *(B-O);
        C = O + (1-eps) *(C-O);
        D = O + (1-eps) *(D-O);

        Asp << B[1]-A[1], A[0]-B[0],  0,
               C[1]-B[1], B[0]-C[0], 0,
               D[1]-C[1], C[0]-D[0], 0,
               A[1]-D[1], D[0]-A[0], 0;
        
        bsp << A[0]*B[1]-B[0]*A[1],
               B[0]*C[1]-C[0]*B[1],
               C[0]*D[1]-D[0]*C[1],
               D[0]*A[1]-A[0]*D[1];
    
    } else {
        std::cout << "force = "<<robot->GetFootForce().transpose() << std::endl;
        throw std::domain_error("N < 3 !\n");
    }
    
    G.block(0,0, N,1) = Asp*(rIB + rICOMoffset) - bsp;
    for (int i=0; i<N; ++i) {
        Vec3<float>& gi = g[i];
        float giNorm = gi.norm();
        G(N+i, 0) = giNorm - lMin;
        G(2*N+i, 0) = lMax - giNorm;
    }
    return G;
}


float qrPosePlanner::ComputeF()
{
    float f=0;
    for (int i : validContactPointId) {
        Vec3<float> r1 = rIB + robotics::math::TransformVecByQuat<float>(quat, rBF.col(i)) - rIF.col(i);
        f+= pow(r1.norm(),2);
    }
    Vec3<float> r2 = rSP_ - ProjectV(rIB + robotics::math::TransformVecByQuat<float>(quat, rBCOM));
    f += omega*pow(r2.norm(),2);
    return f;
}


Vec3<float> qrPosePlanner::ProjectV(Vec3<float> v)
{
    return {v[0], v[1], v[2]};
}

} // Namespace Quadruped
