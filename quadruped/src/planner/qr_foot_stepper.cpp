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

#include "planner/qr_foot_stepper.h"
#include "quadprogpp/QuadProg++.hh"
#include "quadprogpp/Array.hh"

qrFootStepper ::qrFootStepper (qrTerrain& terrain, float defaultFootholdOffset, std::string level)
{
    switch (terrain.terrainType)
    {
        case TerrainType::PLUM_PILES: {
            for (qrGap* gap : terrain.gaps) {
                gaps.push_back(*gap);
            }
        } break;
        case TerrainType::STAIRS: {
            stairUp.height = 0.1;
            stairUp.width = 0.2;
            stairUp.length = 1.0;
            stairUp.k = 3;
            stairUp.startPoint << 1.0f, 0.f,0.f;
            stairDown.height = 0.1;
            stairDown.width = 0.2;
            stairDown.length = 1.0;
            stairDown.k = 3; 
            stairDown.startPoint << 2.4f,0.f,0.2f;
        } break;
    default:
        break;
    }
    
    defaultFootholdDelta = defaultFootholdOffset; // only alone to the X axis.
    meetGap = false;
    lastFootholdsOffset = Eigen::Matrix<float, 3, 4>::Zero();
    nextFootholdsOffset = Eigen::Matrix<float, 3, 4>::Zero();
    nextFootholdsOffset.row(0)
        << defaultFootholdOffset, defaultFootholdOffset, defaultFootholdOffset, defaultFootholdOffset; // x DIRECTION
    // init qp param
    G.resize(1, 1);
    G[0][0] = 1.0;
    a.resize(1);
    a[0] = 0.;

    CE.resize(1, 0);
    e.resize(0);
    CI.resize(1, 6);
    CI[0][0] = 1.0;
    for (int i = 1; i < 6; ++i) {
        CI[0][i] = -1.0;
    }
    CI[0][5] =-1.;
    b.resize(6);

    x.resize(1);
    b[0] = -defaultFootholdDelta;
    b[5] = -1. * (MAXIMUM_STEP - defaultFootholdDelta);
    x[0] = 0.0;
}

double  qrFootStepper ::CheckSolution(Eigen::Matrix<float, 1, 4> currentFootholdsX, double front, double back, qrGap frontGap, qrGap backGap) {
    for (int i = 1; i < 5; ++i) {
        if (i <= 2) {
            CI[0][i] = front;
            b[i] = -CI[0][i] * (defaultFootholdDelta - (frontGap.distance + frontGap.width / 2.0 * CI[0][i])
                                + currentFootholdsX[i - 1]);
        }
        else{
            CI[0][i] = back;
            b[i] = -CI[0][i] * (defaultFootholdDelta - (backGap.distance + backGap.width / 2.0 * CI[0][i])
                                + currentFootholdsX[i - 1]);
        }

    }
    try{
        quadprogpp::solve_quadprog(G, a, CE, e, CI, -b, x);
    }
    catch (std::overflow_error) {
        std::cout << "overflow_error when compute foothold" << std::endl;
        return MAXIMUM_STEP;
    }

    double deltaX = x[0];
    // whether satisfy: Ax + b >= 0
    for (int i = 0; i < CI.ncols(); i++) {
        if ((int)(deltaX * CI[0][i] * 10000) < (int)(b[i]*10000)) {
            return MAXIMUM_STEP;
        }
    }
    return deltaX;
}


int qrFootStepper ::StepGenerator(Eigen::Matrix<float, 1, 4>& currentFootholdsX, Eigen::Matrix<float, 1, 4>& desiredFootholdsOffset) {
    Eigen::Matrix<float, 1, 4> defaultNextFootholdsX = currentFootholdsX.array() + defaultFootholdDelta;
    desiredFootholdsOffset << defaultFootholdDelta, defaultFootholdDelta, defaultFootholdDelta, defaultFootholdDelta;
    
    for (int gapIndex = 0; gapIndex < gaps.size(); gapIndex++){
        qrGap &gap = gaps[gapIndex];
        qrGap frontGap = gap;
        qrGap backGap = gap;
        if (gapIndex > 0) {
            backGap = gaps[gapIndex-1];
        }
        if (gapIndex < gaps.size() - 1) {
            frontGap = gaps[gapIndex+1];
        }
        float deltaX = MAXIMUM_STEP;
        for (int legId = 0; legId < 4; ++legId) {
            float defaultNextFootholdX = defaultNextFootholdsX[legId];
            if (std::abs(defaultNextFootholdX - gap.distance) <= gap.width / 2) {
                printf("meet gap!, gap = %f, legId = %d \n", gap.distance, legId);
                std::cout << "distance between defaultNextFootholdX and gap: "
                        << gap.distance - defaultNextFootholdX
                        << std::endl;
                float stepDeltaX = 0.f;
                if (legId <= 1) { // front leg meet the gap
                    frontGap = gap;
                } 
                else { // back leg meet gap
                    backGap = gap;
                }
                for (int i = -1; i <= 1; i += 2) {
                    for (int j = -1; j <= 1; j+= 2) {
                        float x = CheckSolution(currentFootholdsX, i, j, frontGap, backGap);
                        deltaX = abs(x) < abs(deltaX) ? x: deltaX;                               
                    }
                }
                stepDeltaX = defaultFootholdDelta + deltaX;
                desiredFootholdsOffset = {stepDeltaX, stepDeltaX, stepDeltaX, stepDeltaX};
                if (stepDeltaX < 0.001 || stepDeltaX >= MAXIMUM_STEP) {
                    if (gaitFlag) {
                        return -2;
                    }
                    gaitFlag = true;
                    return -1;
                }                                      
                return 0;
            }
        }
    }
    // recovery of cross gait.
    if (gaitFlag) {
        for (qrGap &gap: gaps) {
            if (std::abs(currentFootholdsX[0] + defaultFootholdDelta / 2.0 - gap.distance) <= gap.width / 2 ||
                std::abs(currentFootholdsX[3] + defaultFootholdDelta / 2.0 - gap.distance) <= gap.width / 2) {
                return 0;
            }
        }
        desiredFootholdsOffset << defaultFootholdDelta / 2.0, defaultFootholdDelta, defaultFootholdDelta, defaultFootholdDelta / 2.0;
        gaitFlag = false;
    }
    return 0;
}

std::tuple<Eigen::Matrix<float,3,4>, Eigen::Matrix<float,3,4>> qrFootStepper ::GetFootholdsInWorldFrame(
                                                                Eigen::Matrix<float, 3, 4>& currentFootholds,
                                                                Eigen::Matrix<float, 6, 1>& currentComPose,
                                                                Eigen::Matrix<float, 6, 1>& desiredComPose,
                                                                std::vector<int>& legIds)
{   
    std::cout << "currentFootholds" << currentFootholds << std::endl;
    Eigen::Matrix<float, 3, 4> nextFootholds = currentFootholds;
    Eigen::Matrix<float, 3, 1> constOffset = {0.1f, 0.f, 0.f};
    dZ << 0.f, 0.f, 0.f, 0.f; 
    
    // for sim a1
    std::vector<Vec2<float>> stairFrames;
    int fourFootOnWhichStairK[4] = {-1, -1, -1, -1};
    
    if (currentFootholds(0,3) < 2.0) { // up
        for (int i=0;i < stairUp.k; i++) {
            Vec2<float> stairFramePoint = {stairUp.startPoint[0] + i * stairUp.width, 0.f + i*stairUp.height};
            stairFrames.push_back(stairFramePoint);
            for(int legId=0; legId<NumLeg;++legId) {
                if (currentFootholds(0, legId) >= stairFramePoint[0]) {
                    fourFootOnWhichStairK[legId]++;
                }
            }
        }
        int minBackFootK = std::min(fourFootOnWhichStairK[2], fourFootOnWhichStairK[3]);
        int maxBackFootK = std::max(fourFootOnWhichStairK[2], fourFootOnWhichStairK[3]);
        int maxFrontFootK = std::max(fourFootOnWhichStairK[1], fourFootOnWhichStairK[0]);
        int minFrontFootK = std::min(fourFootOnWhichStairK[1], fourFootOnWhichStairK[0]);
        printf("[up] fourFootOnWhichStairK = %d, %d, %d, %d\n", fourFootOnWhichStairK[0],fourFootOnWhichStairK[1],fourFootOnWhichStairK[2],fourFootOnWhichStairK[3]);
        for (int legId : legIds) {
            Vec3<float> nextFootPos = nextFootholds.col(legId) + constOffset;
            int footOnWhichStairK = fourFootOnWhichStairK[legId];
            float tmp = 0;
            if (footOnWhichStairK<stairUp.k-1) {
                tmp = stairUp.width * (footOnWhichStairK+1);
            } else {
                nextFootholds.col(legId) = nextFootPos;
                continue;
            }
            printf("foot stepper leg [%d]: tmp = %f\n", legId, tmp);
            if (nextFootPos[0] > stairUp.startPoint[0] - 0.1 + tmp && nextFootPos[0] < stairUp.startPoint[0] - 0.05 + tmp) {
                nextFootPos[0] = stairUp.startPoint[0] - 0.08 + tmp;
            } else if (nextFootPos[0] >= stairUp.startPoint[0] - 0.05 + tmp &&nextFootPos[0] < stairUp.startPoint[0] + 0.02 + tmp) {
                nextFootPos[0] = stairUp.startPoint[0] - 0.05 + tmp;
            } else if (nextFootPos[0] >= stairUp.startPoint[0] + 0.02 + tmp && nextFootPos[0] < stairUp.startPoint[0] + 0.07 + tmp) {
                if (legId<=1) {
                    int legId1 = (legId + 1)%2; // robot->GetSameFrontBackLegId(legId);
                    int legId2 = legId +2;// robot->GetSideLegId(legId);
                    if (footOnWhichStairK <= fourFootOnWhichStairK[legId1] && 
                            footOnWhichStairK<=maxBackFootK+1) {
                        nextFootPos[0] = stairUp.startPoint[0] + 0.05 + tmp;
                        nextFootPos[2] += stairUp.height;
                        dZ[legId] = stairUp.height;
                    } else {
                        nextFootPos[0] = stairUp.startPoint[0] - 0.04 + tmp;
                    }
                    
                } else {
                    int legId1 = 2 + (legId - 2 + 1)%2; // robot->GetSameFrontBackLegId(legId);
                    int legId2 = legId - 2; // robot->GetSideLegId(legId);
                    if (footOnWhichStairK <= fourFootOnWhichStairK[legId1] && 
                            footOnWhichStairK<minFrontFootK) {
                        nextFootPos[0] = stairUp.startPoint[0] + 0.05 + tmp;
                        nextFootPos[2] += stairUp.height;
                        dZ[legId] = stairUp.height;
                    } else {
                        nextFootPos[0] = stairUp.startPoint[0] - 0.04 + tmp;
                    }
                }
                
            }
            nextFootholds.col(legId) = nextFootPos;
        }
        
    } else {
        for (int legId=0; legId< NumLeg; ++legId) {
            fourFootOnWhichStairK[legId] = 0; // ground is 3=k
        }
        for (int i=0;i < stairDown.k; i++) {
            Vec2<float> stairFramePoint = {stairDown.startPoint[0] + i * stairUp.width, stairDown.startPoint[0] - i*stairUp.height};
            stairFrames.push_back(stairFramePoint);
            for(int legId=0; legId<NumLeg;++legId) {
                if (currentFootholds(0, legId) >= stairFramePoint[0]) {
                    fourFootOnWhichStairK[legId]++;
                }
            }
        }
        int minBackFootK = std::min(fourFootOnWhichStairK[2], fourFootOnWhichStairK[3]);
        int maxBackFootK = std::max(fourFootOnWhichStairK[2], fourFootOnWhichStairK[3]);
        int maxFrontFootK = std::max(fourFootOnWhichStairK[1], fourFootOnWhichStairK[0]);
        int minFrontFootK = std::min(fourFootOnWhichStairK[1], fourFootOnWhichStairK[0]);
        printf("[down] fourFootOnWhichStairK = %d, %d, %d, %d\n", fourFootOnWhichStairK[0],fourFootOnWhichStairK[1],fourFootOnWhichStairK[2],fourFootOnWhichStairK[3]);
        for (int legId : legIds) {
            Vec3<float> nextFootPos = nextFootholds.col(legId) + constOffset;
            int footOnWhichStairK = fourFootOnWhichStairK[legId];
            float tmp = 0;
            if (footOnWhichStairK<stairDown.k) {
                tmp = stairDown.width * footOnWhichStairK;
            } else {
                nextFootholds.col(legId) = nextFootPos;
                continue;
            }
            printf("foot stepper leg [%d]: tmp = %f\n", legId, tmp);
            if (nextFootPos[0] > stairDown.startPoint[0] - 0.1 + tmp && nextFootPos[0] < stairDown.startPoint[0] - 0.05 + tmp) {
                nextFootPos[0] = stairDown.startPoint[0] - 0.09 + tmp;
            } else if (nextFootPos[0] >= stairDown.startPoint[0] - 0.05 + tmp && nextFootPos[0] < stairDown.startPoint[0] + 0.02 + tmp) {
                nextFootPos[0] = stairDown.startPoint[0] - 0.03 + tmp;
            } else if (nextFootPos[0] >= stairDown.startPoint[0] + 0.02 + tmp && nextFootPos[0] < stairDown.startPoint[0] + 0.1 + tmp) {
                if (legId<=1) {
                    int legId1 = (legId + 1)%2; // robot->GetSameFrontBackLegId(legId);
                    int legId2 = legId +2;// robot->GetSideLegId(legId);
                    if (footOnWhichStairK <= fourFootOnWhichStairK[legId1] && 
                            footOnWhichStairK<=maxBackFootK) {
                        nextFootPos[0] = stairDown.startPoint[0] + 0.09 + tmp;
                        nextFootPos[2] -= stairDown.height;
                        dZ[legId] = -stairDown.height;
                    } else {
                        nextFootPos[0] = stairDown.startPoint[0] - 0.03 + tmp;
                    }
                    
                } else {
                    int legId1 = 2 + (legId - 2 + 1)%2; // robot->GetSameFrontBackLegId(legId);
                    int legId2 = legId - 2; // robot->GetSideLegId(legId);
                    if (footOnWhichStairK <= fourFootOnWhichStairK[legId1] && 
                            footOnWhichStairK<minFrontFootK) {
                        nextFootPos[0] = stairDown.startPoint[0] + 0.09 + tmp;
                        nextFootPos[2] -= stairDown.height;
                        dZ[legId] = -stairDown.height;
                    } else {
                        nextFootPos[0] = stairDown.startPoint[0] - 0.03 + tmp;
                    }
                }
                
            }
            nextFootholds.col(legId) = nextFootPos;
        }
    }
    
    nextFootholdsOffset.row(2) = dZ;
    return {nextFootholds, nextFootholdsOffset};
}

Eigen::Matrix<float, 3, 4> qrFootStepper ::GetOptimalFootholdsOffset(Eigen::Matrix<float, 3, 4> currentFootholds)
{
    Eigen::Matrix<float, 1, 4> currentFootholdsX = currentFootholds.row(0);
    nextFootholdsOffset.row(0) << defaultFootholdDelta, defaultFootholdDelta, defaultFootholdDelta, defaultFootholdDelta;
    if (gaps.empty()) {
        return nextFootholdsOffset;
    }

    // generate steps
    if (!generatorFlag) {
        // clear element
        while (!steps.empty()) {
            steps.pop();
        }  
        qrGap lastGap = gaps.back();
        Eigen::Matrix<float, 1, 4> desiredFootholdsOffset;
        while (currentFootholdsX.row(0)[3] < lastGap.distance + lastGap.width / 2.0 ) {        
            int flag = StepGenerator(currentFootholdsX, desiredFootholdsOffset);
            if (flag == -2) {
                std::cout << "[ERROR]: no valid solution." << std::endl;
                exit(-1);                    
            }
            if (flag == -1) {
                steps.back()[0] += defaultFootholdDelta / 2.0;
                steps.back()[3] += defaultFootholdDelta / 2.0;
                currentFootholdsX.row(0)[0] += defaultFootholdDelta / 2.0;
                currentFootholdsX.row(0)[3] += defaultFootholdDelta / 2.0;                
            }
            else {
                steps.push(desiredFootholdsOffset);
                currentFootholdsX.row(0) += desiredFootholdsOffset;
            }              
        }

        generatorFlag = true;
    }

    if (!steps.empty()) {
            nextFootholdsOffset.row(0) = steps.front();
            steps.pop();
    }
    return nextFootholdsOffset;
}
