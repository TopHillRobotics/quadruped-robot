//
// Created by xmc on 2021/11/2.
//

#include "gtest/gtest.h"
#include "mpc_controller/qr_gait_generator.h
#include "state_estimator/robot_velocity_estimator.h"
#include "mpc_controller/raibert_swing_leg_controller.h"

using namespace std;

class TestSwingLegController : public testing::Test {
public:
    TestSwingLegController();

    ~TestSwingLegController() = default;


    void SetUp() {
//        std::cout << "-------------------------start---------------------------" << std::endl;
    }

    void TearDown() {
//        std::cout << "--------------------------end--------------------------" << std::endl;
    }

    static void SetUpTestCase() {
        std::cout << "--------------------------SetUpTestCase--------------------------" << std::endl;

    }

    qrSwingLegController *raibertSwingLegController;
    Eigen::Matrix<float, 3, 1> desiredSpeed;
    float desiredTwistingSpeed;
    float desiredHeight;
    float footClearance;
};

TestSwingLegController::TestSwingLegController() {

    desiredSpeed << 0.0, 0.0, 0.0;
    desiredTwistingSpeed = 0.13568;
    desiredHeight = 0.24;
    footClearance = 0.01;
    raibertSwingLegController = new qrSwingLegController(desiredSpeed,
                                                              desiredTwistingSpeed,
                                                              desiredHeight,
                                                              footClearance);
}

TEST_F(TestSwingLegController, Test1_GenParabola) {
    std::cout << "--------------------------Test1--------------------------" << std::endl;

    // case_1 input
    float phase = 0.3485;
    float start = -0.251;
    float mid = -0.130;
    float end = -0.23;


    //case_1 output
    float z = -0.143;


    //call function
    float result = raibertSwingLegController->GenParabola(phase, start, mid, end);

    //check output
    EXPECT_NEAR(z, result, 0.001);
}

TEST_F(TestSwingLegController, Test2_GenSwingFootTrajectory) {
    std::cout << "--------------------------Test2--------------------------" << std::endl;

    // case_1 input
    float inputPhase = 0.143;
    Eigen::Matrix<float, 3, 1> startPos(0.172, 0.127, -0.251);
    Eigen::Matrix<float, 3, 1> endPod(0.164, 0.133, -0.23);


    //case_1 output
    Eigen::Matrix<float, 3, 1> xyz(0.169, 0.129, -0.143);


    //call function
    Eigen::Matrix<float, 3, 1> result = raibertSwingLegController->GenSwingFootTrajectory(inputPhase, startPos, endPod);

    //check output
    EXPECT_NEAR(xyz(0), result(0), 0.001);
    EXPECT_NEAR(xyz(1), result(1), 0.001);
    EXPECT_NEAR(xyz(2), result(2), 0.001);
}

TEST_F(TestSwingLegController, Test3_GetAction) {
    std::cout << "--------------------------Test3--------------------------" << std::endl;

    // case_1 input
    float testYawDot;
    Eigen::Matrix<float, 3, 1> testComVelocity;
    Eigen::Matrix<float, 3, 4> testHipPositions;
    Eigen::Matrix<int, 3, 1> testJointIdx;
    Eigen::Matrix<float, 3, 1> testJointAngles;
    Eigen::Matrix<int, 4, 1> testLegState;
    Eigen::Matrix<float, 4, 1> testStanceDuration;
    Eigen::Matrix<float, 4, 1> testPhase;
    Eigen::Matrix<float, 3, 4> testPhaseSwitchFootLocalPos;
    Eigen::Matrix<float, 12, 1> testKps;
    Eigen::Matrix<float, 12, 1> testKds;

    testComVelocity << -0.02, -0.024, 0;
    testYawDot = 0.167;
    testHipPositions << 0.17, 0.17, -0.195, -0.195,
            -0.135, 0.13, -0.135, 0.13,
            0, 0, 0, 0;
    testPhaseSwitchFootLocalPos << 0.15421, 0.15191, -0.21295, -0.21282,
            -0.13357, 0.12585, -0.12985, 0.12124,
            -0.25171, -0.25593, -0.24654, -0.25039;
    testJointIdx << 0, 1, 2;
    testJointAngles << -0.0007, 1.0803, -2.0927;
    testLegState << 0, 1, 1, 0;
    testStanceDuration << 0.23, 0.23, 0.23, 0.23;
    testPhase << 0.811, 0.707, 0.707, 0.811;
    testKps << 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100;
    testKds << 1, 2, 2, 1, 2, 2, 1, 2, 2, 1, 2, 2;

    //case_1 output
    Eigen::Matrix<float, 3, 1> footTargetPosition(0.17, -0.135, -0.23);
    Eigen::Matrix<float, 3, 1> footPosition(0.1687, -0.1350, -0.2006);
    map<int, Eigen::Matrix<float, 5, 1>> action;
    action[3] << -0.00065, 100, 0, 1, 0;
    action[4] << 1.08029, 100, 0, 2, 0;
    action[5] << -2.09271, 100, 0, 2, 0;

    //call function
    tuple<map<int, Eigen::Matrix<float, 5, 1>>, Eigen::Matrix<float, 3, 1>, Eigen::Matrix<float, 3, 1>> result =
            raibertSwingLegController->GetActionTest(
                    testComVelocity, testYawDot, testHipPositions, testJointIdx, testJointAngles, testLegState,
                    testStanceDuration, testPhase, testPhaseSwitchFootLocalPos, testKps, testKds);
    EXPECT_NEAR(action[0][0], get<0>(result)[0][0], 0.001);
    EXPECT_NEAR(action[0][1], get<0>(result)[0][1], 0.001);
    EXPECT_NEAR(action[0][2], get<0>(result)[0][2], 0.001);
    EXPECT_NEAR(action[0][3], get<0>(result)[0][3], 0.001);
    EXPECT_NEAR(action[0][4], get<0>(result)[0][4], 0.001);
    EXPECT_NEAR(footPosition[0], get<1>(result)[0], 0.001);
    EXPECT_NEAR(footPosition[1], get<1>(result)[1], 0.001);
    EXPECT_NEAR(footPosition[2], get<1>(result)[2], 0.001);
    EXPECT_NEAR(footTargetPosition[0], get<2>(result)[0], 0.001);
    EXPECT_NEAR(footTargetPosition[1], get<2>(result)[1], 0.001);
    EXPECT_NEAR(footTargetPosition[2], get<2>(result)[2], 0.001);

}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);


    return RUN_ALL_TESTS();
}