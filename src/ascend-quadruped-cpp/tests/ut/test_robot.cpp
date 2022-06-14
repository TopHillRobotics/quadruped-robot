
//
// Created by Zhao Yao on 2021/10/29.
//
#include "gtest/gtest.h"
#include "robots/a1_robot.h"

using namespace std;

<<<<<<< HEAD
=======

>>>>>>> origin/develop
class TestRobot : public testing::Test {
public:
    TestRobot();

    ~TestRobot() = default;

<<<<<<< HEAD
    void SetUp()
    {
//        std::cout << "-------------------------start---------------------------" << std::endl;
    }

    void TearDown()
    {
//        std::cout << "--------------------------end--------------------------" << std::endl;
    }

    static void SetUpTestCase()
    {
=======
    void SetUp() {
//        std::cout << "-------------------------start---------------------------" << std::endl;
    }

    void TearDown() {
//        std::cout << "--------------------------end--------------------------" << std::endl;
    }

    static void SetUpTestCase() {
>>>>>>> origin/develop
        std::cout << "--------------------------SetUpTestCase--------------------------" << std::endl;

    }

    bool IsIntEqual(int a, int b);

    bool IsMatrixEqual(Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> matrixA,
                       Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> matrixB);

    A1Robot *robot;

};

<<<<<<< HEAD
TestRobot::TestRobot()
{
=======
TestRobot::TestRobot() {
>>>>>>> origin/develop
    robot = new A1Robot("../config/a1_robot.yaml");
//    std::cout << "*************************Init Test robot class*************************" << std::endl;
}

<<<<<<< HEAD
bool TestRobot::IsIntEqual(int a, int b)
{
=======
bool TestRobot::IsIntEqual(int a, int b) {
>>>>>>> origin/develop
    if (a == b) {
        return true;
    } else {
        return false;
    }
}

bool TestRobot::IsMatrixEqual(Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> matrixA,
<<<<<<< HEAD
                              Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> matrixB)
{
=======
                              Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> matrixB) {
>>>>>>> origin/develop
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> error;
//    float error = (matrixA - matrixB).squaredNorm();
    return true;
}

<<<<<<< HEAD
TEST_F(TestRobot, Test1_ReceiveObservationTest
)
{
    std::cout << "--------------------------Test1--------------------------" <<
              std::endl;

// case 1 input
    LowState input1;
    input1.imu.
        quaternion = {0};
    input1.imu.
        rpy = {0};
    input1.imu.
        gyroscope = {0};
    input1.
        footForce = {0};
    for (
        int motorId = 0;
        motorId < 12; motorId++) {
        input1.motorState[motorId].
            q = 0;
        input1.motorState[motorId].
            dq = 0;
    }

// case 1 output
=======

TEST_F(TestRobot, Test1_ReceiveObservationTest) {
    std::cout << "--------------------------Test1--------------------------" << std::endl;

    // case 1 input
    LowState input1;
    input1.imu.quaternion = {0};
    input1.imu.rpy = {0};
    input1.imu.gyroscope = {0};
    input1.footForce = {0};
    for (int motorId = 0; motorId < 12; motorId++) {
        input1.motorState[motorId].q = 0;
        input1.motorState[motorId].dq = 0;
    }

    // case 1 output
>>>>>>> origin/develop
    Eigen::Matrix<float, 4, 1> baseOrientation(0, 0, 0, 0);
    Eigen::Matrix<float, 3, 1> baseRollPitchYaw(0, 0, 0);
    Eigen::Matrix<float, 3, 1> baseRollPitchYawRate(0, 0, 0);
    Eigen::Matrix<float, 4, 1> footForce(0, 0, 0, 0);
    Eigen::Matrix<bool, 4, 1> footContact(false, false, false, false);

<<<<<<< HEAD
// call function
    robot->
        ReceiveObservationTest(input1);

//check output
    EXPECT_EQ(baseOrientation, robot
        ->
            GetBaseOrientation()
    );
    EXPECT_EQ(baseRollPitchYaw, robot
        ->
            GetBaseRollPitchYaw()
    );
    EXPECT_EQ(baseRollPitchYawRate, robot
        ->
            GetBaseRollPitchYawRate()
    );
    EXPECT_EQ(footForce, robot
        ->
            GetFootForce()
    );
    EXPECT_EQ(footContact, robot
        ->
            GetFootContacts()
    );

}

TEST_F(TestRobot, Test2_ApplyActionTest
)
{
    std::cout << "--------------------------Test2--------------------------" <<
              std::endl;

// case 1 input
    Eigen::Matrix<float, 1, 12> motorCommands;
    MotorMode motorControlMode;
    motorCommands << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    motorControlMode = TORQUE_MODE;

// case 1 output
    std::array<float, 60> interfaceCommands = {0};

// call function
    std::array<float, 60> output = robot->ApplyActionTest(motorCommands, motorControlMode);

//check output
    EXPECT_EQ(interfaceCommands, output
    );

}

TEST_F(TestRobot, Test3_FootPositionInHipFrameToJointAngle
)
{
    std::cout << "--------------------------Test3--------------------------" <<
              std::endl;

// declaration
=======

    // call function
    robot->ReceiveObservationTest(input1);

    //check output
    EXPECT_EQ(baseOrientation, robot->GetBaseOrientation());
    EXPECT_EQ(baseRollPitchYaw, robot->GetBaseRollPitchYaw());
    EXPECT_EQ(baseRollPitchYawRate, robot->GetBaseRollPitchYawRate());
    EXPECT_EQ(footForce, robot->GetFootForce());
    EXPECT_EQ(footContact, robot->GetFootContacts());

}

TEST_F(TestRobot, Test2_ApplyActionTest) {
    std::cout << "--------------------------Test2--------------------------" << std::endl;

    // case 1 input
    Eigen::Matrix<float, 1, 12> motorCommands;
    ControlMode motorControlMode;
    motorCommands << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    motorControlMode = TORQUE_MODE;

    // case 1 output
    std::array<float, 60> interfaceCommands = {0};

    // call function
    std::array<float, 60> output = robot->ApplyActionTest(motorCommands, motorControlMode);

    //check output
    EXPECT_EQ(interfaceCommands, output);

}


TEST_F(TestRobot, Test3_FootPositionInHipFrameToJointAngle) {
    std::cout << "--------------------------Test3--------------------------" << std::endl;

    // declaration
>>>>>>> origin/develop
    Eigen::Matrix<float, 3, 1> footPosition;
    int hipSign;

    Eigen::Matrix<float, 3, 1> jointAngleCase;
    Eigen::Matrix<float, 3, 1> output;

<<<<<<< HEAD
// case 1 input
    hipSign = -1;
    footPosition << -0.005, -0.079, -0.145;
// case 1 output
    jointAngleCase << 0.041338067020921035, 1.2408655839745302, -2.418034767808273;
// call function
    output = robot->FootPositionInHipFrameToJointAngle(footPosition, hipSign);
//check output
    EXPECT_NEAR(jointAngleCase[0], output[0],
                0.01);
    EXPECT_NEAR(jointAngleCase[1], output[1],
                0.01);
    EXPECT_NEAR(jointAngleCase[2], output[2],
                0.01);

// case 2 input
    hipSign = 1;
    footPosition << -0.02, 0.075, -0.14;
// case 2 output
    jointAngleCase << -0.07179742768510668, 1.3719768207322331, -2.4476717009719793;
// call function
    output = robot->FootPositionInHipFrameToJointAngle(footPosition, hipSign);
//check output
    EXPECT_NEAR(jointAngleCase[0], output[0],
                0.01);
    EXPECT_NEAR(jointAngleCase[1], output[1],
                0.01);
    EXPECT_NEAR(jointAngleCase[2], output[2],
                0.01);

}

TEST_F(TestRobot, Test4_FootPositionInHipFrame
)
{
    std::cout << "--------------------------Test4--------------------------" <<
              std::endl;

// declaration
=======
    // case 1 input
    hipSign = -1;
    footPosition << -0.005, -0.079, -0.145;
    // case 1 output
    jointAngleCase << 0.041338067020921035, 1.2408655839745302, -2.418034767808273;
    // call function
    output = robot->FootPositionInHipFrameToJointAngle(footPosition, hipSign);
    //check output
    EXPECT_NEAR(jointAngleCase[0], output[0], 0.01);
    EXPECT_NEAR(jointAngleCase[1], output[1], 0.01);
    EXPECT_NEAR(jointAngleCase[2], output[2], 0.01);


    // case 2 input
    hipSign = 1;
    footPosition << -0.02, 0.075, -0.14;
    // case 2 output
    jointAngleCase << -0.07179742768510668, 1.3719768207322331, -2.4476717009719793;
    // call function
    output = robot->FootPositionInHipFrameToJointAngle(footPosition, hipSign);
    //check output
    EXPECT_NEAR(jointAngleCase[0], output[0], 0.01);
    EXPECT_NEAR(jointAngleCase[1], output[1], 0.01);
    EXPECT_NEAR(jointAngleCase[2], output[2], 0.01);

}

TEST_F(TestRobot, Test4_FootPositionInHipFrame) {
    std::cout << "--------------------------Test4--------------------------" << std::endl;

    // declaration
>>>>>>> origin/develop
    Eigen::Matrix<float, 3, 1> angles;
    int hipSign;

    Eigen::Matrix<float, 3, 1> footPositionInHipFrame;
    Eigen::Matrix<float, 3, 1> output;

<<<<<<< HEAD
// case 1 input
    hipSign = 1;
    angles << -0.006, 0.992, -1.75;
// case 1 output
    footPositionInHipFrame << -0.02991056450383641, 0.08361694149540815, -0.25515339652922225;
// call function
    output = robot->FootPositionInHipFrame(angles, hipSign);
//check output: case, output
    EXPECT_NEAR(footPositionInHipFrame[0], output[0],
                0.001);
    EXPECT_NEAR(footPositionInHipFrame[1], output[1],
                0.001);
    EXPECT_NEAR(footPositionInHipFrame[2], output[2],
                0.001);

// case 2 input
    hipSign = -1;
    angles << 0.002, 1.036, -1.845;
// case 2 output
    footPositionInHipFrame << -0.027312278279553827, -0.08468287973331089, -0.24003763532384179;
// call function
    output = robot->FootPositionInHipFrame(angles, hipSign);
//check output: case, output
    EXPECT_NEAR(footPositionInHipFrame[0], output[0],
                0.001);
    EXPECT_NEAR(footPositionInHipFrame[1], output[1],
                0.001);
    EXPECT_NEAR(footPositionInHipFrame[2], output[2],
                0.001);

}

TEST_F(TestRobot, Test5_AnalyticalLegJacobian
)
{
    std::cout << "--------------------------Test5--------------------------" <<
              std::endl;

// declaration
=======
    // case 1 input
    hipSign = 1;
    angles << -0.006, 0.992, -1.75;
    // case 1 output
    footPositionInHipFrame << -0.02991056450383641, 0.08361694149540815, -0.25515339652922225;
    // call function
    output = robot->FootPositionInHipFrame(angles, hipSign);
    //check output: case, output
    EXPECT_NEAR(footPositionInHipFrame[0], output[0], 0.001);
    EXPECT_NEAR(footPositionInHipFrame[1], output[1], 0.001);
    EXPECT_NEAR(footPositionInHipFrame[2], output[2], 0.001);


    // case 2 input
    hipSign = -1;
    angles << 0.002, 1.036, -1.845;
    // case 2 output
    footPositionInHipFrame << -0.027312278279553827, -0.08468287973331089, -0.24003763532384179;
    // call function
    output = robot->FootPositionInHipFrame(angles, hipSign);
    //check output: case, output
    EXPECT_NEAR(footPositionInHipFrame[0], output[0], 0.001);
    EXPECT_NEAR(footPositionInHipFrame[1], output[1], 0.001);
    EXPECT_NEAR(footPositionInHipFrame[2], output[2], 0.001);

}

TEST_F(TestRobot, Test5_AnalyticalLegJacobian) {
    std::cout << "--------------------------Test5--------------------------" << std::endl;

    // declaration
>>>>>>> origin/develop
    Eigen::Matrix<float, 3, 1> legAngles;
    int legId;

    Eigen::Matrix<float, 3, 3> J;
    Eigen::Matrix<float, 3, 3> output;

<<<<<<< HEAD
// case 1 input
    legId = 1;
    legAngles << -0.009, 0.991, -1.752;
// case 1 output
    J << 0., -0.254, -0.145, 0.255, 0., -0.001, 0.083, 0.029, -0.138;
// call function
    output = robot->AnalyticalLegJacobian(legAngles, legId);
//check output: case, output
//    cout << "J: \n" << J << endl;
//    cout << "output: \n" << output << endl;
    EXPECT_NEAR(J(0, 0), output(0, 0),
                0.001);
    EXPECT_NEAR(J(1, 0), output(1, 0),
                0.001);
    EXPECT_NEAR(J(2, 0), output(2, 0),
                0.001);

// case 2 input
    legId = 0;
    legAngles << 0.011, 1.216, -2.272;
// case 2 output
    J << 0., -0.168, -0.099, 0.169, -0., 0.002, -0.083, 0.014, -0.174;
// call function
    output = robot->AnalyticalLegJacobian(legAngles, legId);
//check output: case, output
//    cout << "J: \n" << J << endl;
//    cout << "output: \n" << output << endl;
    EXPECT_NEAR(J(0, 0), output(0, 0),
                0.001);
    EXPECT_NEAR(J(1, 0), output(1, 0),
                0.001);
    EXPECT_NEAR(J(2, 0), output(2, 0),
                0.001);
}

TEST_F(TestRobot, Test6_FootPositionsInBaseFrame
)
{
    std::cout << "--------------------------Test6--------------------------" <<
              std::endl;

// declaration
=======
    // case 1 input
    legId = 1;
    legAngles << -0.009, 0.991, -1.752;
    // case 1 output
    J << 0., -0.254, -0.145, 0.255, 0., -0.001, 0.083, 0.029, -0.138;
    // call function
    output = robot->AnalyticalLegJacobian(legAngles, legId);
    //check output: case, output
//    cout << "J: \n" << J << endl;
//    cout << "output: \n" << output << endl;
    EXPECT_NEAR(J(0, 0), output(0, 0), 0.001);
    EXPECT_NEAR(J(1, 0), output(1, 0), 0.001);
    EXPECT_NEAR(J(2, 0), output(2, 0), 0.001);

    // case 2 input
    legId = 0;
    legAngles << 0.011, 1.216, -2.272;
    // case 2 output
    J << 0., -0.168, -0.099, 0.169, -0., 0.002, -0.083, 0.014, -0.174;
    // call function
    output = robot->AnalyticalLegJacobian(legAngles, legId);
    //check output: case, output
//    cout << "J: \n" << J << endl;
//    cout << "output: \n" << output << endl;
    EXPECT_NEAR(J(0, 0), output(0, 0), 0.001);
    EXPECT_NEAR(J(1, 0), output(1, 0), 0.001);
    EXPECT_NEAR(J(2, 0), output(2, 0), 0.001);
}

TEST_F(TestRobot, Test6_FootPositionsInBaseFrame) {
    std::cout << "--------------------------Test6--------------------------" << std::endl;

    // declaration
>>>>>>> origin/develop
    Eigen::Matrix<float, 12, 1> footAngles;

    Eigen::Matrix<float, 3, 4> footsPositionsInBaseFrame;
    Eigen::Matrix<float, 3, 4> output;

<<<<<<< HEAD
// case 1 input
    footAngles << -0.006, 1.192, -2.263, -0.01, 0.988, -1.756, 0.013, 1.042, -1.842, -0.026, 1.252, -2.272;
// case 1 output
=======
    // case 1 input
    footAngles << -0.006, 1.192, -2.263, -0.01, 0.988, -1.756, 0.013, 1.042, -1.842, -0.026, 1.252, -2.272;
    // case 1 output
>>>>>>> origin/develop
    footsPositionsInBaseFrame.col(0) << 0.165, -0.136, -0.17;
    footsPositionsInBaseFrame.col(1) << 0.147, 0.127, -0.255;
    footsPositionsInBaseFrame.col(2) << -0.215, -0.132, -0.242;
    footsPositionsInBaseFrame.col(3) << -0.205, 0.125, -0.17;
<<<<<<< HEAD
// call function
    output = robot->FootPositionsInBaseFrame(footAngles);
//check output: case, output
//    cout << "footsPositionsInBaseFrame: \n" << footsPositionsInBaseFrame << endl;
//    cout << "output: \n" << output << endl;
    EXPECT_NEAR(footsPositionsInBaseFrame(0, 0), output(0, 0),
                0.001);
    EXPECT_NEAR(footsPositionsInBaseFrame(1, 0), output(1, 0),
                0.001);
    EXPECT_NEAR(footsPositionsInBaseFrame(2, 0), output(2, 0),
                0.001);

// case 2 input
    footAngles << 0.007, 0.995, -1.776, -0.036, 1.222, -2.244, 0.011, 1.241, -2.258, -0.028, 1.04, -1.825;
// case 2 output
=======
    // call function
    output = robot->FootPositionsInBaseFrame(footAngles);
    //check output: case, output
//    cout << "footsPositionsInBaseFrame: \n" << footsPositionsInBaseFrame << endl;
//    cout << "output: \n" << output << endl;
    EXPECT_NEAR(footsPositionsInBaseFrame(0, 0), output(0, 0), 0.001);
    EXPECT_NEAR(footsPositionsInBaseFrame(1, 0), output(1, 0), 0.001);
    EXPECT_NEAR(footsPositionsInBaseFrame(2, 0), output(2, 0), 0.001);

    // case 2 input
    footAngles << 0.007, 0.995, -1.776, -0.036, 1.222, -2.244, 0.011, 1.241, -2.258, -0.028, 1.04, -1.825;
    // case 2 output
>>>>>>> origin/develop
    footsPositionsInBaseFrame.col(0) << 0.149, -0.133, -0.252;
    footsPositionsInBaseFrame.col(1) << 0.158, 0.123, -0.176;
    footsPositionsInBaseFrame.col(2) << -0.205, -0.133, -0.171;
    footsPositionsInBaseFrame.col(3) << -0.217, 0.123, -0.245;
<<<<<<< HEAD
// call function
    output = robot->FootPositionsInBaseFrame(footAngles);
//check output: case, output
//    cout << "footsPositionsInBaseFrame: \n" << footsPositionsInBaseFrame << endl;
//    cout << "output: \n" << output << endl;
    EXPECT_NEAR(footsPositionsInBaseFrame(0, 0), output(0, 0),
                0.001);
    EXPECT_NEAR(footsPositionsInBaseFrame(1, 0), output(1, 0),
                0.001);
    EXPECT_NEAR(footsPositionsInBaseFrame(2, 0), output(2, 0),
                0.001);

}

TEST_F(TestRobot, Test7_ComputeMotorAnglesFromFootLocalPosition
)
{
    std::cout << "--------------------------Test7--------------------------" <<
              std::endl;
=======
    // call function
    output = robot->FootPositionsInBaseFrame(footAngles);
    //check output: case, output
//    cout << "footsPositionsInBaseFrame: \n" << footsPositionsInBaseFrame << endl;
//    cout << "output: \n" << output << endl;
    EXPECT_NEAR(footsPositionsInBaseFrame(0, 0), output(0, 0), 0.001);
    EXPECT_NEAR(footsPositionsInBaseFrame(1, 0), output(1, 0), 0.001);
    EXPECT_NEAR(footsPositionsInBaseFrame(2, 0), output(2, 0), 0.001);

}

TEST_F(TestRobot, Test7_ComputeMotorAnglesFromFootLocalPosition) {
    std::cout << "--------------------------Test7--------------------------" << std::endl;
>>>>>>> origin/develop

    int legId;
    Eigen::Matrix<float, 3, 1> footLocalPosition;

    Eigen::Matrix<int, 3, 1> jointIdx;
    Eigen::Matrix<float, 3, 1> jointAngles;

    Eigen::Matrix<int, 3, 1> jointIdxOutput;
    Eigen::Matrix<float, 3, 1> jointAnglesOutput;

<<<<<<< HEAD
// case 1 input
    legId = 2;
    footLocalPosition << -0.19708761901155575, -0.13657511076965906, -0.22360062811347164;
// case 1 output
    jointIdx << 6, 7, 8;
    jointAngles << -0.007721434694941235, 1.0280053571146408, -1.9525262189901185;
// call function
    robot->
        ComputeMotorAnglesFromFootLocalPosition(legId, footLocalPosition,
                                                jointIdxOutput, jointAnglesOutput
    );
//check output: case, output
=======
    // case 1 input
    legId = 2;
    footLocalPosition << -0.19708761901155575, -0.13657511076965906, -0.22360062811347164;
    // case 1 output
    jointIdx << 6, 7, 8;
    jointAngles << -0.007721434694941235, 1.0280053571146408, -1.9525262189901185;
    // call function
    robot->ComputeMotorAnglesFromFootLocalPosition(legId, footLocalPosition,
                                                   jointIdxOutput, jointAnglesOutput);
    //check output: case, output
>>>>>>> origin/develop
//    cout << "jointIdx: \n" << jointIdx << endl;
//    cout << "jointIdxOutput: \n" << jointIdxOutput << endl;
//    cout << "jointAngles: \n" << jointAngles << endl;
//    cout << "jointAnglesOutput: \n" << jointAnglesOutput << endl;
<<<<<<< HEAD
    EXPECT_EQ(jointIdx, jointIdxOutput
    );
    EXPECT_NEAR(jointAngles[0], jointAnglesOutput[0],
                0.001);
    EXPECT_NEAR(jointAngles[1], jointAnglesOutput[1],
                0.001);
    EXPECT_NEAR(jointAngles[2], jointAnglesOutput[2],
                0.001);

// case 2 input
    legId = 1;
    footLocalPosition << 0.1632267773313873, 0.12967425769950416, -0.1699995392831369;
// case 2 output
    jointIdx << 3, 4, 5;
    jointAngles << 0.0025016481404202735, 1.203643033783095, -2.2628895271684817;
// call function
    robot->
        ComputeMotorAnglesFromFootLocalPosition(legId, footLocalPosition,
                                                jointIdxOutput, jointAnglesOutput
    );
//check output: case, output
=======
    EXPECT_EQ(jointIdx, jointIdxOutput);
    EXPECT_NEAR(jointAngles[0], jointAnglesOutput[0], 0.001);
    EXPECT_NEAR(jointAngles[1], jointAnglesOutput[1], 0.001);
    EXPECT_NEAR(jointAngles[2], jointAnglesOutput[2], 0.001);


    // case 2 input
    legId = 1;
    footLocalPosition << 0.1632267773313873, 0.12967425769950416, -0.1699995392831369;
    // case 2 output
    jointIdx << 3, 4, 5;
    jointAngles << 0.0025016481404202735, 1.203643033783095, -2.2628895271684817;
    // call function
    robot->ComputeMotorAnglesFromFootLocalPosition(legId, footLocalPosition,
                                                   jointIdxOutput, jointAnglesOutput);
    //check output: case, output
>>>>>>> origin/develop
//    cout << "jointIdx: \n" << jointIdx << endl;
//    cout << "jointIdxOutput: \n" << jointIdxOutput << endl;
//    cout << "jointAngles: \n" << jointAngles << endl;
//    cout << "jointAnglesOutput: \n" << jointAnglesOutput << endl;
<<<<<<< HEAD
    EXPECT_EQ(jointIdx, jointIdxOutput
    );
    EXPECT_NEAR(jointAngles[0], jointAnglesOutput[0],
                0.001);
    EXPECT_NEAR(jointAngles[1], jointAnglesOutput[1],
                0.001);
    EXPECT_NEAR(jointAngles[2], jointAnglesOutput[2],
                0.001);

}

TEST_F(TestRobot, Test8_GetFootPositionsInBaseFrame
)
{
    std::cout << "--------------------------Test8--------------------------" <<
              std::endl;
=======
    EXPECT_EQ(jointIdx, jointIdxOutput);
    EXPECT_NEAR(jointAngles[0], jointAnglesOutput[0], 0.001);
    EXPECT_NEAR(jointAngles[1], jointAnglesOutput[1], 0.001);
    EXPECT_NEAR(jointAngles[2], jointAnglesOutput[2], 0.001);

}

TEST_F(TestRobot, Test8_GetFootPositionsInBaseFrame) {
    std::cout << "--------------------------Test8--------------------------" << std::endl;
>>>>>>> origin/develop

    Eigen::Matrix<float, 3, 4> footPositionsInBaseFrame;
    Eigen::Matrix<float, 3, 4> output;

<<<<<<< HEAD
// case 1 input
    robot->motorAngles << 0.006, 0.986, -1.785, 0.014, 1.215, -2.264, -0.018, 1.232, -2.26, -0.037, 1.041, -1.83;
// case 1 output
=======
    // case 1 input
    robot->motorAngles << 0.006, 0.986, -1.785, 0.014, 1.215, -2.264, -0.018, 1.232, -2.26, -0.037, 1.041, -1.83;
    // case 1 output
>>>>>>> origin/develop
    footPositionsInBaseFrame.col(0) << 0.152, -0.133, -0.251;
    footPositionsInBaseFrame.col(1) << 0.161, 0.132, -0.169;
    footPositionsInBaseFrame.col(2) << -0.203, -0.138, -0.169;
    footPositionsInBaseFrame.col(3) << -0.216, 0.12, -0.245;
<<<<<<< HEAD
// call function
    output = robot->GetFootPositionsInBaseFrame();
//check output: case, output
//    cout << "footPositionsInBaseFrame: \n" << footPositionsInBaseFrame << endl;
//    cout << "output: \n" << output << endl;
    EXPECT_NEAR(footPositionsInBaseFrame(0, 0), output(0, 0),
                0.001);
    EXPECT_NEAR(footPositionsInBaseFrame(1, 0), output(1, 0),
                0.001);
    EXPECT_NEAR(footPositionsInBaseFrame(2, 0), output(2, 0),
                0.001);


// case 2 input
    robot->motorAngles << -0.003, 1.096, -2.089, -0.024, 0.973, -1.747, 0.007, 1.024, -1.838, -0.025, 1.136, -2.097;
// case 2 output
=======
    // call function
    output = robot->GetFootPositionsInBaseFrame();
    //check output: case, output
//    cout << "footPositionsInBaseFrame: \n" << footPositionsInBaseFrame << endl;
//    cout << "output: \n" << output << endl;
    EXPECT_NEAR(footPositionsInBaseFrame(0, 0), output(0, 0), 0.001);
    EXPECT_NEAR(footPositionsInBaseFrame(1, 0), output(1, 0), 0.001);
    EXPECT_NEAR(footPositionsInBaseFrame(2, 0), output(2, 0), 0.001);


    // case 2 input
    robot->motorAngles << -0.003, 1.096, -2.089, -0.024, 0.973, -1.747, 0.007, 1.024, -1.838, -0.025, 1.136, -2.097;
    // case 2 output
>>>>>>> origin/develop
    footPositionsInBaseFrame.col(0) << 0.165, -0.135, -0.201;
    footPositionsInBaseFrame.col(1) << 0.15, 0.123, -0.258;
    footPositionsInBaseFrame.col(2) << -0.211, -0.133, -0.242;
    footPositionsInBaseFrame.col(3) << -0.203, 0.124, -0.201;
<<<<<<< HEAD
// call function
    output = robot->GetFootPositionsInBaseFrame();
//check output: case, output
//    cout << "footPositionsInBaseFrame: \n" << footPositionsInBaseFrame << endl;
//    cout << "output: \n" << output << endl;
    EXPECT_NEAR(footPositionsInBaseFrame(0, 0), output(0, 0),
                0.001);
    EXPECT_NEAR(footPositionsInBaseFrame(1, 0), output(1, 0),
                0.001);
    EXPECT_NEAR(footPositionsInBaseFrame(2, 0), output(2, 0),
                0.001);
}

TEST_F(TestRobot, Test9_ComputeJacobian
)
{
    std::cout << "--------------------------Test9--------------------------" <<
              std::endl;
=======
    // call function
    output = robot->GetFootPositionsInBaseFrame();
    //check output: case, output
//    cout << "footPositionsInBaseFrame: \n" << footPositionsInBaseFrame << endl;
//    cout << "output: \n" << output << endl;
    EXPECT_NEAR(footPositionsInBaseFrame(0, 0), output(0, 0), 0.001);
    EXPECT_NEAR(footPositionsInBaseFrame(1, 0), output(1, 0), 0.001);
    EXPECT_NEAR(footPositionsInBaseFrame(2, 0), output(2, 0), 0.001);
}

TEST_F(TestRobot, Test9_ComputeJacobian) {
    std::cout << "--------------------------Test9--------------------------" << std::endl;
>>>>>>> origin/develop

    int legId;

    Eigen::Matrix<float, 3, 3> J;
    Eigen::Matrix<float, 3, 3> output;

<<<<<<< HEAD
// case 1 input
    legId = 1;
    robot->motorAngles << 0.006, 0.985, -1.786, 0.008, 1.176, -2.209, -0.009, 1.192, -2.204, -0.033, 1.03, -1.819;
// case 1 output
    J << 0., -0.179, -0.102, 0.179, -0., 0.001, 0.087, 0.013, -0.172;
// call function
    output = robot->ComputeJacobian(legId);
//check output: case, output
//    cout << "J: \n" << J << endl;
//    cout << "output: \n" << output << endl;
    EXPECT_NEAR(J(0, 0), output(0, 0),
                0.001);
    EXPECT_NEAR(J(1, 0), output(1, 0),
                0.001);
    EXPECT_NEAR(J(2, 0), output(2, 0),
                0.001);

// case 2 input
    legId = 2;
    robot->motorAngles << 0.021, 0.991, -1.779, 0.003, 1.119, -2.149, 0.006, 1.13, -2.147, -0.016, 1.061, -1.871;
// case 2 output
    J << 0., -0.191, -0.105, 0.191, -0., 0.001, -0.084, 0.011, -0.17;
// call function
    output = robot->ComputeJacobian(legId);
//check output: case, output
//    cout << "J: \n" << J << endl;
//    cout << "output: \n" << output << endl;
    EXPECT_NEAR(J(0, 0), output(0, 0),
                0.001);
    EXPECT_NEAR(J(1, 0), output(1, 0),
                0.001);
    EXPECT_NEAR(J(2, 0), output(2, 0),
                0.001);
}

TEST_F(TestRobot, Test10_GetHipPositionsInBaseFrame
)
{
    std::cout << "--------------------------Test10--------------------------" <<
              std::endl;

    Eigen::Matrix<float, 3, 4> hipPositionsInBaseFrame;
// case 1 input

// case 1 output
=======
    // case 1 input
    legId = 1;
    robot->motorAngles << 0.006, 0.985, -1.786, 0.008, 1.176, -2.209, -0.009, 1.192, -2.204, -0.033, 1.03, -1.819;
    // case 1 output
    J << 0., -0.179, -0.102, 0.179, -0., 0.001, 0.087, 0.013, -0.172;
    // call function
    output = robot->ComputeJacobian(legId);
    //check output: case, output
//    cout << "J: \n" << J << endl;
//    cout << "output: \n" << output << endl;
    EXPECT_NEAR(J(0, 0), output(0, 0), 0.001);
    EXPECT_NEAR(J(1, 0), output(1, 0), 0.001);
    EXPECT_NEAR(J(2, 0), output(2, 0), 0.001);

    // case 2 input
    legId = 2;
    robot->motorAngles << 0.021, 0.991, -1.779, 0.003, 1.119, -2.149, 0.006, 1.13, -2.147, -0.016, 1.061, -1.871;
    // case 2 output
    J << 0., -0.191, -0.105, 0.191, -0., 0.001, -0.084, 0.011, -0.17;
    // call function
    output = robot->ComputeJacobian(legId);
    //check output: case, output
//    cout << "J: \n" << J << endl;
//    cout << "output: \n" << output << endl;
    EXPECT_NEAR(J(0, 0), output(0, 0), 0.001);
    EXPECT_NEAR(J(1, 0), output(1, 0), 0.001);
    EXPECT_NEAR(J(2, 0), output(2, 0), 0.001);
}

TEST_F(TestRobot, Test10_GetHipPositionsInBaseFrame) {
    std::cout << "--------------------------Test10--------------------------" << std::endl;


    Eigen::Matrix<float, 3, 4> hipPositionsInBaseFrame;
    // case 1 input

    // case 1 output
>>>>>>> origin/develop
    hipPositionsInBaseFrame.col(0) << 0.17, -0.135, 0;
    hipPositionsInBaseFrame.col(1) << 0.17, 0.13, 0;
    hipPositionsInBaseFrame.col(2) << -0.195, -0.135, 0;
    hipPositionsInBaseFrame.col(3) << -0.195, 0.13, 0;
<<<<<<< HEAD
// call function
    Eigen::Matrix<float, 3, 4> output = robot->GetHipPositionsInBaseFrame();
//check output: case, output
//    cout << "hipPositionsInBaseFrame: \n" << hipPositionsInBaseFrame << endl;
//    cout << "output: \n" << output << endl;
    EXPECT_NEAR(hipPositionsInBaseFrame(0, 0), output(0, 0),
                0.001);
    EXPECT_NEAR(hipPositionsInBaseFrame(0, 1), output(0, 1),
                0.001);
    EXPECT_NEAR(hipPositionsInBaseFrame(0, 2), output(0, 2),
                0.001);
    EXPECT_NEAR(hipPositionsInBaseFrame(0, 3), output(0, 3),
                0.001);
}

TEST_F(TestRobot, Test11_MapContactForceToJointTorques
)
{
    std::cout << "--------------------------Test11--------------------------" <<
              std::endl;
=======
    // call function
    Eigen::Matrix<float, 3, 4> output = robot->GetHipPositionsInBaseFrame();
    //check output: case, output
//    cout << "hipPositionsInBaseFrame: \n" << hipPositionsInBaseFrame << endl;
//    cout << "output: \n" << output << endl;
    EXPECT_NEAR(hipPositionsInBaseFrame(0, 0), output(0, 0), 0.001);
    EXPECT_NEAR(hipPositionsInBaseFrame(0, 1), output(0, 1), 0.001);
    EXPECT_NEAR(hipPositionsInBaseFrame(0, 2), output(0, 2), 0.001);
    EXPECT_NEAR(hipPositionsInBaseFrame(0, 3), output(0, 3), 0.001);
}

TEST_F(TestRobot, Test11_MapContactForceToJointTorques) {
    std::cout << "--------------------------Test11--------------------------" << std::endl;
>>>>>>> origin/develop

    int legId;
    Eigen::Matrix<float, 3, 1> contractForce = Eigen::Matrix<float, 3, 1>::Zero();

    std::map<int, float> jointTorques;
    std::map<int, float> output;

<<<<<<< HEAD
// case 1 input
    legId = 2;
    contractForce << -1.41, 2.006, -58.928;
    robot->motorAngles << 0.024, 1.216, -2.252, -0.013, 0.991, -1.756, 0.007, 1.045, -1.839, -0.039, 1.254, -2.26;
// case 1 output
    jointTorques[6] = 5.394501056972389;
    jointTorques[7] = -1.451587011143105;
    jointTorques[8] = 8.603084127787607;
// call function
    output = robot->MapContactForceToJointTorques(legId, contractForce);
//check output: case, output
=======
    // case 1 input
    legId = 2;
    contractForce << -1.41, 2.006, -58.928;
    robot->motorAngles << 0.024, 1.216, -2.252, -0.013, 0.991, -1.756, 0.007, 1.045, -1.839, -0.039, 1.254, -2.26;
    // case 1 output
    jointTorques[6] = 5.394501056972389;
    jointTorques[7] = -1.451587011143105;
    jointTorques[8] = 8.603084127787607;
    // call function
    output = robot->MapContactForceToJointTorques(legId, contractForce);
    //check output: case, output
>>>>>>> origin/develop
//    cout << "jointTorques[6]: \n" << jointTorques[6] << endl;
//    cout << "output[6]: \n" << output[6] << endl;
//    cout << "jointTorques[7]: \n" << jointTorques[7] << endl;
//    cout << "output[7]: \n" << output[7] << endl;
//    cout << "jointTorques[8]: \n" << jointTorques[8] << endl;
//    cout << "output[8]: \n" << output[8] << endl;
<<<<<<< HEAD
    EXPECT_NEAR(jointTorques[6], output[6],
                0.01);
    EXPECT_NEAR(jointTorques[7], output[7],
                0.01);
    EXPECT_NEAR(jointTorques[8], output[8],
                0.01);

// case 2 input
    legId = 3;
    contractForce << -6.604, 0.742, -54.739;
    robot->motorAngles << 0.02, 0.984, -1.763, -0.001, 1.157, -2.175, -0.001, 1.173, -2.166, -0.012, 1.035, -1.832;
// case 2 output
    jointTorques[9] = -4.3141293836780195;
    jointTorques[10] = 0.014697261182825994;
    jointTorques[11] = 8.751015533042274;
// call function
    output = robot->MapContactForceToJointTorques(legId, contractForce);
//check output: case, output
=======
    EXPECT_NEAR(jointTorques[6], output[6], 0.01);
    EXPECT_NEAR(jointTorques[7], output[7], 0.01);
    EXPECT_NEAR(jointTorques[8], output[8], 0.01);


    // case 2 input
    legId = 3;
    contractForce << -6.604, 0.742, -54.739;
    robot->motorAngles << 0.02, 0.984, -1.763, -0.001, 1.157, -2.175, -0.001, 1.173, -2.166, -0.012, 1.035, -1.832;
    // case 2 output
    jointTorques[9] = -4.3141293836780195;
    jointTorques[10] = 0.014697261182825994;
    jointTorques[11] = 8.751015533042274;
    // call function
    output = robot->MapContactForceToJointTorques(legId, contractForce);
    //check output: case, output
>>>>>>> origin/develop
//    cout << "jointTorques[9]: \n" << jointTorques[9] << endl;
//    cout << "output[9]: \n" << output[9] << endl;
//    cout << "jointTorques[10]: \n" << jointTorques[10] << endl;
//    cout << "output[10]: \n" << output[10] << endl;
//    cout << "jointTorques[11]: \n" << jointTorques[11] << endl;
//    cout << "output[11]: \n" << output[11] << endl;
<<<<<<< HEAD
    EXPECT_NEAR(jointTorques[9], output[9],
                0.01);
    EXPECT_NEAR(jointTorques[10], output[10],
                0.01);
    EXPECT_NEAR(jointTorques[11], output[11],
                0.01);
=======
    EXPECT_NEAR(jointTorques[9], output[9], 0.01);
    EXPECT_NEAR(jointTorques[10], output[10], 0.01);
    EXPECT_NEAR(jointTorques[11], output[11], 0.01);
>>>>>>> origin/develop
}


//TEST_F(TestRobot, example1) {
//    std::cout << "--------------------------example1--------------------------" << std::endl;
//    EXPECT_EQ(IsIntEqual(1, 1), true);
//}


<<<<<<< HEAD
int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);

=======
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);


>>>>>>> origin/develop
    return RUN_ALL_TESTS();
}
