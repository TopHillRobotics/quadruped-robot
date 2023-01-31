#include <unistd.h>
#include "gtest/gtest.h"
#include "utils/se3.h"
#include "controllers/torque_stance_leg_controller.h"

using namespace robotics;
using namespace std;

// 定义测试类FooTest
<<<<<<< HEAD
class TestStanceLeg : public testing::Test {
=======
class TestStanceLeg: public testing::Test {
>>>>>>> origin/develop
protected:
    // Code here will be called immediately after the constructor (right before each test)
    void SetUp()
    {
        cout << "test start" << endl;
    }

    // Code here will be called immediately after each test (right before the destructor)
    void TearDown()
    {
        cout << "test end" << endl;
    }

public:
<<<<<<< HEAD
    Robot *robot;
    OpenloopGaitGenerator *gaitGenerator;
    RobotVelocityEstimator *robotVelocityEstimator;
=======
    Robot* robot;
    OpenloopGaitGenerator* gaitGenerator;
    RobotVelocityEstimator* robotVelocityEstimator;
>>>>>>> origin/develop
    Eigen::Matrix<float, 3, 1> desiredSpeed;
    float desiredTwistingSpeed;
    float desiredBodyHeight;
    int NumLeg;
    std::vector<float> frictionCoeffs;
    std::string configFilepath = "../config/stance_leg_controller.yaml";

<<<<<<< HEAD
    TorqueStanceLegController *stanceLegPtr;
    TorqueStanceLegController *build()
=======
    TorqueStanceLegController* stanceLegPtr;
    TorqueStanceLegController* build()
>>>>>>> origin/develop
    {
        return new TorqueStanceLegController(robot,
                                             gaitGenerator,
                                             robotVelocityEstimator,
                                             desiredSpeed,
                                             desiredTwistingSpeed,
                                             desiredBodyHeight,
                                             NumLeg,
                                             configFilepath,
                                             frictionCoeffs);
    }
};

<<<<<<< HEAD
TEST_F(TestStanceLeg, TestGetAction
)
{
    stanceLegPtr = build();
    std::tuple<std::map<int, MotorCommand>, Eigen::Matrix<float, 3, 4>> actionContactForce;
=======

TEST_F(TestStanceLeg, TestGetAction)
{
    stanceLegPtr = build();
    std::tuple<std::map<int, MotorCommand>,  Eigen::Matrix<float, 3, 4>> actionContactForce;
>>>>>>> origin/develop

    Eigen::Matrix<float, 4, 1> desiredLegStates;
    desiredLegStates << 1, 1, 1, 1;
    Eigen::Matrix<float, 3, 1> robotComPosition;
    robotComPosition << 0., 0., 0.237;
    Eigen::Matrix<float, 3, 1> robotComVelocity;
    robotComVelocity << -0.016557395458221436, -0.0011760747293010354, 0.0581844188272953;
    Eigen::Matrix<float, 3, 1> robotComRpy;
<<<<<<< HEAD
    robotComRpy << 0.012, -0.017, 0.;
=======
    robotComRpy << 0.012, -0.017,  0.;
>>>>>>> origin/develop
    Eigen::Matrix<float, 3, 1> robotComRpyRate;
    robotComRpyRate << 0.028, 0.248, 0.007;
    Eigen::Matrix<float, 3, 1> desiredComPosition;
    desiredComPosition << 0., 0., 0.24;
    Eigen::Matrix<float, 3, 1> desiredComVelocity;
    desiredComVelocity << 0., 0., 0.;
    Eigen::Matrix<float, 3, 1> desiredComAngularVelocity;
    desiredComAngularVelocity << 0., 0., 0.094;
    Eigen::Matrix<float, 3, 4> contactForces;
    contactForces << -2.131, -0.035, -33.001,
<<<<<<< HEAD
        -1.214, 0.267, -35.991,
        -1.707, 1.003, -27.901,
        -1.583, 0.719, -30.656;
    map<int, float> motorTorques0;
    motorTorques0 = {{0, 2.6939936984577613}, {1, -0.22165711639465652}, {2, 5.029048678331882}};
    map<int, float> motorTorques1;
    motorTorques1 = {{3, -2.987655100613145}, {4, -0.31842306045338115}, {5, 5.807468375973991}};
    map<int, float> motorTorques2;
    motorTorques2 = {{6, 2.5631615440866584}, {7, -0.15752410207213371}, {8, 4.5724388764139805}};
    map<int, float> motorTorques3;
    motorTorques3 = {{9, -2.2664448021091843}, {10, -0.4335804019389013}, {11, 4.587360482280456}};
    std::vector<map<int, float>> motorTorques4;
    motorTorques4.
        push_back(motorTorques0);
    motorTorques4.
        push_back(motorTorques1);
    motorTorques4.
        push_back(motorTorques2);
    motorTorques4.
        push_back(motorTorques3);
    int cycle = 1;
    for (
        int i = 0;
        i < cycle;
        ++i) {
=======
                     -1.214,   0.267, -35.991,
                     -1.707,   1.003, -27.901,
                     -1.583,   0.719, -30.656;
    map<int, float> motorTorques0;
    motorTorques0 = { {0,2.6939936984577613}, {1, -0.22165711639465652}, {2,5.029048678331882} };
    map<int, float> motorTorques1;
    motorTorques1 = { {3, -2.987655100613145}, {4, -0.31842306045338115}, {5, 5.807468375973991} };
    map<int, float> motorTorques2;
    motorTorques2 = { {6, 2.5631615440866584}, {7, -0.15752410207213371}, {8, 4.5724388764139805} };
    map<int, float> motorTorques3;
    motorTorques3 = { {9, -2.2664448021091843}, {10, -0.4335804019389013}, {11, 4.587360482280456} };
    std::vector<map<int, float>> motorTorques4;
    motorTorques4.push_back(motorTorques0);
    motorTorques4.push_back(motorTorques1);
    motorTorques4.push_back(motorTorques2);
    motorTorques4.push_back(motorTorques3);
    int cycle = 1;
    for (int i=0; i<cycle; ++i) {
>>>>>>> origin/develop
        actionContactForce = stanceLegPtr->GetActionTest(desiredLegStates,
                                                         robotComPosition,
                                                         robotComVelocity,
                                                         robotComRpy,
                                                         robotComRpyRate,
                                                         desiredComPosition,
                                                         desiredComVelocity,
                                                         desiredComAngularVelocity,
                                                         contactForces,
                                                         motorTorques4);
    }
    std::map<int, MotorCommand> action;
    action = std::get<0>(actionContactForce);
    std::map<int, MotorCommand> actionTrue;
<<<<<<< HEAD
    actionTrue = {{0, {0, 0, 0, 0, 2.6939936984577613}},
                  {1, {0, 0, 0, 0, -0.22165711639465652}},
                  {2, {0, 0, 0, 0, 5.029048678331882}},
                  {3, {0, 0, 0, 0, -2.987655100613145}},
                  {4, {0, 0, 0, 0, -0.31842306045338115}},
                  {5, {0, 0, 0, 0, 5.807468375973991}},
                  {6, {0, 0, 0, 0, 2.5631615440866584}},
                  {7, {0, 0, 0, 0, -0.15752410207213371}},
                  {8, {0, 0, 0, 0, 4.5724388764139805}},
                  {9, {0, 0, 0, 0, -2.2664448021091843}},
                  {10, {0, 0, 0, 0, -0.4335804019389013}},
                  {11, {0, 0, 0, 0, 4.587360482280456}}};
    cout << "Test action!" <<
         endl;
    for (
        map<int, MotorCommand>::iterator it = action.begin();
        it != action.
            end();
        ++it) {
        EXPECT_NEAR(it
                        ->second.tua, actionTrue[it->first].tua, 1E-3);
=======
    actionTrue = { {0, {0, 0, 0, 0, 2.6939936984577613}},
                   {1, {0, 0, 0, 0, -0.22165711639465652}},
                   {2, {0, 0, 0, 0, 5.029048678331882}},
                   {3, {0, 0, 0, 0, -2.987655100613145}},
                   {4, {0, 0, 0, 0, -0.31842306045338115}},
                   {5, {0, 0, 0, 0, 5.807468375973991}},
                   {6, {0, 0, 0, 0, 2.5631615440866584}},
                   {7, {0, 0, 0, 0, -0.15752410207213371}},
                   {8, {0, 0, 0, 0, 4.5724388764139805}},
                   {9, {0, 0, 0, 0, -2.2664448021091843}},
                   {10, {0, 0, 0, 0, -0.4335804019389013}},
                   {11, {0, 0, 0, 0, 4.587360482280456}} };
    cout<< "Test action!" <<endl;
    for(map<int, MotorCommand>::iterator it = action.begin(); it!=action.end(); ++it) {
        EXPECT_NEAR(it->second.tua, actionTrue[it->first].tua, 1E-3);
>>>>>>> origin/develop
    }
    Eigen::Matrix<float, 3, 4> contactForce;
    contactForce = std::get<1>(actionContactForce);
    Eigen::Matrix<float, 3, 4> contactForceTrue;
<<<<<<< HEAD
    contactForceTrue << -2.131, -0.035, -33.001,
        -1.214, 0.267, -35.991,
        -1.707, 1.003, -27.901,
        -1.583, 0.719, -30.656;
    cout << "Test contactForce!" <<
         endl;
//ASSERT_TRUE(contactForce.isApprox(contactForceTrue));
    for (
        int i = 0;
        i < contactForce.
            rows();
        ++i) {
        for (
            int j = 0;
            j < contactForce.
                cols();
            ++j) {
            EXPECT_NEAR(contactForce(i, j), contactForceTrue(i, j),
                        1E-3);
=======
    contactForceTrue << -2.131,  -0.035, -33.001,
                        -1.214,   0.267, -35.991,
                        -1.707,   1.003, -27.901,
                        -1.583,   0.719, -30.656;
    cout<< "Test contactForce!" <<endl;
    //ASSERT_TRUE(contactForce.isApprox(contactForceTrue));
    for (int i=0; i<contactForce.rows(); ++i) {
        for (int j=0; j<contactForce.cols(); ++j) {
            EXPECT_NEAR(contactForce(i,j), contactForceTrue(i,j), 1E-3);
>>>>>>> origin/develop
        }
    }
}

<<<<<<< HEAD
TEST_F(TestStanceLeg, TestEstimateRobotHeight
)
{
    stanceLegPtr = build();
    Eigen::Matrix<bool, 4, 1> contacts;
=======
TEST_F(TestStanceLeg, TestEstimateRobotHeight)
{
    stanceLegPtr = build();
    Eigen::Matrix<int, 4, 1> contacts;
>>>>>>> origin/develop
    contacts << 0, 1, 1, 0;
    Eigen::Matrix<float, 4, 1> baseOrientation;
    baseOrientation << 0.00840119742094525, -0.01714402056388364, 0.005074046575231602, 0.9998048592061112;
    Eigen::Matrix<float, 3, 3> rotMat;
    rotMat << 0.999360673220515, -0.010434173446201228, -0.034196093998197535,
<<<<<<< HEAD
        0.009858052240817502, 0.9998073478664934, -0.016973095126877568,
        0.0343666062662039, 0.016625136891566073, 0.9992710048815989;
    Eigen::Matrix<float, 4, 3> footPositions;
    footPositions << 0.157, -0.124, -0.212,
        0.146, 0.125, -0.259,
        -0.213, -0.134, -0.242,
        -0.207, 0.119, -0.206;
    int cycle = 10;
    float bodyHeight;
    for (
        int i = 0;
        i < cycle;
        ++i) {
=======
              0.009858052240817502, 0.9998073478664934, -0.016973095126877568,
              0.0343666062662039, 0.016625136891566073, 0.9992710048815989;
    Eigen::Matrix<float, 4, 3> footPositions;
    footPositions << 0.157, -0.124, -0.212,
                     0.146,  0.125, -0.259,
                     -0.213, -0.134, -0.242,
                     -0.207,  0.119, -0.206;
    int cycle = 10;
    float bodyHeight;
    for (int i=0; i<cycle; ++i)
    {
>>>>>>> origin/develop
        bodyHeight = stanceLegPtr->EstimateRobotHeightTest(contacts,
                                                           baseOrientation,
                                                           rotMat,
                                                           footPositions);
    }
    float bodyHeightTrue = 0.25125713023747087;
<<<<<<< HEAD
    cout << "Test bodyHeight !" <<
         endl;
    EXPECT_NEAR(bodyHeight, bodyHeightTrue,
                1E-2);
=======
    cout<< "Test bodyHeight !" <<endl;
    EXPECT_NEAR(bodyHeight, bodyHeightTrue, 1E-2);
>>>>>>> origin/develop
}

int main(int argc, char **argv)
{
    // 分析gtest程序的命令行参数
    ::testing::InitGoogleTest(&argc, argv);

    // 调用RUN_ALL_TESTS()运行所有测试用例
    // main函数返回RUN_ALL_TESTS()的运行结果
    return RUN_ALL_TESTS();
}
