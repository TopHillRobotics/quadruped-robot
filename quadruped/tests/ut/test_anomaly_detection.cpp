#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include "utils/tools.h"
#include "robots/robot_a1.h"
#include "estimators/ground_estimator.h"
#include "estimators/anomaly_detection.h"

using namespace std;
using namespace Quadruped;
using ::testing::_;
using ::testing::Return;


// 定义测试类FooTest
class TestAnomalyDetection : public testing::Test
{
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
};


class MockA1Robot : public RobotA1
{
public:
    MockA1Robot(std::string configFilePathIn) : RobotA1(configFilePathIn) {}
    virtual ~MockA1Robot() = default;
    // MOCK_METHOD0(GetBaseRollPitchYaw, Vec3<float>());
    MOCK_METHOD((Eigen::Matrix<float, 3, 4>), GetFootPositionsInBaseFrame, (), ());
    MOCK_METHOD((Eigen::Matrix<float, 3, 4>), GetHipPositionsInBaseFrame, (), (const));
    // MOCK_METHOD((void), ApplyAction, (const Eigen::MatrixXf &motorCommands, MotorMode motorControlMode), (override));
};


TEST_F(TestAnomalyDetection, Update)
{
    string exepath = robotics::utils::GetExePath();
    std::size_t found = exepath.find("robots/");
    string homeDir = exepath.substr(0, found) + "robots/src/ascend-quadruped-cpp/";
    std::cout << homeDir << std::endl;
    MockA1Robot quadruped(homeDir + "config/a1/a1_robot.yaml");
    
    // 定义函数行为
    Eigen::Matrix<float, 3, 4> res;
    res << 0.18, 0.06, -0.19, -0.19,
            -0.12, 0.12, -0.12, 0.12,
            -0.20, -0.25, -0.25, -0.25;

    EXPECT_CALL(quadruped, GetFootPositionsInBaseFrame())
        // .Times(1) // 执行一次
        .WillOnce(Return(res))        // 第一次执行的返回值
        .WillRepeatedly(Return(res)); // 后续执行的返回值

    quadruped.ReceiveObservation();
    std::cout << "BaseOrientation:\n"
              << quadruped.GetBaseOrientation().transpose() << std::endl;
    // std::cout << quadruped.GetFootPositionsInBaseFrame() << std::endl;
            //   GetHipPositionsInBaseFrame
    WorkspaceDetection workSpaceDetection(&quadruped, nullptr);

    
    cout << ">>>>>>>>>>>>>>>> start google test >>>>>>>>>>>>>>>>>" << endl;
    Eigen::Matrix<float, 3, 4> cilpedFootPos;
    cilpedFootPos = workSpaceDetection.Update();
    std::cout << "cilpedFootPos = " << cilpedFootPos <<std::endl;
}


int main(int argc, char **argv)
{
    // 分析gtest程序的命令行参数
    ::testing::InitGoogleTest(&argc, argv);
    // 调用RUN_ALL_TESTS()运行所有测试用例
    // main函数返回RUN_ALL_TESTS()的运行结果
    return RUN_ALL_TESTS();
}