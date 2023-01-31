#include <unistd.h>
#include "gtest/gtest.h"
#include "estimators/robot_velocity_estimator.h"



// int main()
// {
//     Robot* robot = new A1Robot("../../config/a1_robot.yaml");
//     printf("time = %f\n",robot->timeStep);
//     return 1;     
// }

using namespace std;

// 定义测试类FooTest
<<<<<<< HEAD
class TestEstimator : public testing::Test {
=======
class TestEstimator: public testing::Test {
>>>>>>> origin/develop
protected:
    // Code here will be called immediately after the constructor (right before each test)
    void SetUp()
    {
<<<<<<< HEAD
        cout << "test start" << endl;
    }

    // Code here will be called immediately after each test (right before the destructor)
    void TearDown()
    {
        cout << "test end" << endl;
    }

public:
    MovingWindowFilter *winFilter;
    RobotVelocityEstimator *est;
    MovingWindowFilter *build()
=======
    	cout << "test start" << endl;
    }
 
    // Code here will be called immediately after each test (right before the destructor)
    void TearDown()
    {
	    cout << "test end" << endl;
    }
 
public:
    MovingWindowFilter* winFilter;
    RobotVelocityEstimator* est;
    MovingWindowFilter* build() 
>>>>>>> origin/develop
    {
        return new MovingWindowFilter(120);
    }
};
<<<<<<< HEAD

=======
 
>>>>>>> origin/develop
/*
TEST_F(TestEstimator, TestNeumaierSum)
{
    winFilter = build();
    int cycle = 100;
    double Values[5] = {0.75014373, 0.90864498, 0.03528129, 0.74184424, 0.99092506};
    for(int i = 0; i < cycle; ++i) {
        winFilter->NeumaierSum(Values[i%5]);
    }
    double s = winFilter->GetSum();
    EXPECT_NEAR(s, 68.53678599999998, 1E-5);
}

TEST_F(TestEstimator, TestCalculateAverage)
{
    winFilter = build();
    double Values[5] = {0.75014373, 0.90864498, 0.03528129, 0.74184424, 0.99092506};
    double res;
    long long cycle = 100000;
    for(int i = 0; i < cycle; ++i) {
        res = winFilter->CalculateAverage(Values[i%5]);
    }
    EXPECT_NEAR(res, 0.6853678599999999, 1E-5);
}

TEST_F(TestEstimator, TestComputeDeltaTime)
{
    A1Robot* robot = new A1Robot("../../config/a1_robot.yaml");
    est = new RobotVelocityEstimator(robot, 0.01, 0.01, 0.01, 120);
    LowState state;
    int cycle = 5;
    float Values[5] = {0.03528129*1000, 0.74184424*1000, 0.75014373*1000, 0.90864498*1000, 0.99092506*1000};
    float Res[5] = {0.03528129, 0.70656295, 0.00829949, 0.15850125, 0.08228008};
    for(int i = 0; i < cycle; ++i) {
        state.tick = Values[i%5];
        auto r = est->ComputeDeltaTime(&state);
        EXPECT_NEAR(r, Res[i%5], 1E-5);
    }
}
*/

<<<<<<< HEAD
TEST_F(TestEstimator, TestKalmanFilter
)
{
    A1Robot *robot = new A1Robot("../../config/a1_robot.yaml");
    est = new RobotVelocityEstimator(robot, 0.1, 0.1, 0.1, 120);
    LowState state;
    int cycle = 5;
    float Values[5] = {0.03528129 * 1000, 0.74184424 * 1000, 0.75014373 * 1000, 0.90864498 * 1000, 0.99092506 * 1000};
    float Res[5] = {0.03528129, 0.70656295, 0.00829949, 0.15850125, 0.08228008};
    for (
        int i = 0;
        i < cycle;
        ++i) {
        state.
            tick = Values[i % 5];
        auto r = est->ComputeDeltaTime(&state);
        EXPECT_NEAR(r, Res[i % 5],
                    1E-5);
    }
}

int main(int argc, char **argv)
{
    // 分析gtest程序的命令行参数
    ::testing::InitGoogleTest(&argc, argv);

=======
TEST_F(TestEstimator, TestKalmanFilter)
{
    A1Robot* robot = new A1Robot("../../config/a1_robot.yaml");
    est = new RobotVelocityEstimator(robot, 0.1, 0.1, 0.1, 120);
    LowState state;
    int cycle = 5;
    float Values[5] = {0.03528129*1000, 0.74184424*1000, 0.75014373*1000, 0.90864498*1000, 0.99092506*1000};
    float Res[5] = {0.03528129, 0.70656295, 0.00829949, 0.15850125, 0.08228008};
    for(int i = 0; i < cycle; ++i) {
        state.tick = Values[i%5];
        auto r = est->ComputeDeltaTime(&state);
        EXPECT_NEAR(r, Res[i%5], 1E-5);
    }
}




int main(int argc, char** argv)
{
    // 分析gtest程序的命令行参数
    ::testing::InitGoogleTest(&argc, argv);
 
>>>>>>> origin/develop
    // 调用RUN_ALL_TESTS()运行所有测试用例
    // main函数返回RUN_ALL_TESTS()的运行结果
    return RUN_ALL_TESTS();
}