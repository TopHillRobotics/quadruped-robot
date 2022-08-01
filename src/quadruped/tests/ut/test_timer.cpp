#include "gtest/gtest.h"
#include "robots/timer.h"
#include "utils/se3.h"
#include "state_estimator/robot_velocity_estimator.h"

using namespace std;

// 定义测试类FooTest
<<<<<<< HEAD
class TestTimer : public testing::Test {
=======
class TestTimer: public testing::Test {
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
    qrMovingWindowFilter *winFilter;
    Timer timer;
    qrMovingWindowFilter *build()
=======
    	cout << "test start" << endl;
    }
 
    // Code here will be called immediately after each test (right before the destructor)
    void TearDown()
    {
	    cout << "test end" << endl;
    }
 
public:
    qrMovingWindowFilter* winFilter;
    Timer timer;
    qrMovingWindowFilter* build() 
>>>>>>> origin/develop
    {
        return new qrMovingWindowFilter(120);
    }

};
<<<<<<< HEAD

TEST_F(TestTimer, TestGetTime
)
=======
 
TEST_F(TestTimer, TestGetTime)
>>>>>>> origin/develop
{
    auto tik = timer.GetTime();
    winFilter = build();
    double Values[5] = {0.75014373, 0.90864498, 0.03528129, 0.74184424, 0.99092506};
    double res;
    long long cycle = 100000;
<<<<<<< HEAD
    for (
        int i = 0;
        i < cycle;
        ++i) {
        res = winFilter->CalculateAverage(Values[i % 5]);
    }
    auto tok = timer.GetTime();
    printf("duration = %f s\n", tok - tik);

}

int main(int argc, char **argv)
{
    // 分析gtest程序的命令行参数
    ::testing::InitGoogleTest(&argc, argv);

=======
    for(int i = 0; i < cycle; ++i) {
        res = winFilter->CalculateAverage(Values[i%5]);
    }
    auto tok = timer.GetTime();
    printf("duration = %f s\n", tok-tik);

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