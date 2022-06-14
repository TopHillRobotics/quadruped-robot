/*
    Linsen Zhu
*/

#include "gtest/gtest.h"

using namespace std;

// 定义测试类FooTest
<<<<<<< HEAD
class FooTest : public testing::Test {
=======
class FooTest: public testing::Test {
>>>>>>> origin/develop
protected:
    // Code here will be called immediately after the constructor (right before each test)
    void SetUp()
    {
        m_nTarget = 5;
<<<<<<< HEAD
        cout << "test start" << endl;
    }

    // Code here will be called immediately after each test (right before the destructor)
    void TearDown()
    {
        cout << "test end" << endl;
    }

public:
    int IsLargeThan5(const int &nNum);
    int m_nTarget;
};

// 判断入参是否大于5：如果是，则返回0；否则返回-1
int FooTest::IsLargeThan5(const int &nNum)
{
    if (nNum > m_nTarget) {
        return 0;
    } else {
        return -1;
    }
}

TEST_F(FooTest, HandlesInput6
)
{
    cout << "test 1" <<
         endl;
    EXPECT_EQ(IsLargeThan5(6),
              0);
}

TEST_F(FooTest, HandlesInput5
)
{
    cout << "test 2" <<
         endl;
    EXPECT_EQ(IsLargeThan5(5),
              0);
}

TEST_F(FooTest, HandlesInput4
)
{
    EXPECT_EQ(IsLargeThan5(4),
              -1);
}

=======
    	cout << "test start" << endl;
    }
 
    // Code here will be called immediately after each test (right before the destructor)
    void TearDown()
    {
	    cout << "test end" << endl;
    }
 
public:
    int IsLargeThan5(const int & nNum);
    int m_nTarget;
};
 
// 判断入参是否大于5：如果是，则返回0；否则返回-1
int FooTest::IsLargeThan5(const int & nNum)
{
    if (nNum > m_nTarget)
    {
        return 0;
    }
    else
    {
        return -1;
    }
}
 
TEST_F(FooTest, HandlesInput6)
{
    cout << "test 1" << endl;
    EXPECT_EQ(IsLargeThan5(6), 0);
}
 
TEST_F(FooTest, HandlesInput5)
{ 
    cout << "test 2" << endl;
    EXPECT_EQ(IsLargeThan5(5), 0);
}
 
TEST_F(FooTest, HandlesInput4)
{
    EXPECT_EQ(IsLargeThan5(4), -1);
}
 
>>>>>>> origin/develop
int main(int argc, char **argv)
{
    // 分析gtest程序的命令行参数
    ::testing::InitGoogleTest(&argc, argv);
<<<<<<< HEAD

=======
 
>>>>>>> origin/develop
    // 调用RUN_ALL_TESTS()运行所有测试用例
    // main函数返回RUN_ALL_TESTS()的运行结果
    return RUN_ALL_TESTS();
}


