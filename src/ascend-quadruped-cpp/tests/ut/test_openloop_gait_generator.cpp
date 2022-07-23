//
// Created by xmc on 2021/11/2.
//

#include "gtest/gtest.h"
#include "mpc_controller/openloop_gait_generator.h"

using namespace std;

class TestGaitGenerator : public testing::Test {
public:
    TestGaitGenerator();
    ~TestGaitGenerator() = default;

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

    OpenloopGaitGenerator *gaitGenerator;

};

<<<<<<< HEAD
TestGaitGenerator::TestGaitGenerator()
{
    gaitGenerator = new OpenloopGaitGenerator();
}

TEST_F(TestGaitGenerator, Test1_Reset
)
{
    std::cout << "--------------------------Test1--------------------------" <<
              std::endl;

// case_1 input
    float currentTime = 0;

//case_1 output
    Eigen::Matrix<int, 4, 1> legState(0, 1, 1, 0);
    Eigen::Matrix<int, 4, 1> desiredLegState(0, 1, 1, 0);

//call function
    gaitGenerator->
        Reset(currentTime);

//check output
    EXPECT_EQ(legState, gaitGenerator
        ->legState);
    EXPECT_EQ(desiredLegState, gaitGenerator
        ->desiredLegState);
}

TEST_F(TestGaitGenerator, Test2_Update
)
{
    std::cout << "--------------------------Test2--------------------------" <<
              std::endl;

// case_2 input
    float currentTime = 2.63;

// case_2 output
    Eigen::Matrix<int, 4, 1> desiredLegState(1, 0, 0, 1);
    map<int, float> normalizedPhase;
=======
TestGaitGenerator::TestGaitGenerator() {
    gaitGenerator = new OpenloopGaitGenerator();
}

TEST_F(TestGaitGenerator, Test1_Reset) {
    std::cout << "--------------------------Test1--------------------------" << std::endl;

    // case_1 input
    float currentTime = 0;

    //case_1 output
    Eigen::Matrix<int, 4, 1> legState(0, 1, 1, 0);
    Eigen::Matrix<int, 4, 1> desiredLegState(0, 1, 1, 0);

    //call function
    gaitGenerator->Reset(currentTime);

    //check output
    EXPECT_EQ(legState,gaitGenerator->legState);
    EXPECT_EQ(desiredLegState,gaitGenerator->desiredLegState);
}

TEST_F(TestGaitGenerator, Test2_Update) {
    std::cout << "--------------------------Test2--------------------------" << std::endl;

    // case_2 input
    float currentTime = 2.63;

    // case_2 output
    Eigen::Matrix<int, 4, 1> desiredLegState(1,0,0,1);
    map<int,float> normalizedPhase;
>>>>>>> origin/develop
    normalizedPhase[0] = 0.601;
    normalizedPhase[1] = 0.652;
    normalizedPhase[2] = 0.652;
    normalizedPhase[3] = 0.601;
<<<<<<< HEAD
    Eigen::Matrix<int, 4, 1> legState(1, 0, 0, 1);

// calll function
    gaitGenerator->
        Update(currentTime);

//check output
    EXPECT_NEAR(normalizedPhase[0], gaitGenerator
        ->normalizedPhase(0, 0), 0.001);
    EXPECT_NEAR(normalizedPhase[1], gaitGenerator
        ->normalizedPhase(1, 0), 0.001);
    EXPECT_NEAR(normalizedPhase[2], gaitGenerator
        ->normalizedPhase(2, 0), 0.001);
    EXPECT_NEAR(normalizedPhase[3], gaitGenerator
        ->normalizedPhase(3, 0), 0.001);
    EXPECT_EQ(legState, gaitGenerator
        ->legState);
}
int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);

=======
    Eigen::Matrix<int, 4, 1> legState(1,0,0,1);

    // calll function
    gaitGenerator->Update(currentTime);

    //check output
    EXPECT_NEAR(normalizedPhase[0],gaitGenerator->normalizedPhase(0,0),0.001);
    EXPECT_NEAR(normalizedPhase[1],gaitGenerator->normalizedPhase(1,0),0.001);
    EXPECT_NEAR(normalizedPhase[2],gaitGenerator->normalizedPhase(2,0),0.001);
    EXPECT_NEAR(normalizedPhase[3],gaitGenerator->normalizedPhase(3,0),0.001);
    EXPECT_EQ(legState,gaitGenerator->legState);
}
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);


>>>>>>> origin/develop
    return RUN_ALL_TESTS();
}
