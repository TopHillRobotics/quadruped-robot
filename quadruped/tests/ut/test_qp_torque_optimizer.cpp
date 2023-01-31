#include <unistd.h>
#include "gtest/gtest.h"
#include "utils/se3.h"
#include "controllers/qp_torque_optimizer.h"

using namespace robotics;
using namespace std;

// 定义测试类FooTest
class TestQPSolver : public testing::Test {
protected:
    // Code here will be called immediately after the constructor (right before each test)
<<<<<<< HEAD
    void SetUp()
    {
=======
    void SetUp() {
>>>>>>> origin/develop
        cout << "test start" << endl;
    }

    // Code here will be called immediately after each test (right before the destructor)
<<<<<<< HEAD
    void TearDown()
    {
=======
    void TearDown() {
>>>>>>> origin/develop
        cout << "test end" << endl;
    }

public:
    Timer timer;
};

<<<<<<< HEAD
TEST_F(TestQPSolver, TestComputeMassMatrix
)
{
    float robotMass = 13.741;
    Eigen::Matrix<float, 3, 3> robotInertia;
    robotInertia << 0.0158533 * 4, 0., 0.,
        0., 0.0377999 * 4, 0.,
        0., 0., 0.0456542 * 4;
    Eigen::Matrix<float, 4, 3> footPositions;
    footPositions << 0.15693, -0.12621, -0.18666,
        0.14516, 0.12558, -0.25778,
        -0.21337, -0.13231, -0.23538,
        -0.20874, 0.1211, -0.17989;

    int cycle = 1;
    Eigen::Matrix<float, 6, 12> massMatrix;
    for (
        int i = 0;
        i < cycle;
        ++i) {
=======

TEST_F(TestQPSolver, TestComputeMassMatrix) {
    float robotMass = 13.741;
    Eigen::Matrix<float, 3, 3> robotInertia;
    robotInertia << 0.0158533 * 4, 0., 0.,
            0., 0.0377999 * 4, 0.,
            0., 0., 0.0456542 * 4;
    Eigen::Matrix<float, 4, 3> footPositions;
    footPositions << 0.15693, -0.12621, -0.18666,
            0.14516, 0.12558, -0.25778,
            -0.21337, -0.13231, -0.23538,
            -0.20874, 0.1211, -0.17989;

    int cycle = 1;
    Eigen::Matrix<float, 6, 12> massMatrix;
    for (int i = 0; i < cycle; ++i) {
>>>>>>> origin/develop
        massMatrix = ComputeMassMatrix(robotMass, robotInertia, footPositions);
    }
    Eigen::Matrix<float, 6, 12> massMatrixTrue;
    massMatrixTrue << 0.07277, 0., 0., 0.07277, 0., 0., 0.07277, 0., 0., 0.07277, 0., 0.,
<<<<<<< HEAD
        0., 0.07277, 0., 0., 0.07277, 0., 0., 0.07277, 0., 0., 0.07277, 0.,
        0., 0., 0.07277, 0., 0., 0.07277, 0., 0., 0.07277, 0., 0., 0.07277,
        0., 2.9435, -1.99026, 0., 4.0651, 1.98031, 0., 3.71179, -2.08654, 0., 2.83682, 1.90969,
        -1.23451, 0., -1.03788, -1.70491, 0., -0.96008, -1.55673, 0., 1.41119, -1.18976, 0., 1.38053,
        0.69111, 0.85932, 0., -0.68766, 0.79491, 0., 0.72455, -1.16841, 0., -0.66314, -1.14302, 0.;
    cout << "Test mass_Matrix !" <<
         endl;
    for (
        int i = 0;
        i < massMatrixTrue.
            rows();
        ++i) {
        for (
            int j = 0;
            j < massMatrixTrue.
                cols();
            ++j) {
            EXPECT_NEAR(massMatrix(i, j), massMatrixTrue(i, j),
                        1E-4);
=======
            0., 0.07277, 0., 0., 0.07277, 0., 0., 0.07277, 0., 0., 0.07277, 0.,
            0., 0., 0.07277, 0., 0., 0.07277, 0., 0., 0.07277, 0., 0., 0.07277,
            0., 2.9435, -1.99026, 0., 4.0651, 1.98031, 0., 3.71179, -2.08654, 0., 2.83682, 1.90969,
            -1.23451, 0., -1.03788, -1.70491, 0., -0.96008, -1.55673, 0., 1.41119, -1.18976, 0., 1.38053,
            0.69111, 0.85932, 0., -0.68766, 0.79491, 0., 0.72455, -1.16841, 0., -0.66314, -1.14302, 0.;
    cout << "Test mass_Matrix !" << endl;
    for (int i = 0; i < massMatrixTrue.rows(); ++i) {
        for (int j = 0; j < massMatrixTrue.cols(); ++j) {
            EXPECT_NEAR(massMatrix(i, j), massMatrixTrue(i, j), 1E-4);
>>>>>>> origin/develop
        }
    }
}

<<<<<<< HEAD
TEST_F(TestQPSolver, TestComputeConstraintMatrix
)
{
=======
TEST_F(TestQPSolver, TestComputeConstraintMatrix) {
>>>>>>> origin/develop
    float mpcBodyMass;
    mpcBodyMass = 13.741;
    float frictionCoef;
    frictionCoef = 0.45;
<<<<<<< HEAD
    Eigen::Matrix<bool, 4, 1> contacts;
=======
    Eigen::Matrix<int, 4, 1> contacts;
>>>>>>> origin/develop
    contacts << 1, 0, 0, 1;

    int cycle = 1;
    std::tuple<Eigen::Matrix<float, 12, 24>, Eigen::Matrix<float, 24, 1>> constraintMatrix;
    Eigen::Matrix<float, 12, 24> A;
    Eigen::Matrix<float, 24, 1> lb;
<<<<<<< HEAD
    for (
        int i = 0;
        i < cycle;
        ++i) {
=======
    for (int i = 0; i < cycle; ++i) {
>>>>>>> origin/develop
        constraintMatrix = ComputeConstraintMatrix(mpcBodyMass, contacts, frictionCoef);
    }
    A = std::get<0>(constraintMatrix);
    lb = std::get<1>(constraintMatrix);
    Eigen::Matrix<float, 12, 24> ATrue = Eigen::Matrix<float, 12, 24>::Zero();
    ATrue.block<1, 2>(0, 8) << 1, -1;
    ATrue.block<1, 2>(1, 10) << 1, -1;
    ATrue.block<1, 2>(2, 0) << 1, -1;
    ATrue.block<1, 4>(2, 8) << 0.45, 0.45, 0.45, 0.45;
    ATrue.block<1, 2>(3, 12) << 1, -1;
    ATrue.block<1, 2>(4, 14) << 1, -1;
    ATrue.block<1, 2>(5, 2) << 1, -1;
    ATrue.block<1, 4>(5, 12) << 0.45, 0.45, 0.45, 0.45;
    ATrue.block<1, 2>(6, 16) << 1, -1;
    ATrue.block<1, 2>(7, 18) << 1, -1;
    ATrue.block<1, 2>(8, 4) << 1, -1;
    ATrue.block<1, 4>(8, 16) << 0.45, 0.45, 0.45, 0.45;
    ATrue.block<1, 2>(9, 20) << 1, -1;
    ATrue.block<1, 2>(10, 22) << 1, -1;
    ATrue.block<1, 2>(11, 6) << 1, -1;
    ATrue.block<1, 4>(11, 20) << 0.45, 0.45, 0.45, 0.45;
    Eigen::Matrix<float, 24, 1> lbTrue = Eigen::Matrix<float, 24, 1>::Zero();
    lbTrue.block<8, 1>(0, 0) << 13.46618, -1346.618, -1e-7, -1e-7, -1e-7, -1e-7, 13.46618, -1346.618;
<<<<<<< HEAD
    cout << "Test A and lb !" <<
         endl;
//ASSERT_TRUE(A.isApprox(ATrue));
    for (
        int i = 0;
        i < A.
            rows();
        ++i) {
        for (
            int j = 0;
            j < A.
                cols();
            ++j) {
            EXPECT_NEAR(A(i, j), ATrue(i, j),
                        1E-4);
        }
    }
//ASSERT_TRUE(lb.isApprox(lbTrue));
    for (
        int i = 0;
        i < lb.
            rows();
        ++i) {
        for (
            int j = 0;
            j < lb.
                cols();
            ++j) {
            EXPECT_NEAR(lb(i, j), lbTrue(i, j),
                        1E-4);
=======
    cout << "Test A and lb !" << endl;
    //ASSERT_TRUE(A.isApprox(ATrue));
    for (int i = 0; i < A.rows(); ++i) {
        for (int j = 0; j < A.cols(); ++j) {
            EXPECT_NEAR(A(i, j), ATrue(i, j), 1E-4);
        }
    }
    //ASSERT_TRUE(lb.isApprox(lbTrue));
    for (int i = 0; i < lb.rows(); ++i) {
        for (int j = 0; j < lb.cols(); ++j) {
            EXPECT_NEAR(lb(i, j), lbTrue(i, j), 1E-4);
>>>>>>> origin/develop
        }
    }
}

<<<<<<< HEAD
TEST_F(TestQPSolver, TestComputeObjectiveMatrix
)
{
    Eigen::Matrix<float, 6, 12> massMatrix;
    massMatrix << 0.07277, 0., 0., 0.07277, 0., 0., 0.07277, 0., 0., 0.07277, 0., 0.,
        0., 0.07277, 0., 0., 0.07277, 0., 0., 0.07277, 0., 0., 0.07277, 0.,
        0., 0., 0.07277, 0., 0., 0.07277, 0., 0., 0.07277, 0., 0., 0.07277,
        0., 3.96283, -2.10398, 0., 2.87175, 2.06941, 0., 2.8814, -2.15465, 0., 3.88901, 1.9025,
        -1.66201, 0., -1.01018, -1.20441, 0., -1.07316, -1.20846, 0., 1.33132, -1.63105, 0., 1.42231,
        0.7306, 0.83639, 0., -0.7186, 0.88853, 0., 0.7482, -1.10228, 0., -0.66064, -1.17762, 0.;
=======
TEST_F(TestQPSolver, TestComputeObjectiveMatrix) {
    Eigen::Matrix<float, 6, 12> massMatrix;
    massMatrix << 0.07277, 0., 0., 0.07277, 0., 0., 0.07277, 0., 0., 0.07277, 0., 0.,
            0., 0.07277, 0., 0., 0.07277, 0., 0., 0.07277, 0., 0., 0.07277, 0.,
            0., 0., 0.07277, 0., 0., 0.07277, 0., 0., 0.07277, 0., 0., 0.07277,
            0., 3.96283, -2.10398, 0., 2.87175, 2.06941, 0., 2.8814, -2.15465, 0., 3.88901, 1.9025,
            -1.66201, 0., -1.01018, -1.20441, 0., -1.07316, -1.20846, 0., 1.33132, -1.63105, 0., 1.42231,
            0.7306, 0.83639, 0., -0.7186, 0.88853, 0., 0.7482, -1.10228, 0., -0.66064, -1.17762, 0.;
>>>>>>> origin/develop
    Eigen::Matrix<float, 6, 1> desiredAcc;
    desiredAcc << 0.8462, 0.00923, -1.45496, -1.11834, 0.04254, 2.25868;
    Eigen::Matrix<float, 6, 1> accWeight;
    accWeight << 1., 1., 1., 10., 10., 1.;
    float regWeight;
    regWeight = 0.0001;

    int cycle = 1;
    std::tuple<Eigen::Matrix<float, 12, 12>, Eigen::Matrix<float, 12, 1>> objectiveMatrix;
    Eigen::Matrix<float, 12, 12> quadTerm;
    Eigen::Matrix<float, 12, 1> linearTerm;
<<<<<<< HEAD
    for (
        int i = 0;
        i < cycle;
        ++i) {
=======
    for (int i = 0; i < cycle; ++i) {
>>>>>>> origin/develop
        objectiveMatrix = ComputeObjectiveMatrix(massMatrix, desiredAcc, accWeight, regWeight);
    }
    quadTerm = std::get<0>(objectiveMatrix);
    linearTerm = std::get<1>(objectiveMatrix);
    Eigen::Matrix<float, 12, 12> quadTermTrue;
    quadTermTrue
<<<<<<< HEAD
        << 28.16206, 0.61117, 16.7894, 19.49792, 0.64926, 17.83608, 20.63683, -0.80523, -22.12654, 26.63105, -0.86027, -23.63884,
        0.61117, 157.74518, -83.37712, -0.60093, 114.55122, 82.00741, 0.62588, 113.26859, -85.38492, -0.55245, 153.1353, 75.39286,
        16.7894, -83.37712, 54.47739, 12.16684, -60.42104, -32.69388, 12.20773, -60.6241, 31.89012, 16.47665, -81.82395, -54.39065,
        19.49792, -0.60093, 12.16684, 15.02792, -0.6384, 12.92534, 14.02264, 0.79219, -16.03445, 20.12477, 0.84633, -17.13038,
        0.64926, 114.55122, -60.42104, -0.6384, 83.26449, 59.4285, 0.66489, 81.77276, -61.87604, -0.5869, 110.64177, 54.63512,
        17.83608, 82.00741, -32.69388, 12.92534, 59.4285, 54.34671, 12.96878, 59.62823, -58.87023, 17.50383, 80.47976, 24.11236,
        20.63683, 0.62588, 12.20773, 14.02264, 0.66489, 12.96878, 15.169, -0.82462, -16.08834, 19.22177, -0.88099, -17.18795,
        -0.80523, 113.26859, -60.6241, 0.79219, 81.77276, 59.62823, -0.82462, 84.24527, -62.08399, 0.72831, 113.36151, 54.81874,
        -22.12654, -85.38492, 31.89012, -16.03445, -61.87604, -58.87023, -16.08834, -62.08399, 64.15446, -21.71436, -83.79435, -22.05133,
        26.63105, -0.55245, 16.47665, 20.12477, -0.5869, 17.50383, 19.22177, 0.72831, -21.71436, 27.04517, 0.77808, -23.19849,
        -0.86027, 153.1353, -81.82395, 0.84633, 110.64177, 80.47976, -0.88099, 113.36151, -83.79435, 0.77808, 152.63613, 73.98843,
        -23.63884, 75.39286, -54.39065, -17.13038, 54.63512, 24.11236, -17.18795, 54.81874, -22.05133, -23.19849, 73.98843, 56.42996;
    Eigen::Matrix<float, 12, 1> linearTermTrue;
    linearTermTrue
        << 1.00477, -42.42803, 23.70722, -2.07384, -30.10832, -22.99223, 1.23745, -34.71284, 25.26988, -2.12441, -46.15145, -20.06401;
    cout << "Test quad_term and linear_term !" <<
         endl;
    for (
        int i = 0;
        i < quadTerm.
            rows();
        ++i) {
        for (
            int j = 0;
            j < quadTerm.
                cols();
            ++j) {
            EXPECT_NEAR(quadTerm(i, j), quadTermTrue(i, j),
                        1E-3);
        }
    }
    for (
        int i = 0;
        i < linearTerm.
            rows();
        ++i) {
        for (
            int j = 0;
            j < linearTerm.
                cols();
            ++j) {
            EXPECT_NEAR(linearTerm(i, j), linearTermTrue(i, j),
                        1E-3);
=======
            << 28.16206, 0.61117, 16.7894, 19.49792, 0.64926, 17.83608, 20.63683, -0.80523, -22.12654, 26.63105, -0.86027, -23.63884,
            0.61117, 157.74518, -83.37712, -0.60093, 114.55122, 82.00741, 0.62588, 113.26859, -85.38492, -0.55245, 153.1353, 75.39286,
            16.7894, -83.37712, 54.47739, 12.16684, -60.42104, -32.69388, 12.20773, -60.6241, 31.89012, 16.47665, -81.82395, -54.39065,
            19.49792, -0.60093, 12.16684, 15.02792, -0.6384, 12.92534, 14.02264, 0.79219, -16.03445, 20.12477, 0.84633, -17.13038,
            0.64926, 114.55122, -60.42104, -0.6384, 83.26449, 59.4285, 0.66489, 81.77276, -61.87604, -0.5869, 110.64177, 54.63512,
            17.83608, 82.00741, -32.69388, 12.92534, 59.4285, 54.34671, 12.96878, 59.62823, -58.87023, 17.50383, 80.47976, 24.11236,
            20.63683, 0.62588, 12.20773, 14.02264, 0.66489, 12.96878, 15.169, -0.82462, -16.08834, 19.22177, -0.88099, -17.18795,
            -0.80523, 113.26859, -60.6241, 0.79219, 81.77276, 59.62823, -0.82462, 84.24527, -62.08399, 0.72831, 113.36151, 54.81874,
            -22.12654, -85.38492, 31.89012, -16.03445, -61.87604, -58.87023, -16.08834, -62.08399, 64.15446, -21.71436, -83.79435, -22.05133,
            26.63105, -0.55245, 16.47665, 20.12477, -0.5869, 17.50383, 19.22177, 0.72831, -21.71436, 27.04517, 0.77808, -23.19849,
            -0.86027, 153.1353, -81.82395, 0.84633, 110.64177, 80.47976, -0.88099, 113.36151, -83.79435, 0.77808, 152.63613, 73.98843,
            -23.63884, 75.39286, -54.39065, -17.13038, 54.63512, 24.11236, -17.18795, 54.81874, -22.05133, -23.19849, 73.98843, 56.42996;
    Eigen::Matrix<float, 12, 1> linearTermTrue;
    linearTermTrue
            << 1.00477, -42.42803, 23.70722, -2.07384, -30.10832, -22.99223, 1.23745, -34.71284, 25.26988, -2.12441, -46.15145, -20.06401;
    cout << "Test quad_term and linear_term !" << endl;
    for (int i = 0; i < quadTerm.rows(); ++i) {
        for (int j = 0; j < quadTerm.cols(); ++j) {
            EXPECT_NEAR(quadTerm(i, j), quadTermTrue(i, j), 1E-3);
        }
    }
    for (int i = 0; i < linearTerm.rows(); ++i) {
        for (int j = 0; j < linearTerm.cols(); ++j) {
            EXPECT_NEAR(linearTerm(i, j), linearTermTrue(i, j), 1E-3);
>>>>>>> origin/develop
        }
    }
}

<<<<<<< HEAD
TEST_F(TestQPSolver, TestComputeContactForce
)
{
=======
TEST_F(TestQPSolver, TestComputeContactForce) {
>>>>>>> origin/develop
    double start_t = timer.GetTimeSinceReset();
    float bodyMass;
    bodyMass = 13.741;
    Eigen::Matrix<float, 3, 3> bodyInertia;
    bodyInertia << 0.0158533 * 4, 0., 0.,
<<<<<<< HEAD
        0., 0.0377999 * 4, 0.,
        0., 0., 0.0456542 * 4;
    Eigen::Matrix<float, 4, 3> footPositions;
    footPositions << 0.14883, -0.12923, -0.2513,
        0.16907, 0.13086, -0.16741,
        -0.19163, -0.13662, -0.16783,
        -0.21615, 0.12617, -0.23478;
    Eigen::Matrix<float, 6, 1> desiredAcc;
    desiredAcc << 1.04475, -0.08811, -0.89893, -0.23446, 4.63963, 1.38519;
    Eigen::Matrix<bool, 4, 1> contacts;
=======
            0., 0.0377999 * 4, 0.,
            0., 0., 0.0456542 * 4;
    Eigen::Matrix<float, 4, 3> footPositions;
    footPositions << 0.14883, -0.12923, -0.2513,
            0.16907, 0.13086, -0.16741,
            -0.19163, -0.13662, -0.16783,
            -0.21615, 0.12617, -0.23478;
    Eigen::Matrix<float, 6, 1> desiredAcc;
    desiredAcc << 1.04475, -0.08811, -0.89893, -0.23446, 4.63963, 1.38519;
    Eigen::Matrix<int, 4, 1> contacts;
>>>>>>> origin/develop
    contacts << 1, 0, 0, 1;
    Eigen::Matrix<float, 6, 1> accWeight;
    accWeight << 1., 1., 1., 10., 10., 1.;
    float regWeight = 0.0001;
    float frictionCoef = 0.45;

    int cycle = 1;
    Eigen::Matrix<float, 3, 4> X;
<<<<<<< HEAD
    for (
        int i = 0;
        i < cycle;
        ++i) {
=======
    for (int i = 0; i < cycle; ++i) {
>>>>>>> origin/develop
        X = ComputeContactForceTest(bodyMass, bodyInertia, footPositions,
                                    desiredAcc, contacts, accWeight,
                                    regWeight, frictionCoef);
    }
    Eigen::Matrix<float, 4, 3> XTrueTemp;
    XTrueTemp << -7.4467, -0.47097, -58.5855,
<<<<<<< HEAD
        0., 0., -0.,
        -0., 0., -0.,
        -6.88091, 0.41072, -59.71622;

    Eigen::Matrix<float, 3, 4> XTrue = XTrueTemp.transpose();
    cout << "XTrue: " << XTrue <<
         endl;
    cout << "Test qp_solver !" <<
         endl;
    for (
        int i = 0;
        i < X.
            rows();
        ++i) {
        for (
            int j = 0;
            j < X.
                cols();
            ++j) {
            EXPECT_NEAR(X(i, j), XTrue(i, j),
                        1E-1);
        }
    }
    double end_t = timer.GetTimeSinceReset();
    cout << "-----:" << (end_t - start_t) * 1000.f << " ms" <<
         endl;
}

int main(int argc, char **argv)
{
=======
            0., 0., -0.,
            -0., 0., -0.,
            -6.88091, 0.41072, -59.71622;

    Eigen::Matrix<float, 3, 4> XTrue = XTrueTemp.transpose();
    cout << "XTrue: " << XTrue << endl;
    cout << "Test qp_solver !" << endl;
    for (int i = 0; i < X.rows(); ++i) {
        for (int j = 0; j < X.cols(); ++j) {
            EXPECT_NEAR(X(i, j), XTrue(i, j), 1E-1);
        }
    }
    double end_t = timer.GetTimeSinceReset();
    cout << "-----:" << (end_t - start_t) * 1000.f  << " ms" << endl;
}

int main(int argc, char **argv) {
>>>>>>> origin/develop
    // 分析gtest程序的命令行参数
    ::testing::InitGoogleTest(&argc, argv);

    // 调用RUN_ALL_TESTS()运行所有测试用例
    // main函数返回RUN_ALL_TESTS()的运行结果
    return RUN_ALL_TESTS();
}
