// #include <gmock/gmock.h>
// #include <gtest/gtest.h>
// #include "exec/runtime.h"
// #include "planner/pose_planner.h"
// #include "QuadProg++.hh"
// #include "Array.hh"


// using namespace std;
// using ::testing::_;
// using ::testing::Return;
// using namespace Quadruped;
// // 定义测试类FooTest
// class TestPosePlanner : public testing::Test
// {
// protected:
//     // Code here will be called immediately after the constructor (right before each test)
//     void SetUp()
//     {
//         cout << "test start" << endl;
//     }

//     // Code here will be called immediately after each test (right before the destructor)
//     void TearDown()
//     {
//         cout << "test end" << endl;
//     }

// public:
// };

// class MockqrGaitGenerator : public qrGaitGenerator
// {
// public:
//     // friend TEST_F(TestComAdjuster, Update);
//     MockqrGaitGenerator(qrRobot *robot, string configFilePath)
//         : qrGaitGenerator(robot, configFilePath)
//     {
//     }
// };

// class MockRobotA1 : public RobotA1
// {
// public:
//     MockRobotA1(std::string configFilePathIn) : RobotA1(configFilePathIn) {}
//     virtual ~MockRobotA1() = default;
//     // MOCK_METHOD((Eigen::Matrix<float, 3, 1>), GetBaseRollPitchYaw, (), (const));
//     // MOCK_METHOD0(GetBaseRollPitchYaw, Vec3<float>());
    
//     // MOCK_METHOD((Eigen::Matrix<float, 3, 4>), GetFootPositionsInBaseFrame, (), (override));
//     // MOCK_METHOD(Eigen::Matrix<float, 3, 4>, ReceiveObservation, (), (override));
// };

// class FakePosePlanner : public PosePlanner
// {
// public:
//     FakePosePlanner(qrRobot *robotIn, qrRobotEstimator *robotEstimatorIn)
//     : PosePlanner(robotIn, robotEstimatorIn)
//     {}

//     virtual ~FakePosePlanner() = default;

//     virtual std::tuple<Vec3<float>, Quat<float>> Update(float currentTime)
//     {
//         float tik = robot->GetTimeSinceReset();
//         rIB << 0.0,0.0,0.24;
//         rIB_ = ProjectV(rIB);
//         rBF << 0.18, 0.18, -0.18, -0.18,
//               -0.15, 0.15, -0.15, 0.15,
//               -0.24, -0.24, -0.24, -0.24; // = robot->GetFootPositionsInBaseFrame();
//         ToCounterClockOrder(rBF);
//         rIF = rBF.colwise() +  Vec3<float>(0,0,0.24);  //  rIB; //robot->state.GetFootPositionsInWorldFrame();
//         rBF = rIF.colwise() - rIB;
//         // rIB + rBF - rIF=0
//         cout << rIF << endl;
//         cout << rBH <<endl;
//         rBCOM = robot->comOffset; // rICOM = rIB + Phi(rBCOM);
//         rBCOM << 0., 0., 0.;
//         rICOMoffset = robotics::math::TransformVecByQuat(quat, rBCOM);
//         rICOMoffset << 0., 0., 0.;
//         rICOMoffset_ = ProjectV(rICOMoffset);
//         rSP_ << 0, 0, 0 ;
//         supportPolygonVertices.clear();
//         projectedSupportPolygonVertices.clear();
//         g.clear();
//         contactLegNumber = 0;
//         validContactPointId.clear();
//         for(int i=0; i <4 ;++i){ // i is counterwise order leg index (started from FR-leg).
//             if(contactK[i]){
//                 validContactPointId.insert(i);
//                 contactLegNumber++;
//                 Vec3<float> r = rIF.col(i); 
//                 supportPolygonVertices.push_back(r);
//                 rSP_ += r;    
//                 g.push_back(rIB + robotics::math::TransformVecByQuat<float>(quat, rBH.col(i)) - r);       
//             }
//         }
//         rSP_ = ProjectV(rSP_ / contactLegNumber);

//         for(int i=0; i<g.size(); ++i) {
//             cout << g[i].transpose() << endl;
//         }

//         N = contactLegNumber;
//         if (contactLegNumber==4) {
//             // check sp is or is not convex polygen; 
//             int invalidId=-1;
//             for (int sourceId=1; sourceId<=2; sourceId++) {
//                 int destId = (sourceId + 2) % 4;
//                 Vec3<float>& checkPostive = supportPolygonVertices[sourceId-1];
//                 Vec3<float>& checkNegative = supportPolygonVertices[sourceId+1];
//                 Vec3<float>& sourcePoint = supportPolygonVertices[sourceId];
//                 Vec3<float>& destPoint = supportPolygonVertices[destId];
//                 //
//                 if ((destPoint[0] - sourcePoint[0])*(checkPostive[1] - sourcePoint[1]) - 
//                         (destPoint[1] - sourcePoint[1])*(checkPostive[0] - sourcePoint[0]) > 0) {
//                     invalidId = sourceId -1;
//                     break;
//                 }
//                 //
//                 if ((destPoint[0] - sourcePoint[0])*(checkNegative[1] - sourcePoint[1]) - 
//                         (destPoint[1] - sourcePoint[1])*(checkNegative[0] - sourcePoint[0]) < 0) {
//                     invalidId = sourceId +1;
//                     break;
//                 }
//             }
//             // IF NOT CONVEX, COMPUTE ITS CLOSEURE.
//             if (invalidId>=0) {
//                 printf("invalidId = %d \n", invalidId);
//                 supportPolygonVertices.erase(supportPolygonVertices.begin()+invalidId);
//                 g.erase(g.begin()+invalidId);
//                 N--;
//                 validContactPointId.erase(invalidId);
//             } else {
//                 printf("is convex!\n");
//                 // if is convex polygen, then delete fourth-vertex, adjust first-vertex(FR leg point)
//                 // if (supportPolygonVertices.size()==4) {
//                     // Vec3<float> newPoint = (supportPolygonVertices[0] + supportPolygonVertices[3])/2.0;
//                     // cout << "new point" << newPoint <<endl;
//                     // supportPolygonVertices[0] = newPoint;
//                     // supportPolygonVertices.pop_back();
//                     // g.pop_back();
//                     // contactLegNumber--;
//             } 
//         }
//         for(auto sp: supportPolygonVertices){
//             cout << "sp: " << sp.transpose() <<endl;
//         }

//         for(int i=0; i< N; ++i){
//             Vec3<float>& r = supportPolygonVertices[i];
//             projectedSupportPolygonVertices.push_back(ProjectV(r));
//             // g.back().normalize();
//         }
//         Lambda.conservativeResize(3*N, Eigen::NoChange);
//         cout << rSP_ << endl;  
//         cout << "rICOMoffset = " << rICOMoffset << endl;
//         for(int i : validContactPointId) {
//             printf( "valid id = %d\n",i);
//         }
//         // printf("solve the min L, SQP.\n");
//         for(int loop=0; loop < 30; ++loop) {
//             printf("[Loop %d]\n", loop);
//             float f = ComputeF();
//             cout << "f = " << f << endl;
//             Eigen::MatrixXf GValue = ComputeG();
//             cout << "GValue = "<<GValue.transpose() << endl;
//             Mat6<float> hessF = ComputeHessianF();
//             std::vector<Mat6<float>> hessG = ComputeHessianG();
//             Vec6<float> gradientF = ComputeGradientF();
//             Eigen::MatrixXf gradientG = ComputeGradientG();

//             Mat6<float> hessGSum = Mat6<float>::Zero();
//             if (hessG.size() != Lambda.size()) { // =3*N
//                 throw std::domain_error("The size of hessG and Lambda does not match!");
//             }
//             for (int i=0; i<hessG.size(); ++i) {
//                 hessGSum += Lambda(i) * hessG[i];
//             }
//             // solve the min L, SQP.
//             auto res = QpSolver(hessF, hessGSum, gradientF, gradientG, GValue);
//             Vec6<float> p = std::get<0>(res);
//             Eigen::MatrixXf u = std::get<1>(res);
            
//             rIB += p.head(3);
//             so3Phi = p.tail(3);
//             Quat<float> dQuat = robotics::math::so3ToQuat(so3Phi);
//             quat = robotics::math::ConcatenationTwoQuats(dQuat, quat);
//             cout << "rIB = "<< rIB << " quat" << quat.transpose() << endl;
//             rIB_ = ProjectV(rIB);
//             rBF = rIF.colwise() - rIB;
//             rICOMoffset = robotics::math::TransformVecByQuat(quat, rBCOM);
//             rICOMoffset_ = ProjectV(rICOMoffset);
//             g.clear();
//             for(int i : validContactPointId) { // i is counterwise order leg index (started from FR-leg).
//                 Vec3<float> r = rIF.col(i); 
//                 g.push_back(rIB + robotics::math::TransformVecByQuat<float>(quat, rBH.col(i)) - r);
//             }
        
//             // todo : update Lambda
//             // Eigen::MatrixXf inq = GValue+gradientG*p;
//             // cout << "c+Ad = " << inq <<endl;
             
//             // Eigen::MatrixXf A = gradientG.transpose();
//             // Eigen::Matrix<float, 6, 1> b = -gradientF - (hessF - hessGSum)*p;
//             // A = A(Eigen::placeholders::all, cols);
//             // Lambda = A.completeOrthogonalDecomposition().solve(b);
//             // Eigen::MatrixXf Lambda_ = A.colPivHouseholderQr().solve(b);
            
//             for(int i=0; i<3*N; ++i) {
//                 Lambda(i, 0) = u(i, 0);
//             }
//             cout << "Lambda = " << Lambda.transpose() << endl;
            
//             if (Lambda.size()!=3*N) {
//                 throw std::domain_error("Lambda.size()!=3*N\n");
//             }
//         }

//         float tok = robot->GetTimeSinceReset();
//         printf("time = %f ms", (tok-tik)*1000);
//         // std::tuple<Vec3<float>, Quat<float>> X{rIB, quat};
//         return {rIB, quat};
//     }
// };


// TEST_F(TestPosePlanner, MATH)
// {
//     Quat<float> quat;
//     quat << 0.8,0.6,0.4,0.2;
//     quat.normalize();
//     Vec3<float> v = {1,2,3};
//     Vec3<float> res1 = robotics::math::TransformVecByQuat(quat, v);
//     Mat3<float> mat = robotics::math::quaternionToRotationMatrix(quat).transpose();
//     Vec3<float> res2 = mat*v;
//     for(int i=0; i<3; ++i) {
//         EXPECT_NEAR(res1[i], res2[i], 1E-3);
//     }

//     Quat<float> quat2;
//     quat2 << 0.3,0.4,0.6,0.8;
//     quat2.normalize();
//     Mat3<float> mat2 = robotics::math::quaternionToRotationMatrix(quat2).transpose();
//     Quat<float> quat3 = robotics::math::ConcatenationTwoQuats(quat2, quat);
//     Quat<float> quat3_ = robotics::math::rotationMatrixToQuaternion((mat2*mat).transpose());
//     Quat<float> quat3_3 = robotics::math::quatProduct(quat2, quat);
//     for(int i=0; i < 4; ++i){
//         EXPECT_NEAR(quat3[i], quat3_[i], 1E-3);
//         EXPECT_NEAR(quat3_[i], quat3_3[i], 1E-3);
//     }

//     Vec3<float> so3 = robotics::math::LogQuat(quat);
//     Quat<float> quat_ = robotics::math::so3ToQuat(so3);
//     for(int i=0; i < 4; ++i){
//         EXPECT_NEAR(quat[i], quat_[i], 1E-3);
//     }
// }


// TEST_F(TestPosePlanner, Update)
// {
//     std::string homeDir = GetHomeDir();
//     std::string robotName = "a1";
//     YAML::Node mainConfig = YAML::LoadFile(homeDir + "config/" + robotName + "/main.yaml");
//     int twistMode = mainConfig["speed_update_mode"].as<int>();
//     vector<float> linearVel = mainConfig["const_twist"]["linear"].as<vector<float >>();

//     desiredSpeed = Eigen::MatrixXf::Map(&linearVel[0], 3, 1);
//     desiredTwistingSpeed = mainConfig["const_twist"]["angular"].as<float>();
//     int n=0;
//     ros::init(n, nullptr, "ascend_quadruped_robot");
//     ros::NodeHandle nh;
//     ros::NodeHandle privateNh("~");
//     ros::Rate loop_rate(1000);
//     std::cout << "---------ROS node init finished---------" << std::endl;
    
//     MockRobotA1 quadruped(homeDir + "config/a1/a1_robot.yaml");
//     quadruped.ReceiveObservation();
//     std::cout << "BaseOrientation:\n"
//               << quadruped.GetBaseOrientation().transpose() << std::endl;
//     // MockRobotVelocityEstimator *stateEstimator = new MockRobotVelocityEstimator(quadruped, 0.1f, 0.1f, 0.1f, 120);
//     // cout << "init stateEstimator" << endl;
//     qrLocomotionController *locomotionController = setUpController(&quadruped, homeDir, robotName);
//     qrRobotEstimator* robotEstimator = locomotionController->GetRobotEstimator();
//     FakePosePlanner posePlanner(&quadruped, robotEstimator);
    
//     // 定义函数行为
//     Eigen::Matrix<float, 3, 1> res;
//     res << 0, 0, 0;

//     // EXPECT_CALL(quadruped, GetBaseRollPitchYaw())
//     //     // .Times(1) // 执行一次
//     //     .WillOnce(Return(res))        // 第一次执行的返回值
//     //     .WillRepeatedly(Return(res)); // 后续执行的返回值

//     // Eigen::Matrix<float, 3, 4> res;
//     // res << 0.169, 0.164, -0.205, -0.199,
//     //     -0.15, 0.126, -0.138, 0.095,
//     //     -0.229, -0.165, -0.165, -0.269;
//     // // 定义函数行为
//     // EXPECT_CALL(*quadruped, GetFootPositionsInBaseFrame())
//     //     // .Times(1) // 执行一次
//     //     .WillOnce(Return(res))        // 第一次执行的返回值
//     //     .WillRepeatedly(Return(res)); // 后续执行的返回值


//     cout << ">>>>>>>>>>>>>>>> start google test >>>>>>>>>>>>>>>>>" << endl;
//     // Solution 1: devired from tested class.
//     // Eigen::Matrix<float, 3, 1> fakepInBaseFrame = fakecomAdjuster->Update(0);
//     // Solution 2: using gmock to imitate the other classes's behaviors.
//     // Tips: use FRIEND_TEST(TestComAdjuster, Update); statement make it accessable to private members.
//     cout << posePlanner.Lambda << endl;
//     posePlanner.Update(0);
//     // EXPECT_NEAR(pInBaseFrame[0], -0.019, 1E-3);
//     // EXPECT_NEAR(pInBaseFrame[1], -0.014, 1E-3);
//     // EXPECT_NEAR(pInBaseFrame[2], -0.196, 1E-3);
// }


// void qpTest() 
// {
//     quadprogpp::Matrix<double> GG(0., 5,5);
//     Eigen::Matrix<double,5,1> aa_;
//     quadprogpp::Vector<double> aa(5);
//     for (int i = 0; i<5; i++) {
//         GG[i][i] = 1.;
//     }
//     aa_ << -0.73727161, -0.75526241, -0.04741426, 0.11260887, 0.11260887;
//     for(int i=0; i<5; ++i){
//         aa[i] = aa_[i];
//     }
//     quadprogpp::Matrix<double> CECE(5,3);
//     Eigen::Matrix<double,5,3> CECE_;
//     CECE_ << 3.6, 0., -9.72,
//             -3.4, -1.9, -8.67,
//             -3.8, -1.7, 0.,
//             1.6, -4., 0.,
//             1.6, -4., 0.;
//     for (int i = 0; i < 5; i++)
//     {
//         for(int j=0; j < 3; ++j){
//             CECE[i][j] = CECE_(i,j);
//         }
//     }
    
//     quadprogpp::Vector<double> ee(3);
//     Vec3<double> ee_;
//     ee_ << -1.02, -0.03, -0.081;
//     for(int i=0;i<3;++i){
//         ee[i] = ee_[i];
//     }
//     quadprogpp::Matrix<double> CICI(5,0);
//     quadprogpp::Vector<double> bb(0);
//     quadprogpp::Vector<double> x(5);
//     quadprogpp::Vector<double> u(3);

//     quadprogpp::solve_quadprog_test(GG, aa, CECE, ee, CICI, bb, x, u);
//     std::cout << x << std::endl;
//     std::cout << u << std::endl;
    
// }


// TEST_F(TestPosePlanner, test)
// {
//     qpTest();
//     Vec3<float> rpy(0.2,0,0); // roll=11.5
//     Quat<float> qq = robotics::math::rpyToQuat(rpy);
//     // cout << qq<<endl; // 0.995, 0.0998,0,0
// }

// int main(int argc, char **argv)
// {
//     // 分析gtest程序的命令行参数
//     ::testing::InitGoogleTest(&argc, argv);
//     // 调用RUN_ALL_TESTS()运行所有测试用例
//     // main函数返回RUN_ALL_TESTS()的运行结果
//     return RUN_ALL_TESTS();
// }