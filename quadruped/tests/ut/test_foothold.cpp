//
// Created by xmc on 2021/11/2.
//
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include "exec/runtime.h"

using namespace std;
using std::cout;
using std::endl;
using namespace Quadruped;
using ::testing::_;
using ::testing::Return;

class TestSwingLegController : public testing::Test {
public:
    TestSwingLegController();

    ~TestSwingLegController() = default;

    void SetUp()
    {
       std::cout << "-------------------------start---------------------------" << std::endl;
    }

    void TearDown()
    {
       std::cout << "--------------------------end--------------------------" << std::endl;
    }

    static void SetUpTestCase()
    {
        std::cout << "--------------------------SetUpTestCase--------------------------" << std::endl;

    }

    UserParameters* userParameters;
                                        
    RaibertSwingLegController *swingLegController;
    Robot *quadruped;
    GaitGenerator* gaitGenerator;
    FootholdPlanner* footholdPlanner;
    StateEstimatorContainer<float>* stateEstimators;
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

TestSwingLegController::TestSwingLegController()
{
    string exepath = robotics::utils::GetExePath();
    std::size_t found = exepath.find("robots/");
    string homeDir = exepath.substr(0, found) + "robots/src/ascend-quadruped-cpp/";
    std::cout << homeDir << std::endl;
    userParameters = new UserParameters(homeDir+ "config/user_paramaters.yaml");
    // MockA1Robot quadruped(homeDir + "config/a1/a1_robot.yaml");
    int  n = 0;
    ros::init(n, NULL, "a1_sim");
    ros::NodeHandle nh;
    ros::NodeHandle privateNh("~");    
    quadruped = new A1Sim(nh, privateNh, homeDir + "config/a1_sim/a1_sim.yaml");
    quadruped->Step(Eigen::Matrix<float,5,12>::Zero(), HYBRID_MODE);
    quadruped->ReceiveObservation();
    std::cout << "BaseOrientation:\n" << quadruped->GetBaseOrientation().transpose() << std::endl;
    //
    
    gaitGenerator = new OpenloopGaitGenerator(quadruped, homeDir + "config/" + quadruped->robotName 
                                                         + "/openloop_gait_generator.yaml");
    stateEstimators = new StateEstimatorContainer<float>(quadruped, gaitGenerator, userParameters, 
                                                        "config/" + quadruped->robotName + "/terrain.yaml", 
                                                        homeDir); 
    ROS_INFO("LocomotionController Init Finished");
    DesiredStateCommand* desiredStateCommand = new DesiredStateCommand(nh, quadruped);
    LocomotionController* locomotionController = SetUpController(quadruped, gaitGenerator, desiredStateCommand, 
                                                                stateEstimators,userParameters, homeDir);                                                          
    swingLegController = locomotionController->GetSwingLegController();
    footholdPlanner = swingLegController->footholdPlanner;
    
}

TEST_F(TestSwingLegController, Test1)
{
    Visualization2D& vis = quadruped->stateDataFlow.visualizer;
    vis.SetLabelNames({"des y", "des z", "cur y", "cur z","swing status"});
    
    u8 legId = 2;

    quadruped->motorAngles[3*legId + 1] = 0.9;
    quadruped->motorAngles[3*legId + 2] = -1.8;
    quadruped->UpdateDataFlow();
    cout << quadruped->basePosition << endl;
    auto footPosBase = quadruped->GetFootPositionsInBaseFrame();
    cout << footPosBase << endl;
    cout << quadruped->hipOffset << endl;
    cout << quadruped->motorAngles << endl;
    cout << quadruped->defaultHipPosition << endl;
    
    footholdPlanner->ComputeHeuristicFootHold({legId});
    Vec3<float> footTargetPosition = footholdPlanner->desiredFootholds.col(legId);
    cout << footTargetPosition << endl;                
    
    Vec3<float> s(0.,  0.0, 0);
    Vec3<float> e(0.15,  0., 0.0);
    Vec3<float> footPositionInBaseFrame, footVelocityInBaseFrame, footAccInBaseFrame;
    SplineInfo splineInfo;
    splineInfo.splineType = SplineType::BSpline; //XYLinear_ZParabola
    swingLegController->swingFootTrajectories[legId] = SwingFootTrajectory(splineInfo, footTargetPosition, footTargetPosition, 1.f, 0.17);
    // swingLegController->swingFootTrajectories[legId].ResetFootTrajectory(1.f, footPosBase.col(legId), footTargetPosition, 0.17);
    swingLegController->swingFootTrajectories[legId].ResetFootTrajectory(1.f, s, e, 0.2);
    
    float segment[5] = {0,0.2, 0.3, 0.3,0.2};
    float left_edge = 0;
    for (int i=0; i< 4; ++i) {
        left_edge += segment[i];
        for (float phase = left_edge; phase < left_edge + segment[i+1]; phase+=0.005) {
            bool flag = swingLegController->swingFootTrajectories[legId].GenerateTrajectoryPoint(footPositionInBaseFrame,
                                                                            footVelocityInBaseFrame,
                                                                            footAccInBaseFrame,
                                                                            phase,
                                                                            false);
            /*
            Vec3<float> pos = robotics::math::cubicBezier<Vec3<float>, float>(s,e,phase);
            Vec3<float> vel = robotics::math::cubicBezierFirstDerivative<Vec3<float>, float>(s,e,phase);
            float zp, zv, za;

            if(phase < 0.5) {
                zp = robotics::math::cubicBezier<float>(s[2], s[2] + 0.15, phase * 2);
                zv = robotics::math::cubicBezierFirstDerivative<float>(s[2], s[2] + 0.15, phase * 2) * 2;// / swingTime;
                // za = robotics::math::cubicBezierSecondDerivative<T>(_p0[2], _p0[2] + _height, phase * 2) * 4 / (swingTime * swingTime);
            } else {
                zp = robotics::math::cubicBezier<float>(s[2] + 0.15, e[2], phase * 2 - 1);
                zv = robotics::math::cubicBezierFirstDerivative<float>(s[2] + 0.15, e[2], phase * 2 - 1) * 2;// / swingTime;
                // za = Interpolate::cubicBezierSecondDerivative<T>(_p0[2] + _height, _pf[2], phase * 2 - 1) * 4 / (swingTime * swingTime);
            }

            pos[2] = zp;
            vel[2] = zv;
            // _a[2] = za;
            */
            // vis.datax.push_back(phase);
            vis.datax.push_back(0*i/20.0+footPositionInBaseFrame[0]);
            // std::cout << "phase = " << phase <<" px = " << footPositionInBaseFrame[0] << std::endl; 
            vis.datay1.push_back(footVelocityInBaseFrame[0]);
            vis.datay2.push_back(footPositionInBaseFrame[1]);
            vis.datay3.push_back(footPositionInBaseFrame[2]);
            vis.datay4.push_back(footVelocityInBaseFrame[2]);
            vis.datay5.push_back(footPositionInBaseFrame[0]);
        }
    }
    std::map<std::string, std::string> map_;
    // map_["linestyle"] = "--";
    
    plt::scatter(vis.datax, vis.datay2, vis.datay3, 0.001, map_ );
    string zName ="z";
    string xName ="x";
    plt::set_zlabel(zName);
    plt::xlabel(xName);
    // plt::plot(vis.datax, vis.datay3); 
    // plt::plot(vis.datax, vis.datay4); 
    // plt::plot(vis.datax, vis.datay5); 
    plt::grid(1);
    plt::show();
    // vis.Show();

    // footPositionInBaseFrame = robotBaseR.transpose()* footPositionInBaseFrame;
                    
}



int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}