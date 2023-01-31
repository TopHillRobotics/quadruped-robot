#include "exec/runtime.h"
#include "robots/robot_a1_sim.h"
#include "ros/control2gazebo_msg.h"

#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/SetModelConfiguration.h>
#include <gazebo_msgs/SetPhysicsProperties.h>
#include <std_srvs/Empty.h>

#include <controller_manager/controller_manager.h>
#include <controller_manager_msgs/ListControllerTypes.h>
#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/ReloadControllerLibraries.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/UnloadController.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/ControllerState.h>

using namespace std;
using namespace Quadruped;

bool startControllers(ros::NodeHandle &n, std::string serviceName, std::vector<std::string> &controllersToStart)
{
    ros::ServiceClient switchController = n.serviceClient<controller_manager_msgs::SwitchController>(serviceName);
    controller_manager_msgs::SwitchController switchControllerMsg;
    switchControllerMsg.request.start_controllers = controllersToStart;
    switchControllerMsg.request.strictness = switchControllerMsg.request.STRICT;
    ros::service::waitForService(serviceName, -1);
    switchController.call(switchControllerMsg);

    if (switchControllerMsg.response.ok){
        ROS_INFO_STREAM("Controller start correctly");
        return true;
    } else {
        ROS_ERROR_STREAM("Error occured trying to start controller");
        return false;
    }
}

bool stopControllers(ros::NodeHandle &n, std::string serviceName, std::vector<std::string> &controllers_to_stop)
{
    ros::ServiceClient switchController = n.serviceClient<controller_manager_msgs::SwitchController>(serviceName);
    controller_manager_msgs::SwitchController switchControllerMsg;
    switchControllerMsg.request.stop_controllers = controllers_to_stop;
    switchControllerMsg.request.strictness = switchControllerMsg.request.STRICT;
    switchControllerMsg.request.start_asap = false;
    ros::service::waitForService(serviceName, -1);
    switchController.call(switchControllerMsg);
    if (switchControllerMsg.response.ok){
        ROS_INFO_STREAM("Controller stop correctly");
        return true;
    } else {
        ROS_ERROR_STREAM("Error occured trying to stop controller");
        return false;
    }
}

bool ResetRobotBySystem() {
    int flag = system("rosservice call gazebo/delete_model '{model_name: a1_gazebo}'");
    ROS_INFO("delete statu: %d", flag);    
    int statu0 = system("rosrun gazebo_ros spawn_model -urdf -z 0.6 -model a1_gazebo -param robot_description -unpause");
    ROS_INFO("spawn model statu: %d", statu0);
    int statu1 = system("rosrun controller_manager spawner __ns:=/a1_gazebo joint_state_controller "\
          "FL_hip_controller FL_thigh_controller FL_calf_controller "\
          "FR_hip_controller FR_thigh_controller FR_calf_controller "\
          "RL_hip_controller RL_thigh_controller RL_calf_controller "\
          "RR_hip_controller RR_thigh_controller RR_calf_controller &");
    ROS_INFO("controller statu: %d", statu1);
    sleep(1);
    return true;
}

bool ResetRobot(ros::ServiceClient &modelStateClient, ros::ServiceClient &jointStateClient)
{
    gazebo_msgs::ModelState modelState;
    gazebo_msgs::SetModelState setmodelstate;
    gazebo_msgs::SetModelConfiguration setjointstate;
    
    modelState.model_name = "a1_gazebo";
    modelState.reference_frame = "world";

    geometry_msgs::Twist model_twist;
    geometry_msgs::Pose model_pose;
    model_twist.linear.x = 0.0;
    model_twist.linear.y = 0.0;
    model_twist.linear.z = 0.0;
    model_twist.angular.x = 0.0;
    model_twist.angular.y = 0.0;
    model_twist.angular.z = 0.0;
    model_pose.position.x =0.; // 4.5
    model_pose.position.y =0;
    model_pose.position.z =0.3; // 1.3
    model_pose.orientation.w = 1;
    model_pose.orientation.x = 0; 
    model_pose.orientation.y = 0; 
    model_pose.orientation.z = 0; 
    
    modelState.twist = model_twist;
    modelState.pose = model_pose;

    setmodelstate.request.model_state = modelState;
    setjointstate.request.model_name = "a1_gazebo";
    setjointstate.request.urdf_param_name = "robot_description";
    setjointstate.request.joint_names = {"FR_hip_joint","FR_thigh_joint", "FR_calf_joint",
                                        "FL_hip_joint","FL_thigh_joint", "FL_calf_joint",
                                        "RR_hip_joint","RR_thigh_joint", "RR_calf_joint",
                                        "RL_hip_joint","RL_thigh_joint", "RL_calf_joint"};
    double hip_angle = 0.3; 
    double thigh_angle = 1.1;
    double calf_angle = -2.2;
    
    setjointstate.request.joint_positions = {-hip_angle,thigh_angle,calf_angle,
                                            hip_angle,thigh_angle,calf_angle,
                                            -hip_angle,thigh_angle,calf_angle,
                                            hip_angle,thigh_angle,calf_angle};
    
    ros::service::waitForService("/gazebo/set_model_state", -1);
    modelStateClient.call(setmodelstate);
    ros::service::waitForService("/gazebo/set_model_configuration", -1);
    if (jointStateClient.call(setjointstate))
    {
        ROS_INFO("BRILLIANT!!!");
        return true;
    } else
    {
        ROS_ERROR("Failed to set joints");
        return false;
    }
}

void GetComPositionInWorldFrame(Robot* quadruped, ros::ServiceClient& baseStateClient)
{
    gazebo_msgs::GetLinkState gls_request;
    if (baseStateClient.exists()) { 
        gls_request.request.link_name = std::string("a1_gazebo::base");
        gls_request.request.reference_frame=std::string("world"); 
        // ros::service::waitForService("/gazebo/get_link_state", -1);
        baseStateClient.call(gls_request);
        if (!gls_request.response.success) {
                ROS_INFO("Get Gazebo link state not success!\n");      
        }
    } else { 
        ROS_INFO("Get Gazebo link state goes wrong!\n"); 
    }
    
    const auto & pose_ = gls_request.response.link_state.pose; 
    Vec3<double> posIn = {pose_.position.x, pose_.position.y, pose_.position.z};
    Quat<double> OrientationIn = {pose_.orientation.w,pose_.orientation.x,pose_.orientation.y,pose_.orientation.z};
    quadruped->gazeboBasePosition = posIn.cast<float>(); 
    quadruped->gazeboBaseOrientation = OrientationIn.cast<float>();
    quadruped->gazeboFootPositionInWorldFrame = quadruped->GetFootPositionsInWorldFrame(true, posIn.cast<float>(), OrientationIn.cast<float>());
    // cout << "trueFootPositionInWorldFrame=" << quadruped->gazeboFootPositionInWorldFrame <<endl;
    // std::cout << "gazeboBasePos = " << quadruped->gazeboBasePosition << std::endl;
}


int main(int argc, char **argv)
{    
    std::string homeDir = GetHomeDir();
    std::string robotName = "a1_sim";
    YAML::Node mainConfig = YAML::LoadFile(homeDir + "config/" + robotName + "/main.yaml");
    int twistMode = mainConfig["speed_update_mode"].as<int>();
    vector<float> linearVel = mainConfig["const_twist"]["linear"].as<vector<float >>();
    desiredSpeed = Eigen::MatrixXf::Map(&linearVel[0], 3, 1);
    desiredTwistingSpeed = mainConfig["const_twist"]["angular"].as<float>();
    
    std::vector<std::string> controllerList = {"joint_state_controller", "FL_hip_controller", "FL_thigh_controller",
                                               "FL_calf_controller", "FR_hip_controller", "FR_thigh_controller",
                                               "FR_calf_controller", "RL_hip_controller", "RL_thigh_controller",
                                               "RL_calf_controller", "RR_hip_controller", "RR_thigh_controller",
                                               "RR_calf_controller"};
    
    ros::init(argc, argv, "a1_sim");
    ros::NodeHandle nh;
    ros::NodeHandle privateNh("~");
    
    stopControllers(nh, "/a1_gazebo/controller_manager/switch_controller", controllerList);
    ros::ServiceClient modelStateClient = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    ros::ServiceClient jointStateClient = nh.serviceClient<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration");

    bool flag = ResetRobot(modelStateClient, jointStateClient);
    ROS_INFO("Reset the Robot pose");
    startControllers(nh, "/a1_gazebo/controller_manager/switch_controller", controllerList);
    
    ros::AsyncSpinner spinner(1); // one threads
    spinner.start();
    std::cout << "---------ROS node init finished---------" << std::endl;

    Robot *quadruped = new A1Sim(nh, privateNh, homeDir + "config/a1_sim/a1_sim.yaml");
    quadruped->Step(Eigen::Matrix<float,5,12>::Zero(), HYBRID_MODE);
    quadruped->ReceiveObservation();
    std::cout << "BaseOrientation:\n" << quadruped->GetBaseOrientation().transpose() << std::endl;

    //    Action::SitDown(quadruped, 3, 0.001);
    Action::StandUp(quadruped, 3.f, 5.f, 0.001);
    // Action::KeepStand(quadruped, 10,  0.001);
    // stopControllers(nh, "/a1_gazebo/controller_manager/switch_controller", controllerList);
    LocomotionController *locomotionController = SetUpController(quadruped, homeDir, robotName);
    ROS_INFO("LocomotionController Init Finished");
    locomotionController->Reset();
    ROS_INFO("LocomotionController Reset Finished");

    // ros module init
    ros::ServiceClient baseStateClient = nh.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
    RobotOdometryEstimator *legOdom = new RobotOdometryEstimator(quadruped, locomotionController, nh);
    CmdVelReceiver *cmdVelReceiver = new CmdVelReceiver(nh, privateNh);
    SLAMPoseReceiver *slamPoseReceiver = new SLAMPoseReceiver(nh, privateNh);
    SwitchModeReceiver *switchModeReceiver = new SwitchModeReceiver(nh, privateNh);
    Controller2GazeboMsg *controller2gazeboMsg = new Controller2GazeboMsg(quadruped, locomotionController, nh);
    // SwitchModeReceiver *switchModeReceiver = new SwitchModeReceiver(nh, privateNh);
    ROS_INFO("ROS Modules Init Finished");
    
    ROS_INFO("TimeSinceReset: %f", quadruped->GetTimeSinceReset());
    UpdateControllerParams(locomotionController, {0., 0., 0.}, 0.);
    GetComPositionInWorldFrame(quadruped, baseStateClient);

    float startTime = quadruped->GetTimeSinceReset();
    float currentTime = startTime;
    float startTimeWall = startTime;
    Eigen::Matrix<float,12,1> angles = quadruped->GetMotorAngles();
    Eigen::Matrix<float,12,1> stancemotorAngles = angles;
    Eigen::Matrix<float,12,1> motorAnglesAfterKeepStand = angles;
    for (int legId =0; legId<4; ++legId) {
        motorAnglesAfterKeepStand[3*legId + 0] = 0.;
        motorAnglesAfterKeepStand[3*legId + 1] = 1.2;
        motorAnglesAfterKeepStand[3*legId + 2] = -2.4;
    }
    Eigen::Matrix<float,12,1> motorAngles;
    Eigen::Matrix<float, 12, 1> kps, kds;
    kps = quadruped->GetMotorPositionGains();
    kds = quadruped->GetMotorVelocityGains();
    GaitGenerator *gaitGenerator = locomotionController->GetGaitGenerator();
    GroundSurfaceEstimator* groundEstimator = locomotionController->GetGroundEstimator();
    RobotEstimator* robotEstimator = locomotionController->GetRobotEstimator();
    ContactDetection contactDetection(quadruped,gaitGenerator, groundEstimator);
    
    locomotionController->Update();
    auto swingLegController = locomotionController->GetSwingLegController();
    swingLegController->Reset(0);
    
    ROS_INFO("start control loop....");
    int switchMode;
    int count = 0;
    float avgCost=0;
    const int n = 10000;
    std::vector<float> datax, datay1, datay2,datay3, datay4, datay5;
    
    while (ros::ok() && currentTime - startTime < MAX_TIME_SECONDS) {
        startTimeWall = quadruped->GetTimeSinceReset();
        // switchMode = switchModeReceiver->GetSwitchMode();

        if (twistMode == TwistMode::ROS) {
            desiredSpeed = cmdVelReceiver->GetLinearVelocity();
            desiredTwistingSpeed = cmdVelReceiver->GetAngularVelocity(); 
        }
        // if (switchMode != 2 && quadruped->controlParams["mode"] != switchMode) {
        //     ROS_INFO_STREAM("switch mode from " << quadruped->controlParams["mode"] << " to " << switchMode);
        //     SwitchMode<A1Sim>(quadruped, locomotionController, desiredSpeed, desiredTwistingSpeed, switchMode, startTimeWall);
        // }
        UpdateControllerParams(locomotionController,
                                desiredSpeed,
                                desiredTwistingSpeed); // ros velocity

        if (count % 3 ==0) {
            GetComPositionInWorldFrame(quadruped, baseStateClient);
            Vec3<float> robotComRpyRate = quadruped->GetBaseRollPitchYawRate();
            Vec4<float> footForces = quadruped->GetFootForce();
            Vec3<float> rpy = quadruped->GetBaseRollPitchYaw();
            Vec3<float> robotComVelocity = robotEstimator->GetEstimatedVelocity();  // base frame
        
            datay1.push_back(footForces[0]/10.0);
            datay2.push_back(footForces[1]/10.0);
            datay3.push_back(footForces[2]/10.0);
            datay4.push_back(footForces[3]/10.0);
            datay5.push_back(robotComVelocity[1]*10);
            datax.push_back(count);
        }
      
        locomotionController->Update();
        auto [hybridAction, qpSol] = locomotionController->GetAction();
        contactDetection.Update(0);
        /*
        Eigen::Matrix<float,5,12> hybridActionMat = MotorCommand::convertToMatix(hybridAction);
        if (currentTime - startTime < 1) {
            float blendRatio = (currentTime - startTime) / 1;
            if (blendRatio>1) {
                blendRatio =1;
            }
            motorAngles = blendRatio * motorAnglesAfterKeepStand + (1 - blendRatio) * angles;
            Eigen::Matrix<float,12,1> aa=quadruped->GetMotorAngles();
            stancemotorAngles = blendRatio * hybridActionMat.row(0).transpose() + (1.0f-blendRatio) * aa;
            
        } else {
            motorAngles = motorAnglesAfterKeepStand;
            stancemotorAngles =  hybridActionMat.row(0);
        }
        
        for (int legId=0; legId<4; ++legId) {
            if(gaitGenerator->desiredLegState[legId]==LegState::USERDEFINED_SWING ||
                 gaitGenerator->desiredLegState[legId]==LegState::SWING ) {
                for (int j=0; j<3; ++j) {
                    hybridAction[3*legId + j].p =  motorAngles[3*legId + j];// angles[3*legId + j];//
                    hybridAction[3*legId + j].Kp = kps[3*legId + j];
                    hybridAction[3*legId + j].Kd = kds[3*legId + j];
                    hybridAction[3*legId + j].tua = 0.0;
                    
                }
            } else {  // stand
                for (int j=0; j<3; ++j) {
                    // ;
                    hybridAction[3*legId + j].p = stancemotorAngles[3*legId + j];
                }
            }
        }
        */
        /*
        if(currentTime - startTime >= 1.f) {
            float blendRatio = (currentTime - startTime-1.) / 1.0;
            if (blendRatio>1) {
                blendRatio =1;
            }
            motorAngles = blendRatio * motorAnglesAfterKeepStand + (1 - blendRatio) * angles;
            for (int legId=0; legId<4; ++legId) {
                if(gaitGenerator->desiredLegState[legId]==LegState::USERDEFINED_SWING ||
                    gaitGenerator->desiredLegState[legId]==LegState::SWING ) {
                    for (int j=0; j<3; ++j) {
                        hybridAction[3*legId + j].p =  motorAngles[3*legId + j];
                        hybridAction[3*legId + j].Kp = kps[3*legId + j];
                        hybridAction[3*legId + j].Kd = kds[3*legId + j];
                        hybridAction[3*legId + j].tua = 0.0;
                        
                    }
                    gaitGenerator->normalizedPhase[legId] = 1.0;

                }
            }  
        } else {
            angles = quadruped->GetMotorAngles();
            gaitGenerator->normalizedPhase[2] = (currentTime - startTime)*0.5;
        }
        */
        
        // cout << "command: " << MotorCommand::convertToMatix(hybridAction) << endl;
        quadruped->Step(MotorCommand::convertToMatix(hybridAction), HYBRID_MODE);

        //ros
        legOdom->PublishOdometry();
        
        currentTime = quadruped->GetTimeSinceReset();
        avgCost += (currentTime - startTimeWall);    
        if ((count+1) % 1000==0) {
            printf("avg time cost = %f [ms]\n", avgCost);
            avgCost = 0.;
        }
        if (quadruped->basePosition[2] < 0.10
            || quadruped->stateDataFlow.heightInControlFrame < 0.05
            || quadruped->basePosition[2]>0.40 || abs(quadruped->baseRollPitchYaw[0]) > 0.2
        ) {
            ROS_ERROR("The dog is going down, main function exit.");
            cout << "base pos:" << quadruped->basePosition << endl;
            cout << "base rpy:" << quadruped->GetBaseRollPitchYaw() << endl;
            // exit(0);
            break;
        }
        if (count > 15000) {
            printf("[268]: count is %d \n", count);
            break;
        }
        while (quadruped->GetTimeSinceReset() - startTimeWall < quadruped->timeStep) {}
    //    std::cout << ros::Time::now() << endl;
        // std::cout << "angles = "<<quadruped->GetMotorAngles().transpose() << std::endl;
        // int c=0;
        // c = robotics::utils::getch1();
        // if (c > 0) {
		//     ROS_INFO("[INPUTE]%c", c);
        //     if (c=='n') {
        //         locomotionController->ForwardOne();
        //     }
        // }

        // Quat<float> quat = quadruped->GetBaseOrientation();
        // Vec3<float> rrr = quadruped->GetBaseRollPitchYaw();
        // rrr = rrr / 3.1415 * 180;
        // Vec3<float> rpy = robotics::math::quatToRPY(quat);
        // rpy = rpy / 3.1415 * 180;
        // if((rrr - rpy).norm() > 3 ) {
        //     exit(0);
        // }
        // datay1.push_back(quadruped->gazeboFootPositionInWorldFrame(0,0));
        // datay2.push_back(quadruped->gazeboFootPositionInWorldFrame(0,0));
        // datay3.push_back(quadruped->gazeboBasePosition[0]);
        // datay4.push_back(quadruped->gazeboBasePosition[2]);
        // datax.push_back(count);

        count++;
    }
    stopControllers(nh, "/a1_gazebo/controller_manager/switch_controller", controllerList);
    bool draw = true;
    if (draw) {
        plt::figure();
        // plt::plot(datax,datay);
        std::map<std::string,std::string> map_;
        map_["cmap"] = "viridis";
        map_["alpha"] = "0.5";
        // plt::scatter(datax,datay1, (1.0), {{"label", "T_X"}});
        // plt::scatter(datax,datay2, (1.0), {{"label", "T_Y"}});
        // plt::scatter(datax,datay3, (1.0), {{"label", "E_X"}});
        // plt::scatter(datax,datay4, (1.0), {{"label", "E_Y"}});

        plt::plot(datax,datay1, {{"label", "T_X"}, {"linestyle","-."}});
        plt::plot(datax,datay2, {{"label", "T_Y"}, {"linestyle","-."}});
        plt::plot(datax,datay3, {{"label", "E_X"}, {"linestyle","-."}});
        plt::plot(datax,datay4, {{"label", "E_Y"}, {"linestyle","-."}});
        plt::plot(datax,datay5, {{"label", "V_Y"}, {"linestyle","--"}});
        
        plt::grid(true);
        plt::legend();
        plt::show();
    }
    ros::shutdown();
    return 0;
}
