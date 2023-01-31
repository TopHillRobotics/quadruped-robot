#include "controllers/desired_state_command.hpp"
using robotics::math::clip;

namespace Quadruped {
    DesiredStateCommand::DesiredStateCommand(ros::NodeHandle &nhIn, Robot* robotIn):nh(nhIn)
    {
        stateDes.setZero();
        stateCur.setZero();
        preStateDes.setZero();
        rootPosDes.setZero();
        filteredVel.setZero();
        filteredOmega.setZero();
        vDesInBodyFrame.setZero();
        wDesInBodyFrame.setZero();
        ddqDes.setZero();
        legJointq.setZero();
        legJointdq.setZero();
        JoyCtrlState = RC_MODE::BODY_UP;
        prevJoyCtrlState = RC_MODE::JOY_STAND;
        joycmdBodyHeight = robotIn->bodyHeight;
        joyCmdVz = joyCmdVx = joyCmdVy = 0;
        joyCmdYawRate = joyCmdRollRate = joyCmdPitchRate = 0;
        joyCtrlStateChangeRequest = false;
        joyCtrlOnRequest = true;
        rosCmdRequest = !joyCtrlOnRequest;
        joyCmdExit = false;
        bodyUp = 0;
        movementMode = 0;
        gamepadCommandSub = nh.subscribe(topicName, 10, &DesiredStateCommand::JoyCallback, this);
        std::cout << "init DesiredStateCommand finish\n" ;
    }

    void  DesiredStateCommand::JoyCallback(const sensor_msgs::Joy::ConstPtr &joy_msg)
    {
        joyCmdVz = 0; // joy_msg->axes[1] * JOY_CMD_BODY_HEIGHT_VEL;
	// A key
        if (joy_msg->buttons[0] == 1) {
            if (!joyCtrlOnRequest) {
                ROS_INFO("You have open joy control!!!\n");
                joyCtrlOnRequest = true;
            } else {
                ROS_INFO("You have turned off joy control!!!\n");
                joyCtrlOnRequest = false;    
            }
        }
        rosCmdRequest = ! joyCtrlOnRequest;
        if (joyCtrlOnRequest || rosCmdRequest) {
            // X key
            if (joy_msg->buttons[2] == 1) {
                ROS_INFO("You have change the gait !!!\n");
                if (movementMode == 0) {
                    movementMode = 1;
                }
                joyCtrlStateChangeRequest = true;
            }
            //  else {
            //     ROS_INFO("You have no state change request!!!\n");
            //     joyCtrlStateChangeRequest = false;    
            // }
            if (joyCtrlOnRequest) {
                // right updown
                joyCmdVx = joy_msg->axes[4] * MAX_VELX;
                // right horiz
                joyCmdVy = joy_msg->axes[3] * MAX_VELY;
                // left horiz
                joyCmdYawRate = joy_msg->axes[0] * MAX_YAWRATE;
                // cross button, left and right
                joyCmdRollRate = 0; // joy_msg->axes[6] * JOY_CMD_ROLL_MAX * (-1);
                // cross button, up and down
                joyCmdPitchRate = 0; // joy_msg->axes[7] * JOY_CMD_PITCH_MAX;
            }
            // B key
            if (joy_msg->buttons[1] == 1) {
                ROS_INFO("You have pressed the stop button!!!!\n");
                if (movementMode == 1) {
                    movementMode = 0;
                    joyCtrlStateChangeRequest = true;
                } else if (movementMode == 0) {
                    if (bodyUp >= 0) {
                        // movementMode = 1;
                        bodyUp = 0;
                        joyCtrlStateChangeRequest = true;
                    }
                }
            }

            // Y key
            if (joy_msg->buttons[3] == 1) {
                ROS_INFO("You have pressed the exit button!!!!\n");
                if (movementMode == 0 && bodyUp<=0) {
                    joyCmdExit = true;
                    joyCtrlStateChangeRequest = true;
                    joyCtrlOnRequest = false;
                }
            }
            // rb key
            if (joy_msg->buttons[5] == 1) {
                ROS_INFO("You have pressed the up/down button!!!!\n");
                if (movementMode == 0) {
                    if (bodyUp==0) {
                        bodyUp = 1;
                    } else {
                        bodyUp = -bodyUp;
                    }
                    joyCtrlStateChangeRequest = true;
                }
            }
        }
    }

    void DesiredStateCommand::Update()
    {
        count++;
        if (gaitSwitch) {
            joyCtrlStateChangeRequest = true;
            gaitSwitch = false; 
        }
        
	// std::cout << "JoyCtrlState = " << JoyCtrlState<<std::endl;
        
        stateDes.setZero();
        stateTrajDes.setZero();
        
        // if (joyCmdExit || movementMode==0) {
        //     movementMode = 0;
        // }

        // process joy cmd data to get desired height, velocity, yaw, etc
        // save the result into a1_ctrl_states
        // joycmdBodyHeight += joyCmdVz * dt;
        // if (joycmdBodyHeight >= BODY_HEIGHT_MAX) {
        //     joycmdBodyHeight = BODY_HEIGHT_MAX;
        // }
        // if (joycmdBodyHeight <= BODY_HEIGHT_MIN) {
        //     joycmdBodyHeight = BODY_HEIGHT_MIN;
        // }

        if (joyCtrlStateChangeRequest) {
            std::cout << "JoyCtrlState before = " << (int)JoyCtrlState <<", movementMode = " << movementMode<< std::endl;
            
            if (movementMode > 0) {
                if (JoyCtrlState == RC_MODE::HARD_CODE ||  JoyCtrlState == RC_MODE::BODY_UP) {
                    JoyCtrlState = RC_MODE::JOY_STAND;//prevJoyCtrlState;
                    std::cout << "JoyCtrlState = " << (int)JoyCtrlState << std::endl;
                    
                } else if (JoyCtrlState == RC_MODE::JOY_STAND) {
                    JoyCtrlState = RC_MODE::JOY_ADVANCED_TROT;
                } else {
                    JoyCtrlState = static_cast<RC_MODE>((JoyCtrlState+1) % 
                                                (RC_MODE::RC_MODE_ITEMS+1));
                    if (JoyCtrlState == RC_MODE::HARD_CODE) {
                        JoyCtrlState = static_cast<RC_MODE>(JoyCtrlState+1);
                    } else if (JoyCtrlState > RC_MODE::JOY_ADVANCED_TROT) {
                        JoyCtrlState = RC_MODE::JOY_TROT;
                    }
                }
            } else {
                if (JoyCtrlState <= 3) {
                    prevJoyCtrlState = JoyCtrlState;
                }
                if (joyCmdExit) {
                    JoyCtrlState = RC_MODE::EXIT;
                } else if(bodyUp == -1) {
                    JoyCtrlState = RC_MODE::BODY_DOWN;    
                } else  if (bodyUp == 1){
                    JoyCtrlState = RC_MODE::BODY_UP;
                } else {
                    JoyCtrlState = RC_MODE::JOY_STAND; // corresponed to LOCOMOTON_STAND
                }
            }
            // joyCtrlStateChangeRequest = false;  // erase this change request;
        }

        // root_lin_vel_d is in robot frame
        // root_lin_vel_d[0] = joyCmdVx;
        // root_lin_vel_d[1] = joyCmdVy;

        // root_ang_vel_d is in robot frame
        // root_euler_d[0] = joyCmdRollRate;
        // root_euler_d[1] = joyCmdPitchRate;
        // root_euler_d[2] += joyCmdYawRate * dt;
        
        // determine movement mode
        // if(JoyCtrlState == RC_MODE::JOY_STAND
        //         && prevJoyCtrlState != JoyCtrlState) {
        //     // leave walking mode
        //     // lock current position, should just happen for one instance
        //     movementMode = 0;
        //     rootPosDes.segment<2>(0) = root_pos.segment<2>(0);
        //     kp_linear(0) = kp_linear_lock_x;
        //     kp_linear(1) = kp_linear_lock_y;
        // } else {
        //     movement_mode = 0;
        // }

        // in walking mode, do position locking if no root_lin_vel_d, otherwise do not lock position
        // if (movement_mode == 1) {
        //     if (abs(filteredVel[0]) + abs(filteredVel[1]) > 0.03) {
                // has nonzero velocity, keep refreshing position target, but just xy
                // rootPosDes.segment<2>(0) = root_pos.segment<2>(0);
                // kp_linear.segment<2>(0).setZero();
            // } else {
                // kp_linear(0) = kp_linear_lock_x;
                // kp_linear(1) = kp_linear_lock_y;
        //     }
        // }

        if (JoyCtrlState == RC_MODE::JOY_STAND) {
            joyCmdVx = 0.;                        
            joyCmdVy = 0.;
            joyCmdVz = 0.;
            joyCmdYawRate = 0.;
            joyCmdRollRate = 0;
            joyCmdPitchRate = 0;
            filteredVel << joyCmdVx, joyCmdVy, joyCmdVz;
            filteredOmega << joyCmdRollRate, joyCmdPitchRate, joyCmdYawRate;
        } else if (JoyCtrlState == RC_MODE::JOY_TROT) {
            filteredVel = filteredVel * (1.0 - filterFactor) +  Vec3<float>(joyCmdVx, joyCmdVy, joyCmdVz) * filterFactor;
            filteredOmega = filteredOmega * (1.0 - filterFactor) + Vec3<float>(joyCmdRollRate, joyCmdPitchRate, joyCmdYawRate) * filterFactor;
        } else if (JoyCtrlState == RC_MODE::JOY_ADVANCED_TROT) {
            filteredVel = filteredVel * (1.0 - filterFactor) +  Vec3<float>(joyCmdVx, joyCmdVy, joyCmdVz) * filterFactor;
            filteredOmega = filteredOmega * (1.0 - filterFactor) + Vec3<float>(joyCmdRollRate, joyCmdPitchRate, joyCmdYawRate) * filterFactor;
        } else if (JoyCtrlState == RC_MODE::JOY_WALK) {
            filteredVel = filteredVel * (1.0 - filterFactor) +  Vec3<float>(joyCmdVx, joyCmdVy, joyCmdVz) * filterFactor;
            filteredOmega = filteredOmega * (1.0 - filterFactor) + Vec3<float>(joyCmdRollRate, joyCmdPitchRate, joyCmdYawRate) * filterFactor;
        } else if (JoyCtrlState == RC_MODE::HARD_CODE) { // No Remote Controller
            filteredVel = vDesInBodyFrame;
            filteredOmega = wDesInBodyFrame;
        } else {
            joyCmdVx = 0.;                        
            joyCmdVy = 0.;
            joyCmdVz = 0.;
            joyCmdYawRate = 0.;
            joyCmdRollRate = 0;
            joyCmdPitchRate = 0;
            filteredVel << joyCmdVx, joyCmdVy, joyCmdVz;
            filteredOmega << joyCmdRollRate, joyCmdPitchRate, joyCmdYawRate;
        }

        stateDes(6) = clip(filteredVel[0], MIN_VELX, MAX_VELX);            // forward linear velocity
        stateDes(7) = clip(filteredVel[1], MIN_VELY, MAX_VELY);            // lateral linear velocity
        stateDes(8) = 0.0;                                                     // vertical linear velocity
        stateDes(0) = dt * stateDes(6);                                        // X position
        stateDes(1) = dt * stateDes(7);                                        // Y position
        stateDes(2) = joycmdBodyHeight;                                     // Z position height
        stateDes(3) = 0.0;                                                     // Roll
        stateDes(4) = clip(filteredOmega[1]*dt, MIN_PITCH, MAX_PITCH);     // Pitch
        stateDes(5) = dt * stateDes(11);                                       // Yaw
        stateDes(9) = 0.0;                                                     // Roll rate
        stateDes(10) = 0.0;                                                    // Pitch rate
        stateDes(11) = clip(filteredOmega[2], MIN_YAWRATE, MAX_YAWRATE);   // Yaw turn rate
        if (stateDes(6) < -0.01)
            stateDes(2) = joycmdBodyHeight * 0.85; // 0.37 * 0.7 = 0.26, 0.28 * 0.7 = 0.196; 0.28;                                     // Z position height
        
    }

    void DesiredStateCommand::DesiredStateTrajectory(int N, Vec10<float> dtVec)
    {
        A.setIdentity();
        stateTrajDes.col(0) = stateDes;

        for (int k = 1; k < N; k++) {
            A(0, 6) = dtVec(k - 1);
            A(1, 7) = dtVec(k - 1);
            A(2, 8) = dtVec(k - 1);
            A(3, 9) = dtVec(k - 1);
            A(4, 10) = dtVec(k - 1);
            A(5, 11) = dtVec(k - 1);
            stateTrajDes.col(k) = A * stateTrajDes.col(k - 1);
            // for (int i = 0; i < 12; i++) {
                // std::cout << data.stateTrajDes(i, k) << " ";
            // }
            // std::cout << std::endl;
        }
    }

    void DesiredStateCommand::PrintRawInfo()
    {
        // Increment printing iteration
        printIter++;

        // Print at requested frequency
        if (printIter == printNum) {
            std::cout << "[DESIRED STATE COMMAND] Printing Raw Gamepad Info...\n";
            std::cout << "---------------------------------------------------------\n";
            std::cout << "RC MODE: " << JoyCtrlState << "\n";
            std::cout << "Right Handle: \n"
                        << " | X: " << joyCmdVx
                        << " | Y: " << joyCmdVy << "\n";
            std::cout << "LEFT Button: \n"
                        << " | YAW: " << joyCmdYawRate
                        << " | PITCH: " << joyCmdPitchRate << "\n";
            //   std::cout << "Left Bumper: " << gamepadCommand->leftBumper
            //             << " | Trigger Switch: " << gamepadCommand->leftTriggerButton
            //             << " | Trigger Value: " << gamepadCommand->leftTriggerAnalog
            //             << "\n";
            //   std::cout << "Right Bumper: " << gamepadCommand->rightBumper
            //             << " | Trigger Switch: " << gamepadCommand->rightTriggerButton
            //             << " | Trigger Value: " << gamepadCommand->rightTriggerAnalog
            //             << "\n\n";
            std::cout << std::endl;

            // Reset iteration counter
            printIter = 0;
        }
    }

    void DesiredStateCommand::PrintStateCommandInfo()
    {
        // Increment printing iteration
        printIter++;

        // Print at requested frequency
        if (printIter == printNum) {
            std::cout << "[DESIRED STATE COMMAND] Printing State Command Info...\n";
            std::cout << "---------------------------------------------------------\n";
            std::cout << "Position X: " << stateDes(0)
                      << " | Y: " << stateDes(1) << " | Z: " << stateDes(2)
                      << "\n";
            std::cout << "Orientation Roll: " << stateDes(3)
                      << " | Pitch: " << stateDes(4)
                      << " | Yaw: " << stateDes(5) << "\n";
            std::cout << "Velocity X: " << stateDes(6)
                      << " | Y: " << stateDes(7) << " | Z: " << stateDes(8)
                      << "\n";
            std::cout << "Angular Velocity X: " << stateDes(9)
                      << " | Y: " << stateDes(10) << " | Z: " << stateDes(11)
                      << "\n";
            std::cout << std::endl;
            std::cout << std::endl;

            printIter = 0;
        }
    }
} // namespace Quadruped
