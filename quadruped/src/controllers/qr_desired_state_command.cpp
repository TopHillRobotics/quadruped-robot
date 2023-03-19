// The MIT License

// Copyright (c) 2022
// Robot Motion and Vision Laboratory at East China Normal University
// Contact: tophill.robotics@gmail.com

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "controllers/qr_desired_state_command.hpp"


using robotics::math::clip;

Quadruped::qrDesiredStateCommand::qrDesiredStateCommand(ros::NodeHandle &nhIn, qrRobot* robotIn):nh(nhIn)
{
    stateDes.setZero();
    stateCur.setZero();
    preStateDes.setZero();

    filteredVel.setZero();
    filteredOmega.setZero();

    vDesInBodyFrame.setZero();
    wDesInBodyFrame.setZero();

    ddqDes.setZero();
    legJointq.setZero();
    legJointdq.setZero();

    joyCtrlState = RC_MODE::BODY_UP;
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

    gamepadCommandSub = nh.subscribe(topicName, 10, &qrDesiredStateCommand::JoyCallback, this);

    printf("[Desired State Command] init finish...\n");
}


void Quadruped::qrDesiredStateCommand::JoyCallback(const sensor_msgs::Joy::ConstPtr &joy_msg)
{
    joyCmdVz = 0;

    /* If A key is pressed, joy control will be enabled or disabled
     * currently the joy control is enabled by default
     */
    if (joy_msg->buttons[0] == 1) {
        if (!joyCtrlOnRequest) {
            ROS_INFO("You have open joy control!!!\n");
            joyCtrlOnRequest = true;
        } else {
            ROS_INFO("You have turned off joy control!!!\n");
            joyCtrlOnRequest = false;
        }
    }

    rosCmdRequest = !joyCtrlOnRequest;

    if (joyCtrlOnRequest || rosCmdRequest) {
        /* X key. */
        if (joy_msg->buttons[2] == 1) {
            ROS_INFO("You have change the gait !!!\n");
            if (movementMode == 0) {
                movementMode = 1;
            }
            joyCtrlStateChangeRequest = true;
        }

        /* User control the robot locomotion direction and velocity. */
        if (joyCtrlOnRequest) {
            /* Right joy stick up/down. */
            joyCmdVx = joy_msg->axes[4] * MAX_VELX;

            /* Right joy stick horizontal movement. */
            joyCmdVy = joy_msg->axes[3] * MAX_VELY;

            /* Left joy stick horizontal movement. */
            joyCmdYawRate = joy_msg->axes[0] * MAX_YAWRATE;

            /* left cross button left/right movement
             * if you want to enable it, use:
             * joy_msg->axes[6] * JOY_CMD_ROLL_MAX * (-1);
             */
            joyCmdRollRate = 0;

            /* left cross button up/down movement
             * if you want to enable it, use:
             * joy_msg->axes[7] * JOY_CMD_PITCH_MAX;
             */
            joyCmdPitchRate = 0;
        }

        /* If B key is pressed, the quadruped will stop troting and stand by MPC controller. */
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

        /* If Y key is pressed when the quadruped is sitting down,
         * no commands will be sent to quadruped and it will lie on the ground.
         */
        if (joy_msg->buttons[3] == 1) {
            ROS_INFO("You have pressed the exit button!!!!\n");
            if (movementMode == 0 && bodyUp <= 0) {
                joyCmdExit = true;
                joyCtrlStateChangeRequest = true;
                joyCtrlOnRequest = false;
            }
        }

        /* If Rb key is pressed when quadruped is standing by MPC,
         * quadruped will switch to position mode and can sit down and stand up with position mode.
         */
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


void Quadruped::qrDesiredStateCommand::Update()
{
    if (gaitSwitch) {
        joyCtrlStateChangeRequest = true;
        gaitSwitch = false;
    }

    stateDes.setZero();

    if (joyCtrlStateChangeRequest) {

        std::cout << "joyCtrlState before = " << (int)joyCtrlState <<", movementMode = " << movementMode<< std::endl;

        if (movementMode > 0) {
            /* When the quadruped is troting, check state to stop or convert between advanced trot and FB trot. */
            if (joyCtrlState == RC_MODE::HARD_CODE ||  joyCtrlState == RC_MODE::BODY_UP) {
                joyCtrlState = RC_MODE::JOY_STAND;
                std::cout << "joyCtrlState = " << (int)joyCtrlState << std::endl;

            } else if (joyCtrlState == RC_MODE::JOY_STAND) {
                joyCtrlState = RC_MODE::JOY_ADVANCED_TROT;
            } else {
                joyCtrlState = static_cast<RC_MODE>((joyCtrlState+1) %
                                            (RC_MODE::RC_MODE_ITEMS+1));
                if (joyCtrlState == RC_MODE::HARD_CODE) {
                    joyCtrlState = static_cast<RC_MODE>(joyCtrlState+1);
                } else if (joyCtrlState > RC_MODE::JOY_ADVANCED_TROT) {
                    joyCtrlState = RC_MODE::JOY_TROT;
                }
            }
        } else {
            /* If movementMode is set to stop, but control state still remains locomotion, continue locomotion. */
            if (joyCtrlState <= 3) {
                prevJoyCtrlState = joyCtrlState;
            }
            /* If joy commands to exit, then the quadruped sits down, stands up or keep standing/change to MPC standing. */
            if (joyCmdExit) {
                joyCtrlState = RC_MODE::EXIT;
            } else if(bodyUp == -1) {
                joyCtrlState = RC_MODE::BODY_DOWN;
            } else  if (bodyUp == 1){
                joyCtrlState = RC_MODE::BODY_UP;
            } else {
                /* corresponed to LOCOMOTON_STAND in FSM. */
                joyCtrlState = RC_MODE::JOY_STAND;
            }
        }
    }

    if (joyCtrlState == RC_MODE::JOY_STAND) {
        joyCmdVx = 0.;
        joyCmdVy = 0.;
        joyCmdVz = 0.;
        joyCmdYawRate = 0.;
        joyCmdRollRate = 0;
        joyCmdPitchRate = 0;
        filteredVel << joyCmdVx, joyCmdVy, joyCmdVz;
        filteredOmega << joyCmdRollRate, joyCmdPitchRate, joyCmdYawRate;
    } else if (joyCtrlState == RC_MODE::JOY_TROT) {
        filteredVel = filteredVel * (1.0 - filterFactor) +  Vec3<float>(joyCmdVx, joyCmdVy, joyCmdVz) * filterFactor;
        filteredOmega = filteredOmega * (1.0 - filterFactor) + Vec3<float>(joyCmdRollRate, joyCmdPitchRate, joyCmdYawRate) * filterFactor;
    } else if (joyCtrlState == RC_MODE::JOY_ADVANCED_TROT) {
        filteredVel = filteredVel * (1.0 - filterFactor) +  Vec3<float>(joyCmdVx, joyCmdVy, joyCmdVz) * filterFactor;
        filteredOmega = filteredOmega * (1.0 - filterFactor) + Vec3<float>(joyCmdRollRate, joyCmdPitchRate, joyCmdYawRate) * filterFactor;
    } else if (joyCtrlState == RC_MODE::JOY_WALK) {
        filteredVel = filteredVel * (1.0 - filterFactor) +  Vec3<float>(joyCmdVx, joyCmdVy, joyCmdVz) * filterFactor;
        filteredOmega = filteredOmega * (1.0 - filterFactor) + Vec3<float>(joyCmdRollRate, joyCmdPitchRate, joyCmdYawRate) * filterFactor;
    } else if (joyCtrlState == RC_MODE::HARD_CODE) { // No Remote Controller
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

    stateDes(0) = dt * stateDes(6);
    stateDes(1) = dt * stateDes(7);
    stateDes(2) = joycmdBodyHeight;

    stateDes(3) = 0.0;
    stateDes(4) = clip(filteredOmega[1]*dt, MIN_PITCH, MAX_PITCH);
    stateDes(5) = dt * stateDes(11);

    stateDes(6) = clip(filteredVel[0], MIN_VELX, MAX_VELX);
    stateDes(7) = clip(filteredVel[1], MIN_VELY, MAX_VELY);
    stateDes(8) = 0.0;

    stateDes(9) = 0.0;
    stateDes(10) = 0.0;
    stateDes(11) = clip(filteredOmega[2], MIN_YAWRATE, MAX_YAWRATE);

    /* A compensate for walking back, experientially. */
    if (stateDes(6) < -0.01)
        stateDes(2) = joycmdBodyHeight * 0.85; // 0.37 * 0.7 = 0.26, 0.28 * 0.7 = 0.196; 0.28;                                     // Z position height

}


void Quadruped::qrDesiredStateCommand::PrintRawInfo()
{
    /* Increment printing iteration and print at requested frequency. */
    printIter++;

    if (printIter == printNum) {
        std::cout << "[DESIRED STATE COMMAND] Printing Raw Gamepad Info...\n";
        std::cout << "---------------------------------------------------------\n";
        std::cout << "RC MODE: " << joyCtrlState << "\n";
        std::cout << "Right Handle: \n"
                    << " | X: " << joyCmdVx
                    << " | Y: " << joyCmdVy << "\n";
        std::cout << "LEFT Button: \n"
                    << " | YAW: " << joyCmdYawRate
                    << " | PITCH: " << joyCmdPitchRate << "\n";
        std::cout << std::endl;

        printIter = 0;
    }
}


void Quadruped::qrDesiredStateCommand::PrintStateCommandInfo()
{
    /* Increment printing iteration and print at requested frequency. */
    printIter++;
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

