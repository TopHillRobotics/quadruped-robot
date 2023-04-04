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

#ifndef QR_FSM_STATE_LOCOMOTION_H
#define QR_FSM_STATE_LOCOMOTION_H

#include "qr_control_fsm_data.hpp"
#include "controllers/qr_locomotion_controller.h"
#include "controllers/wbc/qr_wbc_locomotion_controller.hpp"
#include "qr_fsm_state.hpp"

/**
 * @brief FSM State for robot locomotion. Manages the contact specific logic
 * and handles calling the interfaces to the controllers. This state
 * should be independent of controller, gait, and desired trajectory.
 */
template<typename T>
class qrFSMStateLocomotion : public qrFSMState<T> {

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief Constructor of qrFSMStateLocomotion
     * @param controlFSMData: pointer to the data this FSM needed
     */
    qrFSMStateLocomotion(qrControlFSMData<T> *control_fsm_data);

    /**
     * @brief Getter method of locomotionController.
     * @return pointer to locomotionController.
     */
    Quadruped::qrLocomotionController *GetLocomotionController() const {
        return locomotionController;
    }

    /**
     * @see qrFSMState::OnEnter
     */
    void OnEnter();

    /**
     * @see qrFSMState::Run
     */
    virtual void Run();

    /**
     * @see qrFSMState::CheckTransition
     */
    FSM_StateName CheckTransition();

    /**
     * @see qrFSMState::Transition
     */
    qrTransitionData<T> Transition();

    /**
     * @see qrFSMState::OnExit
     */
    void OnExit();

    /**
     * @brief Reset the locomotion controller during locomotion state.
     * @param currentTime: time to reset the locomotion controller
     */
    void Reset(float currentTime) {
        this->resetTime = currentTime;
        this->timeSinceReset = 0;
        locomotionController->Reset();
    }

private:

    /**
     * @brief Switch gait in locomotion state.
     * Usually switch from trot locomotion to walk locomotion.
     * @return success to switch gait
     */
    bool SwitchMode();

    /**
     * @brief Check safty before transition.
     * Currently this function just return true.
     * @return whether locomotion is currently safe
     */
    bool LocomotionSafe();

    /**
     * @brief The quadruped will keep standing for %transitionDuration * 1000 iterations.
     * This method is used for transitioning.
     * @return currently just return true.
     */
    bool StandLoop();
    
    /**
     * @brief Pointer to LocomotionController.
     */
    Quadruped::qrLocomotionController *locomotionController;

    /**
     * @brief Pointer to Whole Body Controller.
     */
    qrWbcLocomotionController<T> *wbcController;

    /**
     * @brief Pointer to data needed by Whole Body Controller.
     */
    qrWbcCtrlData *wbcData;

    /**
     * @brief Keep tracking of the iterations of the locomotion controller.
     */
    unsigned long iter = 0;

};

#endif // QR_FSM_STATE_LOCOMOTION_H
