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

#ifndef QR_FSM_STATE_STANDUP_H
#define QR_FSM_STATE_STANDUP_H

#include "qr_fsm_state.hpp"


template <typename T>
class qrFSMStateStandUp : public qrFSMState<T> {

public:

    /**
     * @brief Constructor of FSM_State_StandUp
     * @param controlFSMData: pointer to the data this FSM needed
     */
    qrFSMStateStandUp(qrControlFSMData<T>* controlFSMData);

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

private:

    /**
     * @brief A variable that indicates whether to stand up.
     * If %standup == 1, then the quadruped should stand up.
     * If %standup == 0, then the quadruped should keep current position.
     * If %standup == -1, then the quadruped should sit down.
     */
    int standUp;

    /**
     * @brief Whether the quadruped stands.
     */
    bool isUp;

    /**
     * @brief When quadruped is standing up, it stores the angle after standing up.
     * When quadruped is sitting down, it stores the angle after sitting down.
     */
    Eigen::Matrix<float, 12 ,1> motorAngles;

};

#endif // QR_FSM_STATE_STANDUP_H
