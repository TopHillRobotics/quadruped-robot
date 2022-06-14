import os
import inspect
# currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
# parentdir = os.path.dirname(os.path.dirname(currentdir))
# os.sys.path.insert(0, parentdir)
os.sys.path.insert(0, "/home/ascend/Documents/motion_imitation-master")
os.sys.path.insert(0, "/home/ascend/Documents/invariant-ekf/build")

import time
import numpy as np
from inekf_interface import INEKFInterface # pytype: disable=import-error
from robot_interface import RobotInterface # pytype: disable=import-error


class INEKF_ON_A1():
    def __init__(self):
        self._inekf_interface = INEKFInterface()
        self._robot_interface = RobotInterface()
        self._raw_state = None
        self._estimated_state = None

    def getImuData(self):
        obs = self._robot_interface.receive_observation()
        return obs.imu

    def ReceiveObservation(self):
        """Receives observation from robot.

        Synchronous ReceiveObservation is not supported in A1,
        so changging it to noop instead.
        """
        print("\n---------------------------------------------------------")
        state = self._robot_interface.receive_observation()
        self._raw_state = state
        # print("raw_state: ", self._raw_state)
        # # Convert quaternion from wxyz to xyzw, which is default for Pybullet.

        q = state.imu.quaternion
        # self._base_orientation = np.array([q[1], q[2], q[3], q[0]])
        print("quaternion", q)
        rpy = state.imu.rpy
        print("rpy:", rpy)
        acc = state.imu.accelerometer
        print("acc:", acc)
        gyro = state.imu.gyroscope
        print("gyro:", gyro)
        print("footForce:", state.footForce)
        print("footForceEst:", state.footForceEst)
        self._motor_angles = np.array([motor.q for motor in state.motorState[:12]])
        self._motor_velocities = np.array(
            [motor.dq for motor in state.motorState[:12]])
        self._joint_states = np.array(
            list(zip(self._motor_angles, self._motor_velocities)))
        print("joint_states:", self._joint_states)

    def update(self):
        T = time.time()
        self._inekf_interface.receive_imu_data(T, self.getImuData())
        self._estimated_state = self._inekf_interface.update()
    
    def update_contact(self, is_constact):
        """
        Input: 
           is_contact is numpy.array or list of bool values.
        """
        self._inekf_interface.update_contact(is_constact)
    

    @property
    def estimated_state(self):
        return self._estimated_state


def main():
    # input("Press enter to continue...")
    estimator = INEKF_ON_A1()
    time.sleep(2)
    estimator.update_contact(np.array([1, 0, 0, 1], dtype=np.bool))
    estimator.update()
    s = estimator.estimated_state # class: RobotState
    r = s.getRotation()
    print(r)
    raise
    while True:
        estimator.update()
        time.sleep(1)


if __name__ == '__main__':
    main()
