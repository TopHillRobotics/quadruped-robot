/**
 * Checks the robot state for safe operation commands after calculating the
 * control iteration. Prints out which command is unsafe. Each state has
 * the option to enable checks for commands that it cares about.
 *
 * Should this EDamp / EStop or just continue?
 * Should break each separate check into its own function for clarity
 */

// #include "SafetyChecker.h"

/**
 * @return safePDesFoot true if safe desired foot placements
 */
template <typename T>
bool SafetyChecker<T>::CheckSafeOrientation() {
  if (abs(data->quadruped->GetBaseRollPitchYaw()[0]) >= 0.5 ||
      abs(data->quadruped->GetBaseRollPitchYaw()[1]) >= 0.5) {
        printf("Orientation safety check failed!\n");
    return false;
  } else {
    return true;
  }
}

/**
 * @return safePDesFoot true if safe desired foot placements
 */
template <typename T>
bool SafetyChecker<T>::CheckPDesFoot() {
  // Assumed safe to start
  bool safePDesFoot = true;

  // Safety parameters
  T maxAngle = 1.0472;  // 60 degrees (should be changed)
  T maxPDes = data->quadruped->upperLegLength * sin(maxAngle);

  // Check all of the legs
  // for (int leg = 0; leg < 4; leg++) {
    // Keep the foot from going too far from the body in +x
    // if (data->_legController->commands[leg].pDes(0) > maxPDes) {
    //   std::cout << "[CONTROL FSM] Safety: PDes leg: " << leg
    //             << " | coordinate: " << 0 << "\n";
    //   std::cout << "   commanded: "
    //             << data->_legController->commands[leg].pDes(0)
    //             << " | modified: " << maxPDes << std::endl;
    //   data->_legController->commands[leg].pDes(0) = maxPDes;
    //   safePDesFoot = false;
    // }

    // Keep the foot from going too far from the body in -x
    // if (data->_legController->commands[leg].pDes(0) < -maxPDes) {
    //   std::cout << "[CONTROL FSM] Safety: PDes leg: " << leg
    //             << " | coordinate: " << 0 << "\n";
    //   std::cout << "   commanded: "
    //             << data->_legController->commands[leg].pDes(0)
    //             << " | modified: " << -maxPDes << std::endl;
    //   data->_legController->commands[leg].pDes(0) = -maxPDes;
    //   safePDesFoot = false;
    // }

    // Keep the foot from going too far from the body in +y
    // if (data->_legController->commands[leg].pDes(1) > maxPDes) {
    //   std::cout << "[CONTROL FSM] Safety: PDes leg: " << leg
    //             << " | coordinate: " << 1 << "\n";
    //   std::cout << "   commanded: "
    //             << data->_legController->commands[leg].pDes(1)
    //             << " | modified: " << maxPDes << std::endl;
    //   data->_legController->commands[leg].pDes(1) = maxPDes;
    //   safePDesFoot = false;
    // }

    // Keep the foot from going too far from the body in -y
    // if (data->_legController->commands[leg].pDes(1) < -maxPDes) {
    //   std::cout << "[CONTROL FSM] Safety: PDes leg: " << leg
    //             << " | coordinate: " << 1 << "\n";
    //   std::cout << "   commanded: "
    //             << data->_legController->commands[leg].pDes(1)
    //             << " | modified: " << -maxPDes << std::endl;
    //   data->_legController->commands[leg].pDes(1) = -maxPDes;
    //   safePDesFoot = false;
    // }

    // Keep the leg under the motor module (don't raise above body or crash into
    // module)
    // if (data->_legController->commands[leg].pDes(2) >
    //     -data->_quadruped->_maxLegLength / 4) {
    //   std::cout << "[CONTROL FSM] Safety: PDes leg: " << leg
    //             << " | coordinate: " << 2 << "\n";
    //   std::cout << "   commanded: "
    //             << data->_legController->commands[leg].pDes(2)
    //             << " | modified: " << -data->_quadruped->_maxLegLength / 4
    //             << std::endl;
    //   data->_legController->commands[leg].pDes(2) =
    //       -data->_quadruped->_maxLegLength / 4;
    //   safePDesFoot = false;
    // }

    // Keep the foot within the kinematic limits
    // if (data->_legController->commands[leg].pDes(2) <
    //     -data->_quadruped->_maxLegLength) {
    //   std::cout << "[CONTROL FSM] Safety: PDes leg: " << leg
    //             << " | coordinate: " << 2 << "\n";
    //   std::cout << "   commanded: "
    //             << data->_legController->commands[leg].pDes(2)
    //             << " | modified: " << -data->_quadruped->_maxLegLength
    //             << std::endl;
    //   data->_legController->commands[leg].pDes(2) =
    //       -data->_quadruped->_maxLegLength;
    //   safePDesFoot = false;
    // }
  // }

  // Return true if all desired positions are safe
  return safePDesFoot;
}

/**
 * @return safePDesFoot true if safe desired foot placements
 */
template <typename T>
bool SafetyChecker<T>::CheckForceFeedForward() {
  // Assumed safe to start
  bool safeForceFeedForward = true;

  // Initialize maximum vertical and lateral forces
  T maxLateralForce = 0;
  T maxVerticalForce = 0;

  // Maximum force limits for each robot
  if (data->quadruped->robotName == "a1_sim") {
    maxLateralForce = 1800;
    maxVerticalForce = 1800;

  } else if (data->quadruped->robotName == "a1_robot") {
    maxLateralForce = 350;
    maxVerticalForce = 350;
  }

  // Check all of the legs
  for (int leg = 0; leg < 4; leg++) {
    for (int motorId(0); motorId< 3; ++motorId) {
      if (data->legCmd[leg*3+motorId].tua>23) {
        data->legCmd[leg*3+motorId].tua = 23;
      } else if (data->legCmd[leg*3+motorId].tua<-23) {
        data->legCmd[leg*3+motorId].tua = -23;
      }
    }
    // Limit the lateral forces in +x body frame
    // if (data->_legController->commands[leg].forceFeedForward(0) >
    //     maxLateralForce) {
    //   std::cout << "[CONTROL FSM] Safety: Force leg: " << leg
    //             << " | coordinate: " << 0 << "\n";
    //   std::cout << "   commanded: "
    //             << data->_legController->commands[leg].forceFeedForward(0)
    //             << " | modified: " << maxLateralForce << std::endl;
    //   data->_legController->commands[leg].forceFeedForward(0) = maxLateralForce;
    //   safeForceFeedForward = false;
    // }

    // Limit the lateral forces in -x body frame
    // if (data->_legController->commands[leg].forceFeedForward(0) <
    //     -maxLateralForce) {
    //   std::cout << "[CONTROL FSM] Safety: Force leg: " << leg
    //             << " | coordinate: " << 0 << "\n";
    //   std::cout << "   commanded: "
    //             << data->_legController->commands[leg].forceFeedForward(0)
    //             << " | modified: " << -maxLateralForce << std::endl;
    //   data->_legController->commands[leg].forceFeedForward(0) =
    //       -maxLateralForce;
    //   safeForceFeedForward = false;
    // }

    // Limit the lateral forces in +y body frame
    // if (data->_legController->commands[leg].forceFeedForward(1) >
    //     maxLateralForce) {
    //   std::cout << "[CONTROL FSM] Safety: Force leg: " << leg
    //             << " | coordinate: " << 1 << "\n";
    //   std::cout << "   commanded: "
    //             << data->_legController->commands[leg].forceFeedForward(1)
    //             << " | modified: " << maxLateralForce << std::endl;
    //   data->_legController->commands[leg].forceFeedForward(1) = maxLateralForce;
    //   safeForceFeedForward = false;
    // }

    // Limit the lateral forces in -y body frame
    // if (data->_legController->commands[leg].forceFeedForward(1) <
    //     -maxLateralForce) {
    //   std::cout << "[CONTROL FSM] Safety: Force leg: " << leg
    //             << " | coordinate: " << 1 << "\n";
    //   std::cout << "   commanded: "
    //             << data->_legController->commands[leg].forceFeedForward(1)
    //             << " | modified: " << -maxLateralForce << std::endl;
    //   data->_legController->commands[leg].forceFeedForward(1) =
    //       -maxLateralForce;
    //   safeForceFeedForward = false;
    // }

    // Limit the vertical forces in +z body frame
    // if (data->_legController->commands[leg].forceFeedForward(2) >
    //     maxVerticalForce) {
    //   std::cout << "[CONTROL FSM] Safety: Force leg: " << leg
    //             << " | coordinate: " << 2 << "\n";
    //   std::cout << "   commanded: "
    //             << data->_legController->commands[leg].forceFeedForward(2)
    //             << " | modified: " << -maxVerticalForce << std::endl;
    //   data->_legController->commands[leg].forceFeedForward(2) =
    //       maxVerticalForce;
    //   safeForceFeedForward = false;
    // }

    // Limit the vertical forces in -z body frame
    // if (data->_legController->commands[leg].forceFeedForward(2) <
    //     -maxVerticalForce) {
    //   std::cout << "[CONTROL FSM] Safety: Force leg: " << leg
    //             << " | coordinate: " << 2 << "\n";
    //   std::cout << "   commanded: "
    //             << data->_legController->commands[leg].forceFeedForward(2)
    //             << " | modified: " << maxVerticalForce << std::endl;
    //   data->_legController->commands[leg].forceFeedForward(2) =
    //       -maxVerticalForce;
    //   safeForceFeedForward = false;
    // }
  }

  // Return true if all feed forward forces are safe
  return safeForceFeedForward;
}

// template class SafetyChecker<double>; This should be fixed... need to make
// RobotRunner a template
template class SafetyChecker<float>;
