/**
 * MotorDriverReadBack.h
 *
 *  Created on: Sep 17, 2018
 *      Author: ckampm
 *
 */

#ifndef INCLUDE_MOTORDRIVERREADBACK_H_
#define INCLUDE_MOTORDRIVERREADBACK_H_

#include <ChimeraTK/ApplicationCore/ApplicationCore.h>
#include <ChimeraTK/ReadAnyGroup.h>
#include <ChimeraTK/MotorDriverCard/StepperMotor.h>
//#include <ChimeraTK/MotorDriverCard/StepperMotorWithReference.h>
#include "ExecutionTimer.h"

#include <functional>

namespace ctk    = ChimeraTK;
namespace ctkmot = ctk::MotorDriver;


/**
 *  Position data
 */
struct Position : public ctk::VariableGroup {

  using ctk::VariableGroup::VariableGroup;

  ctk::ScalarOutput<float> actualValue{this, "actualValue", "", "Actual position"};
  ctk::ScalarOutput<int> actualValueInSteps{this, "actualValueInSteps", "", "Actual position ]"};

  ctk::ScalarOutput<double> encoderReadback{this, "encoder", "", "Encoder readback"};

  ctk::ScalarOutput<float> targetValue{this, "targetValue", "", "Readback of the target position"};
  ctk::ScalarOutput<int> targetValueInSteps{this, "targetValueInSteps", "", "Readback of target position"};
};

/**
 * Limit data
 */
struct Limit : public ctk::VariableGroup {

  using ctk::VariableGroup::VariableGroup;

  ctk::ScalarOutput<double> userValue{this, "userValue", "", "Speed limit set for the motor"};
  ctk::ScalarOutput<double> maxValue{this, "maxValue", "", "Maximum velocity of the motor"};

};



/**
 * Motor status information
 */
struct MotorStatus : public ctk::VariableGroup {

  using ctk::VariableGroup::VariableGroup;

  ctk::ScalarOutput<int> isEnabled{this, "isEnabled", "", "Motor current enable status"};
  ctk::ScalarOutput<int> calibrationMode{this, "calibrationMode", "", "Current calibration mode"};
  ctk::ScalarOutput<int> isIdle{this, "isIdle", "", "Flags if system is idle and a movement or calibration can be started"};

  ctk::ScalarOutput<std::string> state{this, "state", "", "State of the motor driver"};
  ctk::ScalarOutput<int> errorId{this, "errorId", "", "Error ID of the motor driver"};

  ctk::ScalarOutput<int> isFullStepping{this, "isFullStepping", "", "Flags if full-stepping mode of the driver is active."};
  ctk::ScalarOutput<int> autostartEnabled{this, "autostartEnabled", "", "Flags if autostart mode is active"};
};

/**
 * Data for SW-defined position limits
 */
struct SoftwareLimitStat : public ctk::VariableGroup {

  using ctk::VariableGroup::VariableGroup;

  ctk::ScalarOutput<int> isEnabled{this, "isEnabled", "", "SW limit feature enabled"};

  ctk::ScalarOutput<float> maxPosition{this, "maxPosition", "", "Max. SW position limit"};
  ctk::ScalarOutput<float> minPosition{this, "minPosition", "", "Min. SW position limit"};
  ctk::ScalarOutput<int> maxPositionInSteps{this, "maxPositionInSteps", "steps", "Max. SW position limit"};
  ctk::ScalarOutput<int> minPositionInSteps{this, "minPositionInSteps", "steps", "Min. SW position limit"};
};

/**
 * VariableGroup describing the status of a reference switch
 */
struct ReferenceSwitch : public ctk::VariableGroup {

  using ctk::VariableGroup::VariableGroup;

  ctk::ScalarOutput<int>   positionInSteps{this, "positionInSteps", "steps", "Position of the positive reference switch"};
  ctk::ScalarOutput<float> position{this, "position", "", "Position of the positive reference switch"};
  ctk::ScalarOutput<float> tolerance{this, "tolerance", "", "Tolerance of the calibrated positive end switch position."};
  ctk::ScalarOutput<int>   isActive{this, "isActive", "", "Status of negative end switch"};

};


/**
 *  @class ReadbackHandler
 *  @brief Base application module for cyclically reading data from the motor driver card HW.
 */
class ReadbackHandler : public ctk::ApplicationModule {

public:

  ReadbackHandler(std::shared_ptr<ctkmot::StepperMotor> motor, ctk::EntityOwner *owner, const std::string &name, const std::string &description);

  //ctk::ScalarPushInput<int> trigger{this, "trigger", "", "Trigger to initiate reading from HW", {"MOT_TRIG"}};
  ctk::ScalarPushInput<uint64_t> trigger{this, "tick", "", "Trigger to initiate reading from HW", {"MOT_TRIG"}};


  // Diagnostics
  ctk::ScalarOutput<float> actualCycleTime{this, "actualCycleTime", "ms", "Actual cycle time by which the HW is being read", {"MOT_DIAG"}};
  ctk::ScalarOutput<float> actualReceiveTime{this, "actualReceiveTime", "ms", "Actual time required to read all variables in this module from the HW.", {"MOT_DIAG"}};

  virtual void mainLoop() override;

  Position position{this, "position", "Position data", false, {"MOTOR"}};
  Limit           speedLimit{this, "speedLimit", "Speed data", false, {"MOTOR"}};
  Limit           currentLimit{this, "currentLimit", "Current data", false, {"MOTOR"}};
  MotorStatus     status{this, "status", "Status data of the motor driver", false, {"MOTOR"}};
  SoftwareLimitStat  swLimits{this, "swLimits", "Status data of SW limits", false, {"MOTOR"}};
  ReferenceSwitch positiveEndSwitch;
  ReferenceSwitch negativeEndSwitch;

protected:
  std::function<void(void)> readbackFunction;

private:
  void readback();
  void readEndSwitchData();
  /**
   * Some variables are only calculated at startup
   * of the MotorDriverCard lib, so read them only once
   */
  void readConstData();

  std::shared_ptr<ctkmot::StepperMotor> _motor;
  ExecutionTimer<> execTimer;
  ExecutionTimer<> receiveTimer;

  /// Hack to prevent throwing when the motor dummy is used
  const bool _motorIsDummy;
  bool motorIsDummy();
};

#endif /* INCLUDE_MOTORDRIVERREADBACK_H_ */
