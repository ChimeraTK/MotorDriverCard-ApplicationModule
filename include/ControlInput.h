/*
 * ControlInput.h
 *
 *  Created on: Sep 13, 2018
 *      Author: ckampm
 */

#ifndef INCLUDE_CONTROLINPUT_H_
#define INCLUDE_CONTROLINPUT_H_

#include <ChimeraTK/ApplicationCore/ApplicationCore.h>
#include <ChimeraTK/ReadAnyGroup.h>
#include <ChimeraTK/MotorDriverCard/StepperMotor.h>
#include <ChimeraTK/MotorDriverCard/LinearStepperMotor.h>

#include <map>
#include <functional>
#include <memory>
#include <utility>

namespace ctk    = ChimeraTK;
namespace ctkmot = ctk::MotorDriver;

namespace ChimeraTK {
namespace MotorDriver {
/**
 * A map between the TransferElementID of a PV and the associated
 * function of the MotorDriverCard library. This allows to pass on
 * the changed PV to the library by the ID returned from readAny().
 */
using funcmapT = std::map<ctk::TransferElementID, std::function<void(void)>>;


/// Calibration related inputs, only available for motors with HW end reference switches
struct CalibrationCommands: public ctk::VariableGroup {

  using ctk::VariableGroup::VariableGroup;

  ctk::ScalarPushInput<int32_t> calibrateMotor{this, "calibrate", "", "Calibrates the motor"};
  ctk::ScalarPushInput<int32_t> determineTolerance{this, "determineTolerance", "", "Determines tolerance of the end switch positions"};
};

/// Motor control data
struct MotorControl : public ctk::VariableGroup {

  using ctk::VariableGroup::VariableGroup;

  ctk::ScalarPushInput<int> enable{this, "enable", "", "Enable the motor"};
  ctk::ScalarPushInput<int> disable{this, "disable", "", "Disable the motor"};
  ctk::ScalarPushInput<int> start{this, "start", "", "Start the motor"};
  //ctk::ScalarPushInput<int> calibrate{this, "calibDoNotUSE", "", "Calibration input dummy, do not use!"};

  CalibrationCommands calibrationCtrl;

  ctk::ScalarPushInput<int> stop{this, "stop", "", "Stop the motor"};
  ctk::ScalarPushInput<int> emergencyStop{this, "emergencyStop", "", "Emergency stop motor"};
  ctk::ScalarPushInput<int> resetError{this, "resetError", "", "Reset error state"};

  ctk::ScalarPushInput<int> enableFullStepping{this, "enableFullStepping", "", "Enables full-stepping mode of the motor driver, i.e., it will only stop on full steps"};
  ctk::ScalarPushInput<int> enableAutostart{this, "enableAutostart", "", "Sets the autostart flag of the motor driver"};
};

///  Position setpoints
struct PositionSetpoint : public ctk::VariableGroup {

  using ctk::VariableGroup::VariableGroup;

  ctk::ScalarPushInput<int>   positionInSteps{this, "positionInSteps", "steps", "Motor position setpoint"};
  ctk::ScalarPushInput<float> position{this, "position", "", "Motor position setpoint"};

  ctk::ScalarPushInput<int> relativePositionInSteps{
    this, "relativePositionInSteps", "", "Initiates a movement relative to the current position"};
  ctk::ScalarPushInput<float> relativePosition{
    this, "relativePosition", "", "Initiates a movement relative to the current position"};
};


/// Contains settings to define and shift the position reference
struct ReferenceSettings : public ctk::VariableGroup {

  using ctk::VariableGroup::VariableGroup;

  ctk::ScalarPushInput<float> position{this, "position", "", "Writing to this value sets the actual motor position to a given reference"};
  ctk::ScalarPushInput<int> positionInSteps{this, "positionInSteps", "", "Writing to this value sets the actual motor position to a given reference"};

  ctk::ScalarPushInput<int32_t> encoderPosition{this, "encoderPosition", "", "Writing to this value sets the actual encoder position to a given reference"};

  ctk::ScalarPushInput<int>   axisTranslationInSteps{this, "axisTranslationInSteps", "steps", "Offset to translate axis, i.e. shift the reference point."};
  ctk::ScalarPushInput<float> axisTranslation{this, "axisTranslation", "", "Offset to translate axis, i.e. shift the reference point."};
};

/// Control of the software limits
struct SoftwareLimitCtrl : public ctk::VariableGroup {

  using ctk::VariableGroup::VariableGroup;

  ctk::ScalarPushInput<int> enable{this, "enable", "", "Enable SW limits"};
  ctk::ScalarPushInput<float> maxPosition{this, "maxPosition", "", "Positive SW position limit"};
  ctk::ScalarPushInput<float> minPosition{this, "minPosition", "", "Negative SW position limit"};
  ctk::ScalarPushInput<int> maxPositionInSteps{this, "maxPositionInSteps", "", "Positive SW position limit"};
  ctk::ScalarPushInput<int> minPositionInSteps{this, "minPositionInSteps", "", "Negative SW position limit"};
};

/// Notifications to the user
struct Notification : public ctk::VariableGroup {

  using ctk::VariableGroup::VariableGroup;

  ctk::ScalarOutput<int>         hasMessage{this, "hasMessage", "", "Warning flag, true when an invalid input has been issued."};
  ctk::ScalarOutput<std::string> message{this, "message", "", "Message for user notification from ControlInput module"};
};

/// User-definable limits
struct UserLimits : public ctk::VariableGroup {

  using ctk::VariableGroup::VariableGroup;

  ctk::ScalarPushInput<double> current{this, "current", "A", "User current limit for the motor"};
  ctk::ScalarPushInput<double> speed{this, "speed", "", "User speed limit for the motor"};
};

/// Signals triggering the dummy motor
struct DummySignals : public ctk::VariableGroup {

  using ctk::VariableGroup::VariableGroup;

  ctk::ScalarOutput<int32_t> dummyMotorTrigger{this, "dummyMotorTrigger", "", "Triggers the dummy motor module after writing to a control input"};
  ctk::ScalarOutput<int32_t> dummyMotorStop{this, "dummyMotorStop","", "Stops the dummy motor"};

};


/**
 *  @class ControlInputHandlerImpl
 *  @details Contains the implementation of the ControlInputHandler as a template so it can be used
 *           with a specific motor and set of inputs.
 */
class ControlInputHandler : public ctk::ApplicationModule {
public:

  ControlInputHandler(ctk::EntityOwner *owner, const std::string &name, const std::string &description, std::shared_ptr<ctkmot::StepperMotor> motor);

  //virtual ~ControlInputHandler() {}

  virtual void prepare() override;
  virtual void mainLoop() override;

private:
  virtual void createFunctionMap(std::shared_ptr<ctkmot::StepperMotor> _motor);
  virtual void appendCalibrationToMap();
  funcmapT funcMap;

  MotorControl control{this, "control", "Control words of the motor", false, {"MOTOR"}};
  PositionSetpoint positionSetpoint{this, "positionSetpoint", "Position setpoints", false, {"MOTOR"}};
  UserLimits userLimits{this, "userLimits", "User-definable limits", false, {"MOTOR"}};
  SoftwareLimitCtrl swLimits{this, "swLimits", "Control data of SW limits", false, {"MOTOR"}};
  ReferenceSettings referenceSettings{this, "referenceSettings", "Settings to define the position reference", false, {"MOTOR"}};
  Notification notification{this, "notification", "User notification", false, {"MOTOR"}};
  DummySignals dummySignals{this, "dummySignals", " Signals triggering the dummy motor", false, {"DUMMY"}};
  //CalibrationCommands _calibrationCommands;

  // Callbacks for the BasiStepperMotor
  void enableCallback();
  void disableCallback();
  void startCallback();

  //Callbacks for the LinearStepperMotor
  void calibrateCallback();
  void determineToleranceCallback();

  ctk::ReadAnyGroup inputGroup;
  std::shared_ptr<ctkmot::StepperMotor> _motor;

}; /* class ControlInputHandler */
}
}
#endif /* INCLUDE_CONTROLINPUT_H_ */
