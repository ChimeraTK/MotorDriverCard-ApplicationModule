/*
 * StepperMotorCtrl.h
 *
 *  Created on: Sep 13, 2018
 *      Author: ckampm
 */

#ifndef INCLUDE_STEPPERMOTORCTRL_H_
#define INCLUDE_STEPPERMOTORCTRL_H_

#include <ChimeraTK/ApplicationCore/ApplicationCore.h>
#include <ChimeraTK/ReadAnyGroup.h>
#include <ChimeraTK/MotorDriverCard/StepperMotor.h>
#include <ChimeraTK/MotorDriverCard/LinearStepperMotor.h>

#include <map>
#include <functional>
#include <memory>
#include <utility>

#include "Motor.h"

namespace ChimeraTK { namespace MotorDriver {
  /**
 * A map between the TransferElementID of a PV and the associated
 * function of the MotorDriverCard library. This allows to pass on
 * the changed PV to the library by the ID returned from readAny().
 */
  using funcmapT = std::map<TransferElementID, std::function<void(void)>>;

  /// Calibration related inputs, only available for motors with HW end reference switches
  struct CalibrationCommands : public VariableGroup {
    using VariableGroup::VariableGroup;

    ScalarPushInput<int32_t> calibrateMotor{this, "calibrate", "", "Calibrates the motor"};
    ScalarPushInput<int32_t> determineTolerance{
        this, "determineTolerance", "", "Determines tolerance of the end switch positions"};
  };

  /// Motor control data
  struct MotorControl : public VariableGroup {
    using VariableGroup::VariableGroup;

    ScalarPushInput<int> enable{this, "enable", "", "Enable the motor"};
    ScalarPushInput<int> disable{this, "disable", "", "Disable the motor"};
    ScalarPushInput<int> start{this, "start", "", "Start the motor"};

    CalibrationCommands calibrationCtrl{};

    ScalarPushInput<int> stop{this, "stop", "", "Stop the motor"};
    ScalarPushInput<int> emergencyStop{this, "emergencyStop", "", "Emergency stop motor"};
    ScalarPushInput<int> resetError{this, "resetError", "", "Reset error state"};

    ScalarPushInput<int> enableFullStepping{this, "enableFullStepping", "",
        "Enables full-stepping mode of the motor driver, i.e., it will only stop on full steps"};
    ScalarPushInput<int> enableAutostart{this, "enableAutostart", "", "Sets the autostart flag of the motor driver"};
  };

  ///  Position setpoints
  struct PositionSetpoint : public VariableGroup {
    using VariableGroup::VariableGroup;

    ScalarPushInput<int> positionInSteps{this, "positionInSteps", "steps", "Motor position setpoint"};
    ScalarPushInput<float> position{this, "position", "", "Motor position setpoint"};

    ScalarPushInput<int> relativePositionInSteps{
        this, "relativePositionInSteps", "", "Initiates a movement relative to the current position"};
    ScalarPushInput<float> relativePosition{
        this, "relativePosition", "", "Initiates a movement relative to the current position"};
  };

  /// Contains settings to define and shift the position reference
  struct ReferenceSettings : public VariableGroup {
    using VariableGroup::VariableGroup;

    ScalarPushInput<float> position{
        this, "position", "", "Writing to this value sets the actual motor position to a given reference"};
    ScalarPushInput<int> positionInSteps{
        this, "positionInSteps", "", "Writing to this value sets the actual motor position to a given reference"};

    ScalarPushInput<int32_t> encoderPosition{
        this, "encoderPosition", "", "Writing to this value sets the actual encoder position to a given reference"};

    ScalarPushInput<int> axisTranslationInSteps{
        this, "axisTranslationInSteps", "steps", "Offset to translate axis, i.e. shift the reference point."};
    ScalarPushInput<float> axisTranslation{
        this, "axisTranslation", "", "Offset to translate axis, i.e. shift the reference point."};
  };

  /// Control of the software limits
  struct SoftwareLimitCtrl : public VariableGroup {
    using VariableGroup::VariableGroup;

    ScalarPushInput<int> enable{this, "enable", "", "Enable SW limits"};
    ScalarPushInput<float> maxPosition{this, "maxPosition", "", "Positive SW position limit"};
    ScalarPushInput<float> minPosition{this, "minPosition", "", "Negative SW position limit"};
    ScalarPushInput<int> maxPositionInSteps{this, "maxPositionInSteps", "", "Positive SW position limit"};
    ScalarPushInput<int> minPositionInSteps{this, "minPositionInSteps", "", "Negative SW position limit"};
  };

  /// Notifications to the user
  struct Notification : public VariableGroup {
    using VariableGroup::VariableGroup;

    ScalarOutput<int> hasMessage{this, "hasMessage", "", "Warning flag, true when an invalid input has been issued."};
    ScalarOutput<std::string> message{this, "message", "", "Message for user notification from ControlInput module"};
  };

  /// User-definable limits
  struct UserLimits : public VariableGroup {
    using VariableGroup::VariableGroup;

    ScalarPushInput<double> current{this, "current", "A", "User current limit for the motor"};
    ScalarPushInput<double> speed{this, "speed", "", "User speed limit for the motor"};
  };

  /// Signals triggering the dummy motor
  struct DummySignals : public VariableGroup {
    using VariableGroup::VariableGroup;

    ScalarOutput<int32_t> dummyMotorTrigger{
        this, "dummyMotorTrigger", "", "Triggers the dummy motor module after writing to a control input"};
    ScalarOutput<int32_t> dummyMotorStop{this, "dummyMotorStop", "", "Stops the dummy motor"};
  };

  /**
 *  @class ControlInputHandlerImpl
 *  @details Contains the implementation of the ControlInputHandler as a template so it can be used
 *           with a specific motor and set of inputs.
 */
  class ControlInputHandler : public ApplicationModule {
   public:
    ControlInputHandler(
        ModuleGroup* owner, const std::string& name, const std::string& description, std::shared_ptr<Motor> motor);

    //virtual ~ControlInputHandler() {}

    virtual void prepare() override;
    virtual void mainLoop() override;

   private:
    virtual void createFunctionMap(std::shared_ptr<Motor> _motor);
    virtual void appendCalibrationToMap();
    funcmapT funcMap;

    MotorControl control{this, "control", "Control words of the motor", {"MOTOR"}};
    PositionSetpoint positionSetpoint{
        this, "positionSetpoint", "Position setpoints", {"MOTOR"}};
    UserLimits userLimits{this, "userLimits", "User-definable limits", {"MOTOR"}};
    SoftwareLimitCtrl swLimits{this, "swLimits", "Control data of SW limits", {"MOTOR"}};
    ReferenceSettings referenceSettings{
        this, "referenceSettings", "Settings to define the position reference", {"MOTOR"}};
    Notification notification{this, "notification", "User notification", {"MOTOR"}};
    DummySignals dummySignals{
        this, "dummySignals", " Signals triggering the dummy motor", {"DUMMY"}};
    //CalibrationCommands _calibrationCommands;

    // Callbacks for the BasiStepperMotor
    void enableCallback();
    void disableCallback();
    void startCallback();

    //Callbacks for the LinearStepperMotor
    void calibrateCallback();
    void determineToleranceCallback();

    ReadAnyGroup inputGroup;
    std::shared_ptr<Motor> _motor;

  };   /* class ControlInputHandler */
}}     // namespace ChimeraTK::MotorDriver
#endif /* INCLUDE_STEPPERMOTORCTRL_H_ */
