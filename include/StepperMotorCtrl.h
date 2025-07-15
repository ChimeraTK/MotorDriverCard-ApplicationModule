// SPDX-FileCopyrightText: Deutsches Elektronen-Synchrotron DESY, MSK, ChimeraTK Project <chimeratk-support@desy.de>
// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "Motor.h"

#include <ChimeraTK/ApplicationCore/ApplicationCore.h>
#include <ChimeraTK/MotorDriverCard/LinearStepperMotor.h>
#include <ChimeraTK/MotorDriverCard/StepperMotor.h>
#include <ChimeraTK/ReadAnyGroup.h>

#include <functional>
#include <map>
#include <memory>

namespace ChimeraTK::MotorDriver {
  /// Calibration related inputs, only available for motors with HW end reference switches
  struct CalibrationCommands : public VariableGroup {
    using VariableGroup::VariableGroup;

    VoidInput calibrateMotor{this, "calibrate", "Calibrates the motor"};
    VoidInput determineTolerance{this, "determineTolerance", "Determines tolerance of the end switch positions"};
  };

  /********************************************************************************************************************/

  /// Motor control data
  struct MotorControl : public VariableGroup {
    using VariableGroup::VariableGroup;

    VoidInput enable{this, "enable", "Enable the motor"};
    VoidInput disable{this, "disable", "Disable the motor"};
    VoidInput start{this, "start", "Start the motor"};

    CalibrationCommands calibrationCtrl{};

    VoidInput stop{this, "stop", "Stop the motor"};
    VoidInput emergencyStop{this, "emergencyStop", "Emergency stop motor"};
    VoidInput resetError{this, "resetError", "Reset error state"};

    ScalarPushInput<ChimeraTK::Boolean> enableFullStepping{this, "enableFullStepping", "",
        "Enables full-stepping mode of the motor driver, i.e., it will only stop on full steps"};
    ScalarPushInput<ChimeraTK::Boolean> enableAutostart{
        this, "enableAutostart", "", "Sets the autostart flag of the motor driver"};
  };

  /********************************************************************************************************************/

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

  /********************************************************************************************************************/

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

  /********************************************************************************************************************/

  /// Control of the software limits
  struct SoftwareLimitCtrl : public VariableGroup {
    using VariableGroup::VariableGroup;

    ScalarPushInput<ChimeraTK::Boolean> enable{this, "enable", "", "Enable SW limits"};
    ScalarPushInput<float> maxPosition{this, "maxPosition", "", "Positive SW position limit"};
    ScalarPushInput<float> minPosition{this, "minPosition", "", "Negative SW position limit"};
    ScalarPushInput<int> maxPositionInSteps{this, "maxPositionInSteps", "", "Positive SW position limit"};
    ScalarPushInput<int> minPositionInSteps{this, "minPositionInSteps", "", "Negative SW position limit"};
  };

  /********************************************************************************************************************/

  /// User-definable limits
  struct UserLimits : public VariableGroup {
    using VariableGroup::VariableGroup;

    ScalarPushInput<double> current{this, "current", "A", "User current limit for the motor"};
    ScalarPushInput<double> speed{this, "speed", "", "User speed limit for the motor"};
  };

  /********************************************************************************************************************/

  /// Signals triggering the dummy motor
  struct DummySignals : public VariableGroup {
    using VariableGroup::VariableGroup;

    bool dummyMustStop{false};
    VoidOutput dummyMotorTrigger{
        this, "dummyMotorTrigger", "Triggers the dummy motor module after writing to a control input"};
    VoidOutput dummyMotorStop{this, "dummyMotorStop", "Stops the dummy motor"};

    void update();
  };

  /********************************************************************************************************************/

  /**
   *  @class ControlInputHandlerImpl
   *  @details Contains the implementation of the ControlInputHandler as a template so it can be used
   *           with a specific motor and set of inputs.
   */
  class ControlInputHandler : public ApplicationModule {
   public:
    ControlInputHandler(ModuleGroup* owner, const std::string& name, const std::string& description,
        std::shared_ptr<Motor> motor, const std::string& triggerPath, DeviceModule* deviceModule);

    void prepare() override;
    void mainLoop() override;

   private:
    void createFunctionMap();
    void appendCalibrationToMap();
    void writeRecoveryValues();
    /**
     * A map between the TransferElementID of a PV and the associated
     * function of the MotorDriverCard library. This allows to pass on
     * the changed PV to the library by the ID returned from readAny().
     */
    using FunctionMapEntry = std::tuple<bool, std::string, std::function<std::string(void)>>;
    std::map<TransferElementID, FunctionMapEntry> _funcMap;
    void addMapping(
        TransferElementAbstractor& element, bool writeOnRecovery, const std::function<std::string(void)>& func);

    VoidInput deviceBecameFunctional;
    VoidInput trigger;

    MotorControl control{this, "Control", "Control words of the motor", {"MOTOR"}};
    PositionSetpoint positionSetpoint{this, "PositionSetpoint", "Position setpoints", {"MOTOR"}};
    UserLimits userLimits{this, "UserLimits", "User-definable limits", {"MOTOR"}};
    SoftwareLimitCtrl swLimits{this, "SwLimits", "Control data of SW limits", {"MOTOR"}};
    ReferenceSettings referenceSettings{
        this, "ReferenceSettings", "Settings to define the position reference", {"MOTOR"}};
    DummySignals dummySignals{this, "DummySignals", " Signals triggering the dummy motor", {"DUMMY"}};
    // CalibrationCommands _calibrationCommands;
    ScalarOutput<std::string> message{
        this, "../StatusPropagator/message", "n/a", "Message for user notification from ControlInput module"};
    ScalarOutput<std::string> motorState{this, "../StatusPropagator/motorState", "n/a", "State of motor control"};

    // Callbacks for the BasiStepperMotor
    std::string enableCallback();
    std::string disableCallback();
    std::string startCallback();

    // Callbacks for the LinearStepperMotor
    std::string calibrateCallback();
    std::string determineToleranceCallback();

    // ReadAnyGroup _inputGroup{};
    std::shared_ptr<Motor> _motor;

  }; /* class ControlInputHandler */
} // namespace ChimeraTK::MotorDriver
