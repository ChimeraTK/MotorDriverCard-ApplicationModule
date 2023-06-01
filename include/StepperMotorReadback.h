// SPDX-FileCopyrightText: Deutsches Elektronen-Synchrotron DESY, MSK, ChimeraTK Project <chimeratk-support@desy.de>
// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "ExecutionTimer.h"
#include "Motor.h"

#include <ChimeraTK/ApplicationCore/ApplicationCore.h>
#include <ChimeraTK/MotorDriverCard/StepperMotor.h>
#include <ChimeraTK/ReadAnyGroup.h>

#include <functional>

namespace ChimeraTK::MotorDriver {
  /**
   *  Position data
   */
  struct Position : public VariableGroup {
    using VariableGroup::VariableGroup;

    ScalarOutput<float> actualValue{this, "actualValue", "", "Actual position"};
    ScalarOutput<int> actualValueInSteps{this, "actualValueInSteps", "", "Actual position ]"};

    ScalarOutput<double> encoderReadback{this, "encoder", "", "Encoder readback"};

    ScalarOutput<float> targetValue{this, "targetValue", "", "Readback of the target position"};
    ScalarOutput<int> targetValueInSteps{this, "targetValueInSteps", "", "Readback of target position"};
  };

  /********************************************************************************************************************/

  /**
   * Limit data
   */
  struct Limit : public VariableGroup {
    using VariableGroup::VariableGroup;

    ScalarOutput<double> userValue{this, "userValue", "", "Speed limit set for the motor"};
    ScalarOutput<double> maxValue{this, "maxValue", "", "Maximum velocity of the motor"};
  };

  /********************************************************************************************************************/

  /**
   * Motor status information
   */
  struct MotorStatus : public VariableGroup {
    using VariableGroup::VariableGroup;

    ScalarOutput<int> isEnabled{this, "isEnabled", "", "Motor current enable status"};
    ScalarOutput<int> calibrationMode{this, "calibrationMode", "", "Current calibration mode"};
    ScalarOutput<int> isIdle{
        this, "isIdle", "", "Flags if system is idle and a movement or calibration can be started"};

    ScalarOutput<std::string> state{this, "state", "", "State of the motor driver"};
    ScalarOutput<int> errorId{this, "errorId", "", "Error ID of the motor driver"};

    ScalarOutput<int> isFullStepping{
        this, "isFullStepping", "", "Flags if full-stepping mode of the driver is active."};
    ScalarOutput<int> autostartEnabled{this, "autostartEnabled", "", "Flags if autostart mode is active"};

    ScalarOutput<unsigned int> encoderReadoutMode{this, "encoderReadoutMode", "", "Encoder readout mode."};
  };

  /********************************************************************************************************************/

  /**
   * Data for SW-defined position limits
   */
  struct SoftwareLimitStat : public VariableGroup {
    using VariableGroup::VariableGroup;

    ScalarOutput<int> isEnabled{this, "isEnabled", "", "SW limit feature enabled"};

    ScalarOutput<float> maxPosition{this, "maxPosition", "", "Max. SW position limit"};
    ScalarOutput<float> minPosition{this, "minPosition", "", "Min. SW position limit"};
    ScalarOutput<int> maxPositionInSteps{this, "maxPositionInSteps", "steps", "Max. SW position limit"};
    ScalarOutput<int> minPositionInSteps{this, "minPositionInSteps", "steps", "Min. SW position limit"};
  };

  /********************************************************************************************************************/

  /**
   * VariableGroup describing the status of a reference switch
   */
  struct ReferenceSwitch : public VariableGroup {
    using VariableGroup::VariableGroup;

    ScalarOutput<int> positionInSteps{this, "positionInSteps", "steps", "Position of the positive reference switch"};
    ScalarOutput<float> position{this, "position", "", "Position of the positive reference switch"};
    ScalarOutput<float> tolerance{this, "tolerance", "", "Tolerance of the calibrated positive end switch position."};
    ScalarOutput<int> isActive{this, "isActive", "", "Status of negative end switch"};
  };

  /********************************************************************************************************************/

  /**
   *  @class ReadbackHandler
   *  @brief Base application module for cyclically reading data from the motor driver card HW.
   */
  class ReadbackHandler : public ApplicationModule {
   public:
    ReadbackHandler(std::shared_ptr<Motor> motor, ModuleGroup* owner, const std::string& name,
        const std::string& description, const std::string& triggerPath);

    VoidInput trigger{};

    // Diagnostics
    ScalarOutput<float> actualCycleTime{
        this, "actualCycleTime", "ms", "Actual cycle time by which the HW is being read", {"MOT_DIAG"}};
    ScalarOutput<float> actualReceiveTime{this, "actualReceiveTime", "ms",
        "Actual time required to read all variables in this module from the HW.", {"MOT_DIAG"}};

    struct : public VariableGroup {
      using VariableGroup::VariableGroup;

      ScalarOutput<std::string> message{this, "message", "", ""};
      ScalarOutput<int32_t> status{this, "status", "", ""};
    } deviceError{this, "../Device", "", {"MOTOR"}};

    void mainLoop() override;

    Position position{this, "position", "Position data", {"MOTOR"}};
    Limit speedLimit{this, "speedLimit", "Speed data", {"MOTOR"}};
    Limit currentLimit{this, "currentLimit", "Current data", {"MOTOR"}};
    MotorStatus status{this, "status", "Status data of the motor driver", {"MOTOR"}};
    SoftwareLimitStat swLimits{this, "swLimits", "Status data of SW limits", {"MOTOR"}};
    ReferenceSwitch positiveEndSwitch{};
    ReferenceSwitch negativeEndSwitch{};

   protected:
    std::function<void(void)> _readbackFunction;

   private:
    void readback();
    void readEndSwitchData();
    /**
     * Some variables are only calculated at startup
     * of the MotorDriverCard lib, so read them only once
     */
    void readConstData();

    std::shared_ptr<Motor> _motor;
    ::detail::ExecutionTimer<> _execTimer{};
    ::detail::ExecutionTimer<> _receiveTimer{};
    unsigned int _spiErrorCounter{0};

    /// Hack to prevent throwing when the motor dummy is used
    const bool _motorIsDummy;
    [[nodiscard]] bool motorIsDummy() const;

    void tryMotorRenew();
    void tryReadingFromMotor();
    void setStatusFromException(const std::exception& e);
  };

} // namespace ChimeraTK::MotorDriver
