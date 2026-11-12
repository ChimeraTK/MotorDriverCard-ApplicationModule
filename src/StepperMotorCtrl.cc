// SPDX-FileCopyrightText: Deutsches Elektronen-Synchrotron DESY, MSK, ChimeraTK Project <chimeratk-support@desy.de>
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "StepperMotorCtrl.h"

#include <ChimeraTK/ApplicationCore/DeviceManager.h>
#include <ChimeraTK/ApplicationCore/Logger.h>
#include <ChimeraTK/MotorDriverCard/StepperMotorUtil.h>

#include <sstream>
#include <utility>

namespace detail {
  constexpr bool WRITE_ON_RECOVERY{true};
  constexpr bool SKIP_ON_RECOVERY{false};

  /********************************************************************************************************************/

  /// Create a log message describing the outcome of a command that should have let the motor move.
  std::string describeRejectedMovement(const std::string& command, const std::string& reason) {
    return command + " was not executed, motor will not move: " + reason;
  }

  /********************************************************************************************************************/

  /// Log a command that was accepted and is about to be passed on to the motor driver card.
  /// The logger stream is passed by forwarding reference, since it is bound to a temporary created by the caller.
  template<typename LogStream>
  void logAcceptedMovement(
      LogStream&& logStream, const ChimeraTK::MotorDriver::Motor& motor, const std::string& command) {
    logStream << "Executing " << command << " for motor " << motor.toString() << std::endl;
  }
} // namespace detail

namespace ChimeraTK::MotorDriver {

  ControlInputHandler::ControlInputHandler(ModuleGroup* owner, const std::string& name, const std::string& description,
      std::shared_ptr<Motor> motor, const std::string& triggerPath, DeviceModule* deviceModule)
  : ApplicationModule(owner, name, description),
    trigger{this, triggerPath, "Trigger to initiate read-out from HW", std::unordered_set<std::string>{"MOT_TRIG"}},
    _motor(std::move(motor)) {
    // If motor has HW reference switches,
    // calibration is supported
    if(_motor->getMotorParameters().motorType == StepperMotorType::LINEAR ||
        _motor->getMotorParameters().motorType == StepperMotorType::ROTARY) {
      control.calibrationCtrl = CalibrationCommands{&control, ".", "Calibration commands", {"MOTOR"}};
    }
    deviceBecameFunctional =
        VoidInput(this, deviceModule->getDeviceManager().deviceBecameFunctional.getModel().getFullyQualifiedPath(), "");

    debug() << "Created ControlInputHandler for motor " << _motor->toString() << ", calibration "
            << (_motor->getMotorParameters().motorType == StepperMotorType::LINEAR ||
                           _motor->getMotorParameters().motorType == StepperMotorType::ROTARY ?
                       "supported" :
                       "not supported (no HW reference switches)")
            << std::endl;
  }

  /********************************************************************************************************************/

  void ControlInputHandler::addMapping(
      TransferElementAbstractor& element, bool writeOnRecovery, const std::function<std::string(void)>& function) {
    _funcMap[element.getId()] = {writeOnRecovery, element.getName(), function};
    trace() << "Registered command " << element.getName() << " (" << (writeOnRecovery ? "written" : "skipped")
            << " on device recovery)" << std::endl;
  }

  /********************************************************************************************************************/

  /**
   * All handlers below follow the same pattern: on rejection the reason is returned as notification message for the
   * user and additionally logged with 'error' severity. Especially the paths that prevent the motor from moving must be
   * visible in the log in any case, since the notification message is transient ("no historical information").
   */

  void ControlInputHandler::createFunctionMap() {
    addMapping(control.enable, ::detail::SKIP_ON_RECOVERY, [this] { return enableCallback(); });
    addMapping(control.disable, ::detail::SKIP_ON_RECOVERY, [this] { return disableCallback(); });
    addMapping(control.start, ::detail::SKIP_ON_RECOVERY, [this] { return startCallback(); });
    addMapping(control.stop, ::detail::SKIP_ON_RECOVERY, [this] {
      if(control.stop) {
        ::detail::logAcceptedMovement(info(), *_motor, "stop");
        _motor->get()->stop();
        motorState.writeIfDifferent(_motor->get()->getState());
      }
      return "";
    });
    addMapping(control.emergencyStop, ::detail::SKIP_ON_RECOVERY, [this] {
      if(control.emergencyStop) {
        warning() << "Executing emergencyStop for motor " << _motor->toString() << std::endl;
        _motor->get()->emergencyStop();
        motorState.writeIfDifferent(_motor->get()->getState());
      }
      return "";
    });
    addMapping(control.resetError, ::detail::SKIP_ON_RECOVERY, [this] {
      if(control.resetError) {
        ::detail::logAcceptedMovement(info(), *_motor, "resetError");
        _motor->get()->resetError();
        motorState.writeIfDifferent(_motor->get()->getState());
      }

      return "";
    });
    addMapping(control.enableAutostart, ::detail::WRITE_ON_RECOVERY, [this] {
      info() << "Setting autostart mode of motor " << _motor->toString() << " to " << std::boolalpha
             << static_cast<bool>(control.enableAutostart) << std::endl;
      _motor->get()->setAutostart(control.enableAutostart);
      motorState.writeIfDifferent(_motor->get()->getState());
      return "";
    });
    addMapping(control.enableFullStepping, ::detail::WRITE_ON_RECOVERY, [this] {
      info() << "Setting full-stepping mode of motor " << _motor->toString() << " to " << std::boolalpha
             << static_cast<bool>(control.enableFullStepping) << std::endl;
      _motor->get()->enableFullStepping(control.enableFullStepping);
      return "";
    });

    addMapping(positionSetpoint.positionInSteps, ::detail::SKIP_ON_RECOVERY, [this] {
      auto code = _motor->get()->setTargetPositionInSteps(positionSetpoint.positionInSteps);
      motorState.writeIfDifferent(_motor->get()->getState());
      if(code != ExitStatus::SUCCESS) {
        auto reason = ChimeraTK::MotorDriver::toString(code);
        error() << ::detail::describeRejectedMovement("Setting target position to " +
                           std::to_string(static_cast<int>(positionSetpoint.positionInSteps)) + " steps",
                       reason)
                << std::endl;
        return "Could not set target position: " + reason;
      }

      info() << "Set target position of motor " << _motor->toString() << " to "
             << static_cast<int>(positionSetpoint.positionInSteps) << " steps, state: " << _motor->get()->getState()
             << std::endl;

      return std::string();
    });

    addMapping(positionSetpoint.position, ::detail::SKIP_ON_RECOVERY, [this] {
      auto code = _motor->get()->setTargetPosition(positionSetpoint.position);
      motorState.writeIfDifferent(_motor->get()->getState());
      if(code != ExitStatus::SUCCESS) {
        auto reason = ChimeraTK::MotorDriver::toString(code);
        std::ostringstream command;
        command << "Setting target position to " << static_cast<float>(positionSetpoint.position) << " units";
        error() << ::detail::describeRejectedMovement(command.str(), reason) << std::endl;
        return "Could not set target position: " + reason;
      }

      info() << "Set target position of motor " << _motor->toString() << " to "
             << static_cast<float>(positionSetpoint.position) << " units, state: " << _motor->get()->getState()
             << std::endl;

      return std::string();
    });
    addMapping(positionSetpoint.relativePositionInSteps, ::detail::SKIP_ON_RECOVERY, [this] {
      auto code = _motor->get()->moveRelativeInSteps(positionSetpoint.relativePositionInSteps);
      motorState.writeIfDifferent(_motor->get()->getState());
      if(code != ExitStatus::SUCCESS) {
        auto reason = ChimeraTK::MotorDriver::toString(code);
        error() << ::detail::describeRejectedMovement("Relative movement by " +
                           std::to_string(static_cast<int>(positionSetpoint.relativePositionInSteps)) + " steps",
                       reason)
                << std::endl;
        return "Could not set relative position: " + reason;
      }

      info() << "Started relative movement of motor " << _motor->toString() << " by "
             << static_cast<int>(positionSetpoint.relativePositionInSteps) << " steps from position "
             << _motor->get()->getCurrentPositionInSteps() << " steps" << std::endl;

      return std::string();
    });
    addMapping(positionSetpoint.relativePosition, ::detail::SKIP_ON_RECOVERY, [this] {
      auto code = _motor->get()->moveRelative(positionSetpoint.relativePosition);
      motorState.writeIfDifferent(_motor->get()->getState());
      if(code != ExitStatus::SUCCESS) {
        auto reason = ChimeraTK::MotorDriver::toString(code);
        std::ostringstream command;
        command << "Relative movement by " << static_cast<float>(positionSetpoint.relativePosition) << " units";
        error() << ::detail::describeRejectedMovement(command.str(), reason) << std::endl;
        return "Could not set relative position: " + reason;
      }

      info() << "Started relative movement of motor " << _motor->toString() << " by "
             << static_cast<float>(positionSetpoint.relativePosition) << " units from position "
             << _motor->get()->getCurrentPositionInSteps() << " steps" << std::endl;

      return std::string();
    });

    addMapping(referenceSettings.positionInSteps, ::detail::SKIP_ON_RECOVERY, [this] {
      auto code = _motor->get()->setActualPositionInSteps(referenceSettings.positionInSteps);
      motorState.writeIfDifferent(_motor->get()->getState());
      if(code != ExitStatus::SUCCESS) {
        auto reason = ChimeraTK::MotorDriver::toString(code);
        error() << "Could not set reference position in steps to "
                << static_cast<int>(referenceSettings.positionInSteps) << " for motor " << _motor->toString() << ": "
                << reason << std::endl;
        return "Could not set reference position in steps: " + reason;
      }

      info() << "Set actual position of motor " << _motor->toString() << " to "
             << static_cast<int>(referenceSettings.positionInSteps) << " steps" << std::endl;

      return std::string();
    });
    addMapping(referenceSettings.position, ::detail::SKIP_ON_RECOVERY, [this] {
      auto code = _motor->get()->setActualPosition(referenceSettings.position);
      motorState.writeIfDifferent(_motor->get()->getState());
      if(code != ExitStatus::SUCCESS) {
        auto reason = ChimeraTK::MotorDriver::toString(code);
        error() << "Could not set reference position to " << static_cast<float>(referenceSettings.position)
                << " for motor " << _motor->toString() << ": " << reason << std::endl;
        return "Could not set reference position: " + reason;
      }

      info() << "Set actual position of motor " << _motor->toString() << " to "
             << static_cast<float>(referenceSettings.position) << " units" << std::endl;

      return std::string();
    });
    addMapping(referenceSettings.encoderPosition, ::detail::SKIP_ON_RECOVERY, [this] {
      auto code = _motor->get()->setActualEncoderPosition(referenceSettings.encoderPosition);
      if(code != ExitStatus::SUCCESS) {
        auto reason = ChimeraTK::MotorDriver::toString(code);
        error() << "Could not set reference encoder position to "
                << static_cast<int32_t>(referenceSettings.encoderPosition) << " for motor " << _motor->toString()
                << ": " << reason << std::endl;
        return "Could not set reference encoder position: " + reason;
      }

      info() << "Set actual encoder position of motor " << _motor->toString() << " to "
             << static_cast<int32_t>(referenceSettings.encoderPosition) << std::endl;

      return std::string();
    });
    addMapping(referenceSettings.axisTranslationInSteps, ::detail::SKIP_ON_RECOVERY, [this] {
      auto code = _motor->get()->translateAxisInSteps(referenceSettings.axisTranslationInSteps);
      if(code != ExitStatus::SUCCESS) {
        auto reason = ChimeraTK::MotorDriver::toString(code);
        error() << "Could not translate axis by " << static_cast<int>(referenceSettings.axisTranslationInSteps)
                << " steps for motor " << _motor->toString() << ": " << reason << std::endl;
        return "Could not set axis translation: " + reason;
      }

      info() << "Translated axis of motor " << _motor->toString() << " by "
             << static_cast<int>(referenceSettings.axisTranslationInSteps) << " steps" << std::endl;

      return std::string();
    });
    addMapping(referenceSettings.axisTranslation, ::detail::SKIP_ON_RECOVERY, [this] {
      auto code = _motor->get()->translateAxis(referenceSettings.axisTranslation);
      if(code != ExitStatus::SUCCESS) {
        auto reason = ChimeraTK::MotorDriver::toString(code);
        error() << "Could not translate axis by " << static_cast<float>(referenceSettings.axisTranslation)
                << " units for motor " << _motor->toString() << ": " << reason << std::endl;
        return "Could not set axis translation: " + reason;
      }

      info() << "Translated axis of motor " << _motor->toString() << " by "
             << static_cast<float>(referenceSettings.axisTranslation) << " units" << std::endl;

      return std::string();
    });

    addMapping(swLimits.enable, ::detail::WRITE_ON_RECOVERY, [this] {
      auto code = _motor->get()->setSoftwareLimitsEnabled(swLimits.enable);
      if(code != ExitStatus::SUCCESS) {
        auto reason = ChimeraTK::MotorDriver::toString(code);
        error() << "Could not " << (static_cast<bool>(swLimits.enable) ? "enable" : "disable")
                << " software limits for motor " << _motor->toString() << ": " << reason << std::endl;
        return "Could not enable software limits: " + reason;
      }

      info() << (static_cast<bool>(swLimits.enable) ? "Enabled" : "Disabled") << " software limits of motor "
             << _motor->toString() << std::endl;

      return std::string();
    });
    addMapping(swLimits.maxPosition, ::detail::WRITE_ON_RECOVERY, [this] {
      auto code = _motor->get()->setMaxPositionLimit(swLimits.maxPosition);
      if(code != ExitStatus::SUCCESS) {
        auto reason = ChimeraTK::MotorDriver::toString(code);
        error() << "Could not set max position limit to " << static_cast<float>(swLimits.maxPosition) << " for motor "
                << _motor->toString() << ": " << reason << std::endl;
        return "Could not set max position limit: " + reason;
      }

      info() << "Set max position limit of motor " << _motor->toString() << " to "
             << static_cast<float>(swLimits.maxPosition) << " units" << std::endl;

      return std::string();
    });
    addMapping(swLimits.minPosition, ::detail::WRITE_ON_RECOVERY, [this] {
      auto code = _motor->get()->setMinPositionLimit(swLimits.minPosition);
      if(code != ExitStatus::SUCCESS) {
        auto reason = ChimeraTK::MotorDriver::toString(code);
        error() << "Could not set min position limit to " << static_cast<float>(swLimits.minPosition) << " for motor "
                << _motor->toString() << ": " << reason << std::endl;
        return "Could not set min position limits: " + reason;
      }

      info() << "Set min position limit of motor " << _motor->toString() << " to "
             << static_cast<float>(swLimits.minPosition) << " units" << std::endl;

      return std::string();
    });
    addMapping(swLimits.maxPositionInSteps, ::detail::SKIP_ON_RECOVERY, [this] {
      auto code = _motor->get()->setMaxPositionLimitInSteps(swLimits.maxPositionInSteps);
      if(code != ExitStatus::SUCCESS) {
        auto reason = ChimeraTK::MotorDriver::toString(code);
        error() << "Could not set max position limit to " << static_cast<int>(swLimits.maxPositionInSteps)
                << " steps for motor " << _motor->toString() << ": " << reason << std::endl;
        return "Could not set max position limits: " + reason;
      }

      info() << "Set max position limit of motor " << _motor->toString() << " to "
             << static_cast<int>(swLimits.maxPositionInSteps) << " steps" << std::endl;

      return std::string();
    });
    addMapping(swLimits.minPositionInSteps, ::detail::SKIP_ON_RECOVERY, [this] {
      auto code = _motor->get()->setMinPositionLimitInSteps(swLimits.minPositionInSteps);
      if(code != ExitStatus::SUCCESS) {
        auto reason = ChimeraTK::MotorDriver::toString(code);
        error() << "Could not set min position limit to " << static_cast<int>(swLimits.minPositionInSteps)
                << " steps for motor " << _motor->toString() << ": " << reason << std::endl;
        return "Could not set min position limits: " + reason;
      }

      info() << "Set min position limit of motor " << _motor->toString() << " to "
             << static_cast<int>(swLimits.minPositionInSteps) << " steps" << std::endl;

      return std::string();
    });

    addMapping(userLimits.current, ::detail::WRITE_ON_RECOVERY, [this] {
      try {
        auto code = _motor->get()->setUserCurrentLimit(userLimits.current);
        motorState.writeIfDifferent(_motor->get()->getState());
        if(code != ExitStatus::SUCCESS) {
          auto reason = ChimeraTK::MotorDriver::toString(code);
          error() << "Could not set user current limit to " << static_cast<double>(userLimits.current)
                  << " A for motor " << _motor->toString() << ": " << reason << std::endl;
          return "Could not set user current limits: " + reason;
        }
      }
      catch(ChimeraTK::logic_error& e) {
        // Do nothing, this is a dummy then
        debug() << "Ignoring user current limit setting for motor " << _motor->toString()
                << ", no current limit support: " << e.what() << std::endl;
      }
      return std::string();
    });
    addMapping(userLimits.speed, ::detail::WRITE_ON_RECOVERY, [this] {
      try {
        auto code = _motor->get()->setUserSpeedLimit(userLimits.speed);
        if(code != ExitStatus::SUCCESS) {
          auto reason = ChimeraTK::MotorDriver::toString(code);
          error() << "Could not set user speed limit to " << static_cast<double>(userLimits.speed) << " for motor "
                  << _motor->toString() << ": " << reason << std::endl;
          return "Could not set user speed limits: " + reason;
        }
      }
      catch(ChimeraTK::logic_error& e) {
        // Do nothing, this is a dummy then
        debug() << "Ignoring user speed limit setting for motor " << _motor->toString()
                << ", no speed limit support: " << e.what() << std::endl;
      }
      return std::string();
    });
  }

  /********************************************************************************************************************/

  void ControlInputHandler::prepare() {
    createFunctionMap();

    if(_motor->getMotorParameters().motorType == StepperMotorType::LINEAR ||
        _motor->getMotorParameters().motorType == StepperMotorType::ROTARY) {
      appendCalibrationToMap();
    }
    else {
      debug() << "Motor " << _motor->toString() << " has no HW reference switches, calibration commands are unavailable"
              << std::endl;
    }
  }

  /********************************************************************************************************************/

  void ControlInputHandler::writeRecoveryValues() {
    info() << "Device became functional, writing recovery values for motor " << _motor->toString() << std::endl;
    for(auto& entry : _funcMap) {
      auto& [inRecovery, name, call] = entry.second;
      if(inRecovery) {
        debug() << "Writing recovery value " << name << " for motor " << _motor->toString() << std::endl;
        std::ignore = call();
      }
    }
  }

  /********************************************************************************************************************/

  void ControlInputHandler::mainLoop() {
    auto inputGroup = readAnyGroup();

    // before anything else, wait for the deviceBecameFunctional trigger
    // then flush out all values that are written during recovery

    motorState.setDataValidity(DataValidity::faulty);
    motorState.write();

    info() << "Waiting for the motor device of " << _motor->toString() << " to become functional" << std::endl;
    inputGroup.readUntil(deviceBecameFunctional.getId());
    writeRecoveryValues();

    // Write once to propagate inital values
    motorState.setDataValidity(DataValidity::ok);
    motorState.writeIfDifferent(_motor->get()->getState());
    dummySignals.writeAll();
    message.write();

    debug() << "ControlInputHandler for motor " << _motor->toString() << " entered main loop" << std::endl;

    while(true) {
      try {
        std::string notificationMessage;

        auto changedVarId = inputGroup.readAny();
        if(changedVarId == deviceBecameFunctional.getId()) {
          writeRecoveryValues();
          // Flush state of notifications
          message.setAndWrite("");

          continue;
        }

        // Some state changes are only triggered by the readout of the state, so we regularly
        // read it out and publish any changes
        if(changedVarId == trigger.getId() && _motor->isOpen()) {
          motorState.writeIfDifferent(_motor->get()->getState());
          continue;
        }

        if(not _motor->isOpen()) {
          auto name = _funcMap.contains(changedVarId) ? std::get<1>(_funcMap.at(changedVarId)) : "<unknown>";
          error() << ::detail::describeRejectedMovement("Command '" + name + "'", "the motor device is in recovery")
                  << std::endl;
          notificationMessage = "Motor device is in recovery, not doing control step";
        }
        else {
          // FIXME Keep this try/catch only as long as we rely on the dummy for tests
          try {
            auto& [_, name, call] = _funcMap.at(changedVarId);
            trace() << "Executing command " << name << " for motor " << _motor->toString() << std::endl;
            notificationMessage = call();
          }
          catch(ChimeraTK::logic_error& e) {
            auto name = _funcMap.contains(changedVarId) ? std::get<1>(_funcMap.at(changedVarId)) : "<unknown>";
            error() << ::detail::describeRejectedMovement(
                           "Command '" + name + "'", "exception from motor driver card: " + std::string(e.what()))
                    << std::endl;
            notificationMessage = "Exception: " + std::string(e.what());
          }
        }

        dummySignals.dummyMotorStop.writeIfDifferent(control.stop || control.emergencyStop);
        dummySignals.dummyMotorTrigger++;
        dummySignals.dummyMotorTrigger.write();
        message.setAndWrite(notificationMessage);
      }
      catch(std::exception& e) {
        // Comes from the state readout. Do nothing here. either it was a single glitch
        // or the readout module will see it as well and start the recovery procedure
        debug() << "Exception while reading the state of motor " << _motor->toString()
                << ", relying on readback module for recovery: " << e.what() << std::endl;
      }
    }
  }

  /********************************************************************************************************************/

  std::string ControlInputHandler::enableCallback() {
    if(control.enable) {
      ::detail::logAcceptedMovement(info(), *_motor, "enable");
      _motor->get()->setEnabled(true);
      motorState.writeIfDifferent(_motor->get()->getState());
    }
    else {
      debug() << "Ignoring 'enable' with value 0 for motor " << _motor->toString() << std::endl;
    }
    return "";
  }

  /********************************************************************************************************************/

  std::string ControlInputHandler::disableCallback() {
    if(control.disable) {
      ::detail::logAcceptedMovement(info(), *_motor, "disable");
      _motor->get()->setEnabled(false);
      motorState.writeIfDifferent(_motor->get()->getState());
    }
    else {
      debug() << "Ignoring 'disable' with value 0 for motor " << _motor->toString() << std::endl;
    }

    return {};
  }

  /********************************************************************************************************************/

  std::string ControlInputHandler::startCallback() {
    if(control.start) {
      if(_motor->get()->isSystemIdle()) {
        ::detail::logAcceptedMovement(info(), *_motor, "start");
        _motor->get()->start();
        motorState.writeIfDifferent(_motor->get()->getState());
      }
      else {
        auto state = _motor->get()->getState();
        error() << ::detail::describeRejectedMovement(
                       "start", "motor is not in IDLE state but in state '" + state + "'")
                << " (motor " << _motor->toString() << ")" << std::endl;
        return "WARNING: Called startMotor while motor is not in IDLE state.";
      }
    }
    else {
      debug() << "Ignoring 'start' with value 0 for motor " << _motor->toString() << std::endl;
    }

    return {};
  }

  /********************************************************************************************************************/

  void ControlInputHandler::appendCalibrationToMap() {
    addMapping(
        control.calibrationCtrl.calibrateMotor, ::detail::SKIP_ON_RECOVERY, [this] { return calibrateCallback(); });
    addMapping(control.calibrationCtrl.determineTolerance, ::detail::SKIP_ON_RECOVERY,
        [this] { return determineToleranceCallback(); });
  }

  /********************************************************************************************************************/

  std::string ControlInputHandler::calibrateCallback() {
    if(!_motor->get()->hasHWReferenceSwitches()) {
      debug() << "Ignoring calibration request for motor " << _motor->toString()
              << ", no HW reference switches available" << std::endl;
      return {};
    }

    if(control.calibrationCtrl.calibrateMotor) {
      if(_motor->get()->isSystemIdle()) {
        ::detail::logAcceptedMovement(info(), *_motor, "calibrate");
        auto code = _motor->get()->calibrate();
        if(code != ExitStatus::SUCCESS) {
          auto reason = ChimeraTK::MotorDriver::toString(code);
          error() << ::detail::describeRejectedMovement("calibrate", reason) << " (motor " << _motor->toString() << ")"
                  << std::endl;
          return "Could not calibrate motor: " + reason;
        }
      }
      else {
        auto state = _motor->get()->getState();
        error() << ::detail::describeRejectedMovement(
                       "calibrate", "motor is not in IDLE state but in state '" + state + "'")
                << " (motor " << _motor->toString() << ")" << std::endl;
        return "WARNING: Called calibrateMotor while motor is not in IDLE state.";
      }
    }
    else {
      debug() << "Ignoring 'calibrate' with value 0 for motor " << _motor->toString() << std::endl;
    }

    return {};
  }

  /********************************************************************************************************************/

  std::string ControlInputHandler::determineToleranceCallback() {
    if(!_motor->get()->hasHWReferenceSwitches()) {
      debug() << "Ignoring determineTolerance request for motor " << _motor->toString()
              << ", no HW reference switches available" << std::endl;
      return "";
    }

    if(control.calibrationCtrl.determineTolerance) {
      if(_motor->get()->isSystemIdle()) {
        ::detail::logAcceptedMovement(info(), *_motor, "determineTolerance");
        auto code = _motor->get()->determineTolerance();
        if(code != ExitStatus::SUCCESS) {
          auto reason = ChimeraTK::MotorDriver::toString(code);
          error() << ::detail::describeRejectedMovement("determineTolerance", reason) << " (motor "
                  << _motor->toString() << ")" << std::endl;
          return "Could not determine endswitch tolerance: " + reason;
        }
      }
      else {
        auto state = _motor->get()->getState();
        error() << ::detail::describeRejectedMovement(
                       "determineTolerance", "motor is not in IDLE state but in state '" + state + "'")
                << " (motor " << _motor->toString() << ")" << std::endl;
        return "WARNING: Called determineTolerance while motor is not in IDLE state.";
      }
    }
    else {
      debug() << "Ignoring 'determineTolerance' with value 0 for motor " << _motor->toString() << std::endl;
    }
    return "";
  }
} // namespace ChimeraTK::MotorDriver
