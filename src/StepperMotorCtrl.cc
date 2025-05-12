// SPDX-FileCopyrightText: Deutsches Elektronen-Synchrotron DESY, MSK, ChimeraTK Project <chimeratk-support@desy.de>
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "StepperMotorCtrl.h"

#include <ChimeraTK/ApplicationCore/DeviceManager.h>
#include <ChimeraTK/MotorDriverCard/StepperMotorUtil.h>

#include <utility>

namespace detail {
  constexpr bool WRITE_ON_RECOVERY{true};
  constexpr bool SKIP_ON_RECOVERY{false};
} // namespace detail

namespace ChimeraTK::MotorDriver {

  ControlInputHandler::ControlInputHandler(ModuleGroup* owner, const std::string& name, const std::string& description,
      std::shared_ptr<Motor> motor, const std::string& triggerPath, DeviceModule* deviceModule)
  : ApplicationModule(owner, name, description),
    trigger{this, triggerPath, "Trigger to initiate read-out from HW", std::unordered_set<std::string>{"MOT_TRIG"}},
    _motor(std::move(motor)) {
    // If motor has HW reference switches,
    // calibration is supported
    if(_motor->getMotorParameters().motorType == StepperMotorType::LINEAR) {
      control.calibrationCtrl = CalibrationCommands{&control, ".", "Calibration commands", {"MOTOR"}};
    }
    deviceBecameFunctional =
        VoidInput(this, deviceModule->getDeviceManager().deviceBecameFunctional.getModel().getFullyQualifiedPath(), "");
  }

  /********************************************************************************************************************/

  void ControlInputHandler::addMapping(
      TransferElementAbstractor& element, bool writeOnRecovery, const std::function<std::string(void)>& function) {
    _funcMap[element.getId()] = {writeOnRecovery, element.getName(), function};
  }

  void ControlInputHandler::createFunctionMap() {
    addMapping(control.enable, ::detail::SKIP_ON_RECOVERY, [this] { return enableCallback(); });
    addMapping(control.disable, ::detail::SKIP_ON_RECOVERY, [this] { return disableCallback(); });
    addMapping(control.start, ::detail::SKIP_ON_RECOVERY, [this] { return startCallback(); });
    addMapping(control.stop, ::detail::SKIP_ON_RECOVERY, [this] {
      if(control.stop) {
        _motor->get()->stop();
        motorState.writeIfDifferent(_motor->get()->getState());
      }
      return "";
    });
    addMapping(control.emergencyStop, ::detail::SKIP_ON_RECOVERY, [this] {
      if(control.emergencyStop) {
        _motor->get()->emergencyStop();
        motorState.writeIfDifferent(_motor->get()->getState());
      }
      return "";
    });
    addMapping(control.resetError, ::detail::SKIP_ON_RECOVERY, [this] {
      if(control.resetError) {
        _motor->get()->resetError();
        motorState.writeIfDifferent(_motor->get()->getState());
      }

      return "";
    });
    addMapping(control.enableAutostart, ::detail::WRITE_ON_RECOVERY, [this] {
      _motor->get()->setAutostart(control.enableAutostart);
      motorState.writeIfDifferent(_motor->get()->getState());
      return "";
    });
    addMapping(control.enableFullStepping, ::detail::WRITE_ON_RECOVERY, [this] {
      _motor->get()->enableFullStepping(control.enableFullStepping);
      return "";
    });

    addMapping(positionSetpoint.positionInSteps, ::detail::SKIP_ON_RECOVERY, [this] {
      auto code = _motor->get()->setTargetPositionInSteps(positionSetpoint.positionInSteps);
      motorState.writeIfDifferent(_motor->get()->getState());
      if(code != ExitStatus::SUCCESS) {
        return "Could not set target position: " + ChimeraTK::MotorDriver::toString(code);
      }

      return std::string();
    });
    addMapping(positionSetpoint.position, ::detail::SKIP_ON_RECOVERY, [this] {
      auto code = _motor->get()->setTargetPosition(positionSetpoint.position);
      motorState.writeIfDifferent(_motor->get()->getState());
      if(code != ExitStatus::SUCCESS) {
        return "Could not set target position: " + ChimeraTK::MotorDriver::toString(code);
      }

      return std::string();
    });
    addMapping(positionSetpoint.relativePositionInSteps, ::detail::SKIP_ON_RECOVERY, [this] {
      auto code = _motor->get()->moveRelativeInSteps(positionSetpoint.relativePositionInSteps);
      motorState.writeIfDifferent(_motor->get()->getState());
      if(code != ExitStatus::SUCCESS) {
        return "Could not set relative position: " + ChimeraTK::MotorDriver::toString(code);
      }
      return std::string();
    });
    addMapping(positionSetpoint.relativePosition, ::detail::SKIP_ON_RECOVERY, [this] {
      auto code = _motor->get()->moveRelative(positionSetpoint.relativePosition);
      motorState.writeIfDifferent(_motor->get()->getState());
      if(code != ExitStatus::SUCCESS) {
        return "Could not set relative position: " + ChimeraTK::MotorDriver::toString(code);
      }
      return std::string();
    });

    addMapping(referenceSettings.positionInSteps, ::detail::SKIP_ON_RECOVERY, [this] {
      auto code = _motor->get()->setActualPositionInSteps(referenceSettings.positionInSteps);
      motorState.writeIfDifferent(_motor->get()->getState());
      if(code != ExitStatus::SUCCESS) {
        return "Could not set reference position in steps: " + ChimeraTK::MotorDriver::toString(code);
      }
      return std::string();
    });
    addMapping(referenceSettings.position, ::detail::SKIP_ON_RECOVERY, [this] {
      auto code = _motor->get()->setActualPosition(referenceSettings.position);
      motorState.writeIfDifferent(_motor->get()->getState());
      if(code != ExitStatus::SUCCESS) {
        return "Could not set reference position: " + ChimeraTK::MotorDriver::toString(code);
      }
      return std::string();
    });
    addMapping(referenceSettings.encoderPosition, ::detail::SKIP_ON_RECOVERY, [this] {
      auto code = _motor->get()->setActualEncoderPosition(referenceSettings.encoderPosition);
      if(code != ExitStatus::SUCCESS) {
        return "Could not set reference encoder position: " + ChimeraTK::MotorDriver::toString(code);
      }

      return std::string();
    });
    addMapping(referenceSettings.axisTranslationInSteps, ::detail::SKIP_ON_RECOVERY, [this] {
      auto code = _motor->get()->translateAxisInSteps(referenceSettings.axisTranslationInSteps);
      if(code != ExitStatus::SUCCESS) {
        return "Could not set axis translation: " + ChimeraTK::MotorDriver::toString(code);
      }
      return std::string();
    });
    addMapping(referenceSettings.axisTranslation, ::detail::SKIP_ON_RECOVERY, [this] {
      auto code = _motor->get()->translateAxis(referenceSettings.axisTranslation);
      if(code != ExitStatus::SUCCESS) {
        return "Could not set axis translation: " + ChimeraTK::MotorDriver::toString(code);
      }

      return std::string();
    });

    addMapping(swLimits.enable, ::detail::WRITE_ON_RECOVERY, [this] {
      auto code = _motor->get()->setSoftwareLimitsEnabled(swLimits.enable);
      if(code != ExitStatus::SUCCESS) {
        return "Could not enable software limits: " + ChimeraTK::MotorDriver::toString(code);
      }
      return std::string();
    });
    addMapping(swLimits.maxPosition, ::detail::WRITE_ON_RECOVERY, [this] {
      auto code = _motor->get()->setMaxPositionLimit(swLimits.maxPosition);
      if(code != ExitStatus::SUCCESS) {
        return "Could not set max position limit: " + ChimeraTK::MotorDriver::toString(code);
      }
      return std::string();
    });
    addMapping(swLimits.minPosition, ::detail::WRITE_ON_RECOVERY, [this] {
      auto code = _motor->get()->setMinPositionLimit(swLimits.minPosition);
      if(code != ExitStatus::SUCCESS) {
        return "Could not set min position limits: " + ChimeraTK::MotorDriver::toString(code);
      }

      return std::string();
    });
    addMapping(swLimits.maxPositionInSteps, ::detail::SKIP_ON_RECOVERY, [this] {
      auto code = _motor->get()->setMaxPositionLimitInSteps(swLimits.maxPositionInSteps);
      if(code != ExitStatus::SUCCESS) {
        return "Could not set max position limits: " + ChimeraTK::MotorDriver::toString(code);
      }

      return std::string();
    });
    addMapping(swLimits.minPositionInSteps, ::detail::SKIP_ON_RECOVERY, [this] {
      auto code = _motor->get()->setMinPositionLimitInSteps(swLimits.minPositionInSteps);
      if(code != ExitStatus::SUCCESS) {
        return "Could not set min position limits: " + ChimeraTK::MotorDriver::toString(code);
      }
      return std::string();
    });

    addMapping(userLimits.current, ::detail::WRITE_ON_RECOVERY, [this] {
      try {
        auto code = _motor->get()->setUserCurrentLimit(userLimits.current);
        motorState.writeIfDifferent(_motor->get()->getState());
        if(code != ExitStatus::SUCCESS) {
          return "Could not set user current limits: " + ChimeraTK::MotorDriver::toString(code);
        }
      }
      catch(ChimeraTK::logic_error&) {
        // Do nothing, this is a dummy then
      }
      return std::string();
    });
    addMapping(userLimits.speed, ::detail::WRITE_ON_RECOVERY, [this] {
      try {
        auto code = _motor->get()->setUserSpeedLimit(userLimits.speed);
        if(code != ExitStatus::SUCCESS) {
          return "Could not set user speed limits: " + ChimeraTK::MotorDriver::toString(code);
        }
      }
      catch(ChimeraTK::logic_error&) {
        // Do nothing, this is a dummy then
      }
      return std::string();
    });
  }

  void ControlInputHandler::prepare() {
    createFunctionMap();

    if(_motor->getMotorParameters().motorType == StepperMotorType::LINEAR) {
      appendCalibrationToMap();
    }
  }

  /********************************************************************************************************************/

  void ControlInputHandler::writeRecoveryValues() {
    for(auto& entry : _funcMap) {
      auto& [inRecovery, _, call] = entry.second;
      if(inRecovery) {
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

    inputGroup.readUntil(deviceBecameFunctional.getId());
    writeRecoveryValues();

    // Write once to propagate inital values
    motorState.setDataValidity(DataValidity::ok);
    motorState.writeIfDifferent(_motor->get()->getState());
    dummySignals.writeAll();
    message.write();

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
          notificationMessage = "Motor device is in recovery, not doing control step";
        }
        else {
          // FIXME Keep this try/catch only as long as we rely on the dummy for tests
          try {
            notificationMessage = std::get<2>(_funcMap.at(changedVarId))();
          }
          catch(ChimeraTK::logic_error& e) {
            notificationMessage = "Exception: " + std::string(e.what());
          }
        }

        dummySignals.dummyMotorStop.writeIfDifferent(control.stop || control.emergencyStop);
        dummySignals.dummyMotorTrigger++;
        dummySignals.dummyMotorTrigger.write();
        message.setAndWrite(notificationMessage);
      }
      catch(ChimeraTK::runtime_error& e) {
        // Comes from the state readout. Do nothing here. either it was a single glitch
        // or the readout module will see it as well and start the recovery procedure
      }
    }
  }

  /********************************************************************************************************************/

  std::string ControlInputHandler::enableCallback() {
    if(control.enable) {
      _motor->get()->setEnabled(true);
      motorState.writeIfDifferent(_motor->get()->getState());
    }
    return "";
  }

  /********************************************************************************************************************/

  std::string ControlInputHandler::disableCallback() {
    if(control.disable) {
      _motor->get()->setEnabled(false);
      motorState.writeIfDifferent(_motor->get()->getState());
    }

    return {};
  }

  /********************************************************************************************************************/

  std::string ControlInputHandler::startCallback() {
    if(control.start) {
      if(_motor->get()->isSystemIdle()) {
        _motor->get()->start();
        motorState.writeIfDifferent(_motor->get()->getState());
      }
      else {
        return "WARNING: Called startMotor while motor is not in IDLE state.";
      }
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
      return {};
    }

    if(control.calibrationCtrl.calibrateMotor) {
      if(_motor->get()->isSystemIdle()) {
        auto code = _motor->get()->calibrate();
        if(code != ExitStatus::SUCCESS) {
          return "Could not calibrate motor: " + ChimeraTK::MotorDriver::toString(code);
        }
      }
      else {
        return "WARNING: Called calibrateMotor while motor is not in IDLE state.";
      }
    }

    return {};
  }

  /********************************************************************************************************************/

  std::string ControlInputHandler::determineToleranceCallback() {
    if(!_motor->get()->hasHWReferenceSwitches()) {
      return "";
    }

    if(control.calibrationCtrl.determineTolerance) {
      if(_motor->get()->isSystemIdle()) {
        auto code = _motor->get()->determineTolerance();
        if(code != ExitStatus::SUCCESS) {
          return "Could not determine endswitch tolerance: " + ChimeraTK::MotorDriver::toString(code);
        }
      }
      else {
        return "WARNING: Called determineTolerance while motor is not in IDLE state.";
      }
    }
    return "";
  }
} // namespace ChimeraTK::MotorDriver
