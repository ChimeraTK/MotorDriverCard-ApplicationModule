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
  : ApplicationModule(owner, name, description), trigger{this, triggerPath, "Trigger to initiate read-out from HW",
                                                     std::unordered_set<std::string>{"MOT_TRIG"}},
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
      TransferElementAbstractor& element, bool writeOnRecovery, std::function<void(void)> function) {
    _funcMap[element.getId()] = {writeOnRecovery, element.getName(), function};
  }

  void ControlInputHandler::createFunctionMap() {
    addMapping(control.enable, ::detail::SKIP_ON_RECOVERY, [this] { enableCallback(); });
    addMapping(control.disable, ::detail::SKIP_ON_RECOVERY, [this] { disableCallback(); });
    addMapping(control.start, ::detail::SKIP_ON_RECOVERY, [this] { startCallback(); });
    addMapping(control.stop, ::detail::SKIP_ON_RECOVERY, [this] {
      if(control.stop) {
        _motor->get()->stop();
        motorState.writeIfDifferent(_motor->get()->getState());
      }
    });
    addMapping(control.emergencyStop, ::detail::SKIP_ON_RECOVERY, [this] {
      if(control.emergencyStop) {
        _motor->get()->emergencyStop();
        motorState.writeIfDifferent(_motor->get()->getState());
      }
    });
    addMapping(control.resetError, ::detail::SKIP_ON_RECOVERY, [this] {
      if(control.resetError) {
        _motor->get()->resetError();
        motorState.writeIfDifferent(_motor->get()->getState());
      }
    });
    addMapping(control.enableAutostart, ::detail::WRITE_ON_RECOVERY, [this] {
      _motor->get()->setAutostart(control.enableAutostart);
      motorState.writeIfDifferent(_motor->get()->getState());
    });
    addMapping(control.enableFullStepping, ::detail::WRITE_ON_RECOVERY,
        [this] { _motor->get()->enableFullStepping(control.enableFullStepping); });

    addMapping(positionSetpoint.positionInSteps, ::detail::SKIP_ON_RECOVERY, [this] {
      auto code = _motor->get()->setTargetPositionInSteps(positionSetpoint.positionInSteps);
      motorState.writeIfDifferent(_motor->get()->getState());
      if(code != ExitStatus::SUCCESS) {
        notification.message = "Could not set target position: " + ChimeraTK::MotorDriver::toString(code);
      }
    });
    addMapping(positionSetpoint.position, ::detail::SKIP_ON_RECOVERY, [this] {
      auto code = _motor->get()->setTargetPosition(positionSetpoint.position);
      motorState.writeIfDifferent(_motor->get()->getState());
      if(code != ExitStatus::SUCCESS) {
        notification.message = "Could not set target position: " + ChimeraTK::MotorDriver::toString(code);
      }
    });
    addMapping(positionSetpoint.relativePositionInSteps, ::detail::SKIP_ON_RECOVERY, [this] {
      auto code = _motor->get()->moveRelativeInSteps(positionSetpoint.relativePositionInSteps);
      motorState.writeIfDifferent(_motor->get()->getState());
      if(code != ExitStatus::SUCCESS) {
        notification.message = "Could not set relative position: " + ChimeraTK::MotorDriver::toString(code);
      }
    });
    addMapping(positionSetpoint.relativePosition, ::detail::SKIP_ON_RECOVERY, [this] {
      auto code = _motor->get()->moveRelative(positionSetpoint.relativePosition);
      motorState.writeIfDifferent(_motor->get()->getState());
      if(code != ExitStatus::SUCCESS) {
        notification.message = "Could not set relative position: " + ChimeraTK::MotorDriver::toString(code);
      }
    });

    addMapping(referenceSettings.positionInSteps, ::detail::SKIP_ON_RECOVERY, [this] {
      auto code = _motor->get()->setActualPositionInSteps(referenceSettings.positionInSteps);
      motorState.writeIfDifferent(_motor->get()->getState());
      if(code != ExitStatus::SUCCESS) {
        notification.message = "Could not set reference position in steps: " + ChimeraTK::MotorDriver::toString(code);
      }
    });
    addMapping(referenceSettings.position, ::detail::SKIP_ON_RECOVERY, [this] {
      auto code = _motor->get()->setActualPosition(referenceSettings.position);
      motorState.writeIfDifferent(_motor->get()->getState());
      if(code != ExitStatus::SUCCESS) {
        notification.message = "Could not set reference position: " + ChimeraTK::MotorDriver::toString(code);
      }
    });
    addMapping(referenceSettings.encoderPosition, ::detail::SKIP_ON_RECOVERY, [this] {
      auto code = _motor->get()->setActualEncoderPosition(referenceSettings.encoderPosition);
      if(code != ExitStatus::SUCCESS) {
        notification.message = "Could not set reference encoder position: " + ChimeraTK::MotorDriver::toString(code);
      }
    });
    addMapping(referenceSettings.axisTranslationInSteps, ::detail::SKIP_ON_RECOVERY, [this] {
      auto code = _motor->get()->translateAxisInSteps(referenceSettings.axisTranslationInSteps);
      if(code != ExitStatus::SUCCESS) {
        notification.message = "Could not set axis translation: " + ChimeraTK::MotorDriver::toString(code);
      }
    });
    addMapping(referenceSettings.axisTranslation, ::detail::SKIP_ON_RECOVERY, [this] {
      auto code = _motor->get()->translateAxis(referenceSettings.axisTranslation);
      if(code != ExitStatus::SUCCESS) {
        notification.message = "Could not set axis translation: " + ChimeraTK::MotorDriver::toString(code);
      }
    });

    addMapping(swLimits.enable, ::detail::WRITE_ON_RECOVERY, [this] {
      auto code = _motor->get()->setSoftwareLimitsEnabled(swLimits.enable);
      if(code != ExitStatus::SUCCESS) {
        notification.message = "Could not enable software limits: " + ChimeraTK::MotorDriver::toString(code);
      }
    });
    addMapping(swLimits.maxPosition, ::detail::WRITE_ON_RECOVERY, [this] {
      auto code = _motor->get()->setMaxPositionLimit(swLimits.maxPosition);
      if(code != ExitStatus::SUCCESS) {
        notification.message = "Could not set max position limit: " + ChimeraTK::MotorDriver::toString(code);
      }
    });
    addMapping(swLimits.minPosition, ::detail::WRITE_ON_RECOVERY, [this] {
      auto code = _motor->get()->setMinPositionLimit(swLimits.minPosition);
      if(code != ExitStatus::SUCCESS) {
        notification.message = "Could not set min position limits: " + ChimeraTK::MotorDriver::toString(code);
      }
    });
    addMapping(swLimits.maxPositionInSteps, ::detail::SKIP_ON_RECOVERY, [this] {
      auto code = _motor->get()->setMaxPositionLimitInSteps(swLimits.maxPositionInSteps);
      if(code != ExitStatus::SUCCESS) {
        notification.message = "Could not set max position limits: " + ChimeraTK::MotorDriver::toString(code);
      }
    });
    addMapping(swLimits.minPositionInSteps, ::detail::WRITE_ON_RECOVERY, [this] {
      auto code = _motor->get()->setMinPositionLimitInSteps(swLimits.minPositionInSteps);
      if(code != ExitStatus::SUCCESS) {
        notification.message = "Could not set min position limits: " + ChimeraTK::MotorDriver::toString(code);
      }
    });

    addMapping(userLimits.current, ::detail::WRITE_ON_RECOVERY, [this] {
      try {
        auto code = _motor->get()->setUserCurrentLimit(userLimits.current);
        motorState.writeIfDifferent(_motor->get()->getState());
        if(code != ExitStatus::SUCCESS) {
          notification.message = "Could not set user current limits: " + ChimeraTK::MotorDriver::toString(code);
        }
      }
      catch(ChimeraTK::logic_error&) {
        // Do nothing, this is a dummy then
      }
    });
    addMapping(userLimits.speed, ::detail::WRITE_ON_RECOVERY, [this] {
      try {
        auto code = _motor->get()->setUserSpeedLimit(userLimits.speed);
        if(code != ExitStatus::SUCCESS) {
          notification.message = "Could not set user speed limits: " + ChimeraTK::MotorDriver::toString(code);
        }
      }
      catch(ChimeraTK::logic_error&) {
        // Do nothing, this is a dummy then
      }
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
        call();
      }
    }
  }

  /********************************************************************************************************************/

  void ControlInputHandler::mainLoop() {
    auto inputGroup = this->readAnyGroup();

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
    notification.writeAll();

    while(true) {
      notification.message = "";
      notification.hasMessage = false;

      auto changedVarId = inputGroup.readAny();
      if(changedVarId == deviceBecameFunctional.getId()) {
        writeRecoveryValues();
        continue;
      }

      // Some state changes are only triggered by the readout of the state, so we regularly
      // read it out and publish any changes
      if(changedVarId == trigger.getId() && _motor->isOpen()) {
        motorState.writeIfDifferent(_motor->get()->getState());
        continue;
      }

      if(not _motor->isOpen()) {
        notification.hasMessage = true;
        notification.message = "Motor device is in recovery, not doing control step";
      }
      else {
        // FIXME Keep this only as long as we rely on the dummy for tests
        try {
          std::get<2>(_funcMap.at(changedVarId))();
        }
        catch(ChimeraTK::logic_error& e) {
          notification.message = "Exception: " + std::string(e.what());
        }
      }

      // We could either use string::compare or cast the notification.message to string.
      // NOLINTNEXTLINE(readability-string-compare)
      if(std::string("").compare(notification.message)) {
        notification.hasMessage = true;
      }

      dummySignals.dummyMotorStop.writeIfDifferent(control.stop || control.emergencyStop);
      dummySignals.dummyMotorTrigger++;
      dummySignals.dummyMotorTrigger.write();
      notification.writeAll();
    }
  }

  /********************************************************************************************************************/

  void ControlInputHandler::enableCallback() {
    if(control.enable) {
      _motor->get()->setEnabled(true);
      motorState.writeIfDifferent(_motor->get()->getState());
    }
  }

  /********************************************************************************************************************/

  void ControlInputHandler::disableCallback() {
    if(control.disable) {
      _motor->get()->setEnabled(false);
      motorState.writeIfDifferent(_motor->get()->getState());
    }
  }

  /********************************************************************************************************************/

  void ControlInputHandler::startCallback() {
    if(control.start) {
      if(_motor->get()->isSystemIdle()) {
        _motor->get()->start();
        motorState.writeIfDifferent(_motor->get()->getState());
      }
      else {
        notification.message = "WARNING: Called startMotor while motor is not in IDLE state.";
      }
    }
  }

  /********************************************************************************************************************/

  void ControlInputHandler::appendCalibrationToMap() {
    addMapping(control.calibrationCtrl.calibrateMotor, ::detail::SKIP_ON_RECOVERY, [this] { calibrateCallback(); });
    addMapping(control.calibrationCtrl.determineTolerance, ::detail::SKIP_ON_RECOVERY,
        [this] { determineToleranceCallback(); });
  }

  /********************************************************************************************************************/

  void ControlInputHandler::calibrateCallback() {
    if(!_motor->get()->hasHWReferenceSwitches()) {
      return;
    }

    if(control.calibrationCtrl.calibrateMotor) {
      if(_motor->get()->isSystemIdle()) {
        auto code = _motor->get()->calibrate();
        if(code != ExitStatus::SUCCESS) {
          notification.message = "Could not calibrate motor: " + ChimeraTK::MotorDriver::toString(code);
        }
      }
      else {
        notification.message = "WARNING: Called calibrateMotor while motor is not in IDLE state.";
      }
    }
  }

  /********************************************************************************************************************/

  void ControlInputHandler::determineToleranceCallback() {
    if(!_motor->get()->hasHWReferenceSwitches()) {
      return;
    }

    if(control.calibrationCtrl.determineTolerance) {
      if(_motor->get()->isSystemIdle()) {
        auto code = _motor->get()->determineTolerance();
        if(code != ExitStatus::SUCCESS) {
          notification.message = "Could not determine endswitch tolerance: " + ChimeraTK::MotorDriver::toString(code);
        }
      }
      else {
        notification.message = "WARNING: Called determineTolerance while motor is not in IDLE state.";
      }
    }
  }
} // namespace ChimeraTK::MotorDriver
