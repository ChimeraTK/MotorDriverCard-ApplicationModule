// SPDX-FileCopyrightText: Deutsches Elektronen-Synchrotron DESY, MSK, ChimeraTK Project <chimeratk-support@desy.de>
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "StepperMotorCtrl.h"

#include <utility>

namespace ChimeraTK::MotorDriver {

  ControlInputHandler::ControlInputHandler(
      ModuleGroup* owner, const std::string& name, const std::string& description, std::shared_ptr<Motor> motor)
  : ApplicationModule(owner, name, description), _motor(std::move(motor)) {
    // If motor has HW reference switches,
    // calibration is supported
    if(_motor->getMotorParameters().motorType == StepperMotorType::LINEAR) {
      control.calibrationCtrl = CalibrationCommands{&control, ".", "Calibration commands", {"MOTOR"}};
    }
  }

  /********************************************************************************************************************/

  void ControlInputHandler::createFunctionMap() {
    _funcMap[control.enable.getId()] = [this] { enableCallback(); };
    _funcMap[control.disable.getId()] = [this] { disableCallback(); };
    _funcMap[control.start.getId()] = [this] { startCallback(); };
    _funcMap[control.stop.getId()] = [this] {
      if(control.stop) {
        _motor->get()->stop();
      }
    };
    _funcMap[control.emergencyStop.getId()] = [this] {
      if(control.emergencyStop) {
        _motor->get()->emergencyStop();
      }
    };
    _funcMap[control.resetError.getId()] = [this] {
      if(control.resetError) {
        _motor->get()->resetError();
      }
    };
    _funcMap[control.enableAutostart.getId()] = [this] { _motor->get()->setAutostart(control.enableAutostart); };
    _funcMap[control.enableFullStepping.getId()] = [this] {
      _motor->get()->enableFullStepping(control.enableFullStepping);
    };

    _funcMap[positionSetpoint.positionInSteps.getId()] = [this] {
      _motor->get()->setTargetPositionInSteps(positionSetpoint.positionInSteps);
    };
    _funcMap[positionSetpoint.position.getId()] = [this] {
      _motor->get()->setTargetPosition(positionSetpoint.position);
    };
    _funcMap[positionSetpoint.relativePositionInSteps.getId()] = [this] {
      _motor->get()->moveRelativeInSteps(positionSetpoint.relativePositionInSteps);
    };
    _funcMap[positionSetpoint.relativePosition.getId()] = [this] {
      _motor->get()->moveRelative(positionSetpoint.relativePosition);
    };

    _funcMap[referenceSettings.positionInSteps.getId()] = [this] {
      _motor->get()->setActualPositionInSteps(referenceSettings.positionInSteps);
    };
    _funcMap[referenceSettings.position.getId()] = [this] {
      _motor->get()->setActualPosition(referenceSettings.position);
    };
    _funcMap[referenceSettings.encoderPosition.getId()] = [this] {
      _motor->get()->setActualEncoderPosition(referenceSettings.encoderPosition);
    };
    _funcMap[referenceSettings.axisTranslationInSteps.getId()] = [this] {
      _motor->get()->translateAxisInSteps(referenceSettings.axisTranslationInSteps);
    };
    _funcMap[referenceSettings.axisTranslation.getId()] = [this] {
      _motor->get()->translateAxis(referenceSettings.axisTranslation);
    };

    _funcMap[swLimits.enable.getId()] = [this] { _motor->get()->setSoftwareLimitsEnabled(swLimits.enable); };
    _funcMap[swLimits.maxPosition.getId()] = [this] { _motor->get()->setMaxPositionLimit(swLimits.maxPosition); };
    _funcMap[swLimits.minPosition.getId()] = [this] { _motor->get()->setMinPositionLimit(swLimits.minPosition); };
    _funcMap[swLimits.maxPositionInSteps.getId()] = [this] {
      _motor->get()->setMaxPositionLimitInSteps(swLimits.maxPositionInSteps);
    };
    _funcMap[swLimits.minPositionInSteps.getId()] = [this] {
      _motor->get()->setMinPositionLimitInSteps(swLimits.minPositionInSteps);
    };

    _funcMap[userLimits.current.getId()] = [this] { _motor->get()->setUserCurrentLimit(userLimits.current); };
    _funcMap[userLimits.speed.getId()] = [this] { _motor->get()->setUserSpeedLimit(userLimits.speed); };
  }

  void ControlInputHandler::prepare() {
    createFunctionMap();

    if(_motor->getMotorParameters().motorType == StepperMotorType::LINEAR) {
      appendCalibrationToMap();
    }

    writeAll();
  }

  /********************************************************************************************************************/

  void ControlInputHandler::mainLoop() {
    auto inputGroup = this->readAnyGroup();

    // Write once to propagate inital values
    writeAll();

    while(true) {
      notification.message = "";
      notification.hasMessage = false;

      auto changedVarId = inputGroup.readAny();

      if(not _motor->isOpen()) {
        notification.hasMessage = true;
        notification.message = "Motor device is in recovery, not doing control step";
      } else {

        // FIXME Keep this only as long as we rely on the dummy for tests
        try {
          _funcMap.at(changedVarId)();
        }
        catch(ChimeraTK::logic_error& e) {
          notification.message = "Exception: " + std::string(e.what());
        }
      }

      // We could either use string::compare or cast the notification.message to string.
      // NOLINTNEXTLINE(readability-string-compare)
      if(std::string("").compare(notification.message)) {
        notification.hasMessage = 1;
      }

      dummySignals.dummyMotorStop = control.stop || control.emergencyStop;
      dummySignals.dummyMotorTrigger++;

      writeAll();
    }
  }

  /********************************************************************************************************************/

  void ControlInputHandler::enableCallback() {
    if(control.enable) {
      _motor->get()->setEnabled(true);
    }
  }

  /********************************************************************************************************************/

  void ControlInputHandler::disableCallback() {
    if(control.disable) {
      _motor->get()->setEnabled(false);
    }
  }

  /********************************************************************************************************************/

  void ControlInputHandler::startCallback() {
    if(control.start) {
      if(_motor->get()->isSystemIdle()) {
        _motor->get()->start();
      }
      else {
        notification.message = "WARNING: Called startMotor while motor is not in IDLE state.";
      }
    }
  }

  /********************************************************************************************************************/

  void ControlInputHandler::appendCalibrationToMap() {
    _funcMap[control.calibrationCtrl.calibrateMotor.getId()] = [this] { calibrateCallback(); };
    _funcMap[control.calibrationCtrl.determineTolerance.getId()] = [this] { determineToleranceCallback(); };
  }

  /********************************************************************************************************************/

  void ControlInputHandler::calibrateCallback() {
    if(!_motor->get()->hasHWReferenceSwitches()) {
      return;
    }

    if(control.calibrationCtrl.calibrateMotor) {
      if(_motor->get()->isSystemIdle()) {
        _motor->get()->calibrate();
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
        _motor->get()->determineTolerance();
      }
      else {
        notification.message = "WARNING: Called determineTolerance while motor is not in IDLE state.";
      }
    }
  }
} // namespace ChimeraTK::MotorDriver
