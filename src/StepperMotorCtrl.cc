/*
 * StepperMotorCtrl.cc
 *
 *  Created on: Sep 13, 2018
 *      Author: ckampm
 */

#include "StepperMotorCtrl.h"
#include "mtca4u/MotorDriverCard/MotorDriverException.h"

namespace ChimeraTK { namespace MotorDriver {

  ControlInputHandler::ControlInputHandler(
      EntityOwner* owner, const std::string& name, const std::string& description, std::shared_ptr<Motor> motor)
  : ApplicationModule(owner, name, description), funcMap(), inputGroup(), _motor(motor) {
    // If motor has HW reference switches,
    // calibration is supported
    if(_motor->get()->hasHWReferenceSwitches()) {
      control.calibrationCtrl = CalibrationCommands{
          &control, "calibrationControl", "Calibration commands", HierarchyModifier::hideThis, {"MOTOR"}};
    }
  }

  void ControlInputHandler::createFunctionMap(std::shared_ptr<Motor> motor) {
    funcMap[control.enable.getId()] = [this] { enableCallback(); };
    funcMap[control.disable.getId()] = [this] { disableCallback(); };
    funcMap[control.start.getId()] = [this] { startCallback(); };
    funcMap[control.stop.getId()] = [this, motor] {
      if(control.stop) {
        motor->get()->stop();
      }
    };
    funcMap[control.emergencyStop.getId()] = [this, motor] {
      if(control.emergencyStop) {
        motor->get()->emergencyStop();
      }
    };
    funcMap[control.resetError.getId()] = [this, motor] {
      if(control.resetError) {
        motor->get()->resetError();
      }
    };
    funcMap[control.enableAutostart.getId()] = [this, motor] { motor->get()->setAutostart(control.enableAutostart); };
    funcMap[control.enableFullStepping.getId()] = [this, motor] {
      motor->get()->enableFullStepping(control.enableFullStepping);
    };

    funcMap[positionSetpoint.positionInSteps.getId()] = [this, motor] {
      motor->get()->setTargetPositionInSteps(positionSetpoint.positionInSteps);
    };
    funcMap[positionSetpoint.position.getId()] = [this, motor] {
      motor->get()->setTargetPosition(positionSetpoint.position);
    };
    funcMap[positionSetpoint.relativePositionInSteps.getId()] = [this, motor] {
      motor->get()->moveRelativeInSteps(positionSetpoint.relativePositionInSteps);
    };
    funcMap[positionSetpoint.relativePosition.getId()] = [this, motor] {
      motor->get()->moveRelative(positionSetpoint.relativePosition);
    };

    funcMap[referenceSettings.positionInSteps.getId()] = [this, motor] {
      motor->get()->setActualPositionInSteps(referenceSettings.positionInSteps);
    };
    funcMap[referenceSettings.position.getId()] = [this, motor] {
      motor->get()->setActualPosition(referenceSettings.position);
    };
    funcMap[referenceSettings.encoderPosition.getId()] = [this, motor] {
      motor->get()->setActualEncoderPosition(referenceSettings.encoderPosition);
    };
    funcMap[referenceSettings.axisTranslationInSteps.getId()] = [this, motor] {
      motor->get()->translateAxisInSteps(referenceSettings.axisTranslationInSteps);
    };
    funcMap[referenceSettings.axisTranslation.getId()] = [this, motor] {
      motor->get()->translateAxis(referenceSettings.axisTranslation);
    };

    funcMap[swLimits.enable.getId()] = [this, motor] { motor->get()->setSoftwareLimitsEnabled(swLimits.enable); };
    funcMap[swLimits.maxPosition.getId()] = [this, motor] { motor->get()->setMaxPositionLimit(swLimits.maxPosition); };
    funcMap[swLimits.minPosition.getId()] = [this, motor] { motor->get()->setMinPositionLimit(swLimits.minPosition); };
    funcMap[swLimits.maxPositionInSteps.getId()] = [this, motor] {
      motor->get()->setMaxPositionLimitInSteps(swLimits.maxPositionInSteps);
    };
    funcMap[swLimits.minPositionInSteps.getId()] = [this, motor] {
      motor->get()->setMinPositionLimitInSteps(swLimits.minPositionInSteps);
    };

    funcMap[userLimits.current.getId()] = [this, motor] { motor->get()->setUserCurrentLimit(userLimits.current); };
    funcMap[userLimits.speed.getId()] = [this, motor] { motor->get()->setUserSpeedLimit(userLimits.speed); };
  }

  void ControlInputHandler::prepare() {
    createFunctionMap(_motor);

    if(_motor->get()->hasHWReferenceSwitches()) {
      appendCalibrationToMap();
    }

    writeAll();
  }

  void ControlInputHandler::mainLoop() {
    inputGroup = this->readAnyGroup();

    // Write once to propagate inital values
    writeAll();

    while(true) {
      notification.message = "";
      notification.hasMessage = 0;

      auto changedVarId = inputGroup.readAny();

      if(not _motor->isOpen()) {
        std::cerr << "Motor device is in recovery, not doing control step" << std::endl;
        continue;
      }

      //FIXME Keep this only as long as we rely on the dummy for tests
      try {
        funcMap.at(changedVarId)();
      }
      catch(mtca4u::MotorDriverException& e) {
        notification.message = "Exception: " + std::string(e.what());
      }

      if(std::string("").compare(notification.message)) {
        notification.hasMessage = 1;
      }

      dummySignals.dummyMotorStop = control.stop || control.emergencyStop;
      dummySignals.dummyMotorTrigger++;

      writeAll();
    }
  }

  void ControlInputHandler::enableCallback() {
    if(control.enable) {
      _motor->get()->setEnabled(true);
    }
  }

  void ControlInputHandler::disableCallback() {
    if(control.disable) {
      _motor->get()->setEnabled(false);
    }
  }

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

  void ControlInputHandler::appendCalibrationToMap() {
    funcMap[control.calibrationCtrl.calibrateMotor.getId()] = [this] { calibrateCallback(); };
    funcMap[control.calibrationCtrl.determineTolerance.getId()] = [this] { determineToleranceCallback(); };
  }

  void ControlInputHandler::calibrateCallback() {
    if(control.calibrationCtrl.calibrateMotor) {
      if(_motor->get()->isSystemIdle()) {
        _motor->get()->calibrate();
      }
      else {
        notification.message = "WARNING: Called calibrateMotor while motor is not in IDLE state.";
      }
    }
  }

  void ControlInputHandler::determineToleranceCallback() {
    if(control.calibrationCtrl.determineTolerance) {
      if(_motor->get()->isSystemIdle()) {
        _motor->get()->determineTolerance();
      }
      else {
        notification.message = "WARNING: Called determineTolerance while motor is not in IDLE state.";
      }
    }
  }
}} // namespace ChimeraTK::MotorDriver
