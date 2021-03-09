/*
 * StepperMotorReadback.cc
 *
 *  Created on: Sep 17, 2018
 *      Author: ckampm
 */

#include "StepperMotorReadback.h"
#include "ChimeraTK/MotorDriverCard/StepperMotorException.h"

namespace ChimeraTK { namespace MotorDriver {

  void ReadbackHandler::mainLoop() {
    readConstData();

    while(true) {
      //First calculate the initial values

      // FIXME This is only to evaluate the timer
      if(!execTimer.isInitialized()) {
        execTimer.initializeMeasurement();
      }
      else {
        execTimer.measureIterativeMean();
      }
      auto ct = std::chrono::duration_cast<std::chrono::microseconds>(execTimer.getMeasurementResult());
      actualCycleTime = static_cast<float>(ct.count()) / 1000.f;

      receiveTimer.initializeMeasurement();

      readback();
      if(_motor->hasHWReferenceSwitches()) {
        readEndSwitchData();
      }

      receiveTimer.measureOnce();
      auto rt = std::chrono::duration_cast<std::chrono::microseconds>(receiveTimer.getMeasurementResult());
      actualReceiveTime = static_cast<float>(rt.count()) / 1000.f;

      //We now have all data. Write them
      writeAll();

      //Wait for cyclic trigger
      trigger.read();
    }
  }

  ReadbackHandler::ReadbackHandler(
      std::shared_ptr<StepperMotor> motor, EntityOwner* owner, const std::string& name, const std::string& description)
  : ApplicationModule::ApplicationModule(owner, name, description), positiveEndSwitch{}, negativeEndSwitch{},
    readbackFunction{}, _motor{motor}, execTimer{}, receiveTimer{}, _motorIsDummy{motorIsDummy()} {
    if(_motor->hasHWReferenceSwitches()) {
      positiveEndSwitch =
          ReferenceSwitch{this, "positiveEndSwitch", "Data of the positive end switch", false, {"MOTOR"}};
      negativeEndSwitch =
          ReferenceSwitch{this, "negativeEndSwitch", "Data of the negative end switch", false, {"MOTOR"}};
    }
  }

  void ReadbackHandler::readConstData() {
    if(!_motorIsDummy) {
      currentLimit.maxValue = _motor->getSafeCurrentLimit();
      speedLimit.maxValue = _motor->getMaxSpeedCapability();
    }
  }

  void ReadbackHandler::readback() {
    status.isEnabled = _motor->getEnabled();

    auto calibMode = _motor->getCalibrationMode();
    auto error = _motor->getError();

    status.calibrationMode = static_cast<int>(calibMode);
    status.errorId = static_cast<int32_t>(error);

    status.isFullStepping = _motor->isFullStepping();
    status.autostartEnabled = _motor->getAutostart();

    position.actualValueInSteps = _motor->getCurrentPositionInSteps();
    position.encoderReadback = _motor->getEncoderPosition();
    position.targetValueInSteps = _motor->getTargetPositionInSteps();

    // Update values that have a static relation to HW readback
    position.actualValue = _motor->recalculateStepsInUnits(position.actualValueInSteps);
    position.targetValue = _motor->recalculateStepsInUnits(position.targetValueInSteps);

    status.isIdle = _motor->isSystemIdle();
    status.state = _motor->getState();
    swLimits.isEnabled = _motor->getSoftwareLimitsEnabled();
    swLimits.maxPositionInSteps = _motor->getMaxPositionLimitInSteps();
    swLimits.minPositionInSteps = _motor->getMinPositionLimitInSteps();
    swLimits.maxPosition = _motor->recalculateStepsInUnits(swLimits.maxPositionInSteps);
    swLimits.minPosition = _motor->recalculateStepsInUnits(swLimits.minPositionInSteps);

    if(!_motorIsDummy) {
      currentLimit.userValue = _motor->getUserCurrentLimit(); // Include when this method gets implemented
    }
    speedLimit.userValue = _motor->getUserSpeedLimit();
  }

  /// Reading data specific for motor with end switches
  void ReadbackHandler::readEndSwitchData() {
    negativeEndSwitch.isActive = _motor->isNegativeReferenceActive();
    positiveEndSwitch.isActive = _motor->isPositiveReferenceActive();

    positiveEndSwitch.positionInSteps = _motor->getPositiveEndReferenceInSteps();
    negativeEndSwitch.positionInSteps = _motor->getNegativeEndReferenceInSteps();
    positiveEndSwitch.position = _motor->recalculateStepsInUnits(positiveEndSwitch.positionInSteps);
    negativeEndSwitch.position = _motor->recalculateStepsInUnits(negativeEndSwitch.positionInSteps);

    positiveEndSwitch.tolerance = _motor->getTolerancePositiveEndSwitch();
    negativeEndSwitch.tolerance = _motor->getToleranceNegativeEndSwitch();
  }

  bool ReadbackHandler::motorIsDummy() {
    bool isDummy = false;
    try {
      // Throws if dummy is used
      _motor->getSafeCurrentLimit();
    }
    catch(StepperMotorException& e) {
      if(e.getID() == StepperMotorException::FEATURE_NOT_AVAILABLE) isDummy = true;
    }
    return isDummy;
  }

}} // namespace ChimeraTK::MotorDriver
