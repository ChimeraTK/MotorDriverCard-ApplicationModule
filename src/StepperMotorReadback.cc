// SPDX-FileCopyrightText: Deutsches Elektronen-Synchrotron DESY, MSK, ChimeraTK Project <chimeratk-support@desy.de>
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "StepperMotorReadback.h"

#include <utility>

namespace ChimeraTK::MotorDriver {

  void ReadbackHandler::mainLoop() {
    readConstData();

    deviceError.status = static_cast<int32_t>(StatusOutput::Status::OK);
    deviceError.message = "";
    deviceError.setCurrentVersionNumber({});
    deviceError.writeAll();

    while(true) {
      if(_motor->isOpen()) {
        // First calculate the initial values
        tryReadingFromMotor();
      }

      // We now have all data. Write them, whether or not the read failed. this will set the validity properly.
      writeAll();

      // Wait for cyclic trigger
      trigger.read();

      if(not _motor->isOpen()) {
        tryMotorRenew();
      }
    }
  }

  /********************************************************************************************************************/

  ReadbackHandler::ReadbackHandler(std::shared_ptr<Motor> motor, ModuleGroup* owner, const std::string& name,
      const std::string& description, const std::string& triggerPath)
  : ApplicationModule::ApplicationModule(owner, name, description), _motor{std::move(motor)}, _motorIsDummy{
                                                                                                  motorIsDummy()} {
    if(_motor->get()->hasHWReferenceSwitches()) {
      positiveEndSwitch = ReferenceSwitch{this, "positiveEndSwitch", "Data of the positive end switch", {"MOTOR"}};
      negativeEndSwitch = ReferenceSwitch{this, "negativeEndSwitch", "Data of the negative end switch", {"MOTOR"}};
    }
    trigger = VoidInput{
        this, triggerPath, "Trigger to initiate reading from HW", std::unordered_set<std::string>{"MOT_TRIG"}};
  }

  /********************************************************************************************************************/

  void ReadbackHandler::readConstData() {
    if(!_motorIsDummy) {
      currentLimit.maxValue = _motor->get()->getSafeCurrentLimit();
      speedLimit.maxValue = _motor->get()->getMaxSpeedCapability();
    }
  }

  /********************************************************************************************************************/

  void ReadbackHandler::readback() {
    status.isEnabled = _motor->get()->getEnabled();

    auto calibMode = _motor->get()->getCalibrationMode();
    auto error = _motor->get()->getError();

    status.calibrationMode = static_cast<int>(calibMode);
    status.errorId = static_cast<int32_t>(error);

    status.isFullStepping = _motor->get()->isFullStepping();
    status.autostartEnabled = _motor->get()->getAutostart();

    status.encoderReadoutMode = _motor->get()->getEncoderReadoutMode();

    position.actualValueInSteps = _motor->get()->getCurrentPositionInSteps();
    position.encoderReadback = _motor->get()->getEncoderPosition();
    position.targetValueInSteps = _motor->get()->getTargetPositionInSteps();

    // Update values that have a static relation to HW readback
    position.actualValue = _motor->get()->recalculateStepsInUnits(position.actualValueInSteps);
    position.targetValue = _motor->get()->recalculateStepsInUnits(position.targetValueInSteps);

    status.isIdle = _motor->get()->isSystemIdle();
    status.state = _motor->get()->getState();
    swLimits.isEnabled = _motor->get()->getSoftwareLimitsEnabled();
    swLimits.maxPositionInSteps = _motor->get()->getMaxPositionLimitInSteps();
    swLimits.minPositionInSteps = _motor->get()->getMinPositionLimitInSteps();
    swLimits.maxPosition = _motor->get()->recalculateStepsInUnits(swLimits.maxPositionInSteps);
    swLimits.minPosition = _motor->get()->recalculateStepsInUnits(swLimits.minPositionInSteps);

    if(!_motorIsDummy) {
      currentLimit.userValue = _motor->get()->getUserCurrentLimit(); // Include when this method gets implemented
    }
    speedLimit.userValue = _motor->get()->getUserSpeedLimit();
  }

  /********************************************************************************************************************/

  /// Reading data specific for motor with end switches
  void ReadbackHandler::readEndSwitchData() {
    negativeEndSwitch.isActive = _motor->get()->isNegativeReferenceActive();
    positiveEndSwitch.isActive = _motor->get()->isPositiveReferenceActive();

    positiveEndSwitch.positionInSteps = _motor->get()->getPositiveEndReferenceInSteps();
    negativeEndSwitch.positionInSteps = _motor->get()->getNegativeEndReferenceInSteps();
    positiveEndSwitch.position = _motor->get()->recalculateStepsInUnits(positiveEndSwitch.positionInSteps);
    negativeEndSwitch.position = _motor->get()->recalculateStepsInUnits(negativeEndSwitch.positionInSteps);

    positiveEndSwitch.tolerance = _motor->get()->getTolerancePositiveEndSwitch();
    negativeEndSwitch.tolerance = _motor->get()->getToleranceNegativeEndSwitch();
  }

  /********************************************************************************************************************/

  bool ReadbackHandler::motorIsDummy() const {
    bool isDummy = false;
    try {
      // Throws if dummy is used
      _motor->get()->getSafeCurrentLimit();
    }
    catch(ChimeraTK::logic_error&) {
      isDummy = true;
    }
    return isDummy;
  }

  /********************************************************************************************************************/

  void ReadbackHandler::tryMotorRenew() {
    try {
      _motor->renew();
      std::cerr << "device recovered: " << _motor->toString() << std::endl;
      decrementDataFaultCounter();
      deviceError.status = static_cast<int32_t>(StatusOutput::Status::OK);
      deviceError.message = "";
      deviceError.setCurrentVersionNumber({});
      deviceError.writeAll();
      _spiErrorCounter = 0;
    }
    catch(ChimeraTK::runtime_error& e) {
      if(std::string(deviceError.message) != e.what()) {
        setStatusFromException(e);
      }
    }
  }

  /********************************************************************************************************************/

  void ReadbackHandler::tryReadingFromMotor() {
    constexpr unsigned int SPI_RETRY_COUNT{5};
    try {
      // FIXME This is only to evaluate the timer
      if(!_execTimer.isInitialized()) {
        _execTimer.initializeMeasurement();
      }
      else {
        _execTimer.measureIterativeMean();
      }
      auto ct = std::chrono::duration_cast<std::chrono::microseconds>(_execTimer.getMeasurementResult());
      actualCycleTime = static_cast<float>(ct.count()) / 1000.F;

      _receiveTimer.initializeMeasurement();

      readback();
      if(_motor->get()->hasHWReferenceSwitches()) {
        readEndSwitchData();
      }

      _receiveTimer.measureOnce();
      auto rt = std::chrono::duration_cast<std::chrono::microseconds>(_receiveTimer.getMeasurementResult());
      actualReceiveTime = static_cast<float>(rt.count()) / 1000.F;

      // We have apparently communicated successfully. Reset the error counter
      _spiErrorCounter = 0;
    }
    catch(ChimeraTK::runtime_error& e) {
      if(_spiErrorCounter > SPI_RETRY_COUNT) {
        std::cerr << "motor device failed: " << _motor->toString() << std::endl;
        incrementDataFaultCounter();
        _motor->close();
        setStatusFromException(e);
      }
      else {
        _spiErrorCounter++;
      }
    }
  }

  /********************************************************************************************************************/

  void ReadbackHandler::setStatusFromException(const std::exception& e) {
    deviceError.status = static_cast<int32_t>(StatusOutput::Status::FAULT);
    deviceError.message = e.what();
    deviceError.setCurrentVersionNumber({});
    deviceError.writeAll();
  }
} // namespace ChimeraTK::MotorDriver
