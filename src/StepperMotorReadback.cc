/*
 * StepperMotorReadback.cc
 *
 *  Created on: Sep 17, 2018
 *      Author: ckampm
 */

#include "StepperMotorReadback.h"
#include "ChimeraTK/MotorDriverCard/StepperMotorException.h"
#include <mtca4u/MotorDriverCard/MotorDriverException.h>

namespace ChimeraTK { namespace MotorDriver {

  void ReadbackHandler::mainLoop() {
    readConstData();
    unsigned int spiErrorCounter = 0;
    const unsigned int SPI_RETRY_COUNT = 5;

    deviceError.status = static_cast<int32_t>(StatusOutput::Status::OK);
    deviceError.message = "";
    deviceError.setCurrentVersionNumber({});
    deviceError.writeAll();

    while(true) {
      if(_motor->isOpen()) {
        //First calculate the initial values
        try {
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
          if(_motor->get()->hasHWReferenceSwitches()) {
            readEndSwitchData();
          }

          receiveTimer.measureOnce();
          auto rt = std::chrono::duration_cast<std::chrono::microseconds>(receiveTimer.getMeasurementResult());
          actualReceiveTime = static_cast<float>(rt.count()) / 1000.f;

          // We have aparently communicated successfully. Reset the error counter
          spiErrorCounter = 0;
        }
        catch(mtca4u::MotorDriverException& e) {
          if((e.getID() == mtca4u::MotorDriverException::SPI_ERROR ||
                 e.getID() == mtca4u::MotorDriverException::SPI_TIMEOUT)) {
            if(spiErrorCounter > SPI_RETRY_COUNT) {
              std::cerr << "SPI failed..." << std::endl;
              incrementDataFaultCounter();
              _motor->close();
              deviceError.status = static_cast<int32_t>(StatusOutput::Status::FAULT);
              deviceError.message = e.what();
              deviceError.setCurrentVersionNumber({});
              deviceError.writeAll();
            }
            else {
              spiErrorCounter++;
            }
          }
          else {
            throw std::current_exception();
          }
        }
      }

      //We now have all data. Write them, whether or not the read failed. this will set the validity properly.
      writeAll();

      //Wait for cyclic trigger
      trigger.read();

      if(not _motor->isOpen()) {
        try {
          _motor->renew();
          std::cerr << "device recovered: " << _motor->toString() << std::endl;
          decrementDataFaultCounter();
          deviceError.status = static_cast<int32_t>(StatusOutput::Status::OK);
          deviceError.message = "";
          deviceError.setCurrentVersionNumber({});
          deviceError.writeAll();
          spiErrorCounter = 0;
        }
        catch(mtca4u::MotorDriverException& e) {
          if(std::string(deviceError.message) != e.what()) {
            deviceError.message = e.what();
            deviceError.setCurrentVersionNumber({});
            deviceError.message.write();
          }
        }
      }
    }
  }

  ReadbackHandler::ReadbackHandler(
      std::shared_ptr<Motor> motor, EntityOwner* owner, const std::string& name, const std::string& description)
  : ApplicationModule::ApplicationModule(owner, name, description), positiveEndSwitch{}, negativeEndSwitch{},
    readbackFunction{}, _motor{motor}, execTimer{}, receiveTimer{}, _motorIsDummy{motorIsDummy()} {
    if(_motor->get()->hasHWReferenceSwitches()) {
      positiveEndSwitch = ReferenceSwitch{
          this, "positiveEndSwitch", "Data of the positive end switch", HierarchyModifier::none, {"MOTOR"}};
      negativeEndSwitch = ReferenceSwitch{
          this, "negativeEndSwitch", "Data of the negative end switch", HierarchyModifier::none, {"MOTOR"}};
    }
  }

  void ReadbackHandler::readConstData() {
    if(!_motorIsDummy) {
      currentLimit.maxValue = _motor->get()->getSafeCurrentLimit();
      speedLimit.maxValue = _motor->get()->getMaxSpeedCapability();
    }
  }

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

  bool ReadbackHandler::motorIsDummy() {
    bool isDummy = false;
    try {
      // Throws if dummy is used
      _motor->get()->getSafeCurrentLimit();
    }
    catch(StepperMotorException& e) {
      if(e.getID() == StepperMotorException::FEATURE_NOT_AVAILABLE) isDummy = true;
    }
    return isDummy;
  }

}} // namespace ChimeraTK::MotorDriver
