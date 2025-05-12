// SPDX-FileCopyrightText: Deutsches Elektronen-Synchrotron DESY, MSK, ChimeraTK Project <chimeratk-support@desy.de>
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "StepperMotorReadback.h"

#include <ChimeraTK/ApplicationCore/DeviceManager.h>

#include <cstdint>
#include <utility>

namespace ChimeraTK::MotorDriver {

  /********************************************************************************************************************/

  void ReadbackHandler::prepare() {
    // The module starts with the fake device state in FAIL mode since we
    // did not open the motor yet and know nothing about it
    moduleError.status = StatusOutput::Status::FAULT;
    moduleError.message = "Initial bringup of device";
    moduleError.setCurrentVersionNumber({});
    incrementDataFaultCounter();
    moduleError.status.setDataValidity(DataValidity::ok);
    moduleError.message.setDataValidity(DataValidity::ok);
    writeAll();
    decrementDataFaultCounter();
  }

  /********************************************************************************************************************/

  void ReadbackHandler::mainLoop() {
    auto group = ReadAnyGroup({trigger, deviceBecameFunctional});

    // Report an exception once on startup. If the device is already working, we get an initial toggle of
    // deviceBecameFunctional that way, otherwise we would sit in the wait for deviceBecameFunctional until the first
    // error happens.
    _deviceModule->reportException("Bootstrap module startup");

    while(true) {
      switch(static_cast<StatusOutput::Status>(moduleError.status)) {
        case StatusOutput::Status::FAULT:
          group.readUntil(deviceBecameFunctional.getId());

          tryMotorRenew();
          tryReadingFromMotor();
          break;
        case StatusOutput::Status::OK: {
          group.readUntil(trigger.getId());
          tryReadingFromMotor();
          break;
        }
        default:
          assert(false);
          break;
      }

      // If we are (still) in FAULT mode after this round of either readout or trying to open the motor
      // flag everything as invalid and shove it out the door
      if(moduleError.status == StatusOutput::Status::FAULT) {
        incrementDataFaultCounter();
      }

      // Flag the status and message outputs of the module errors as valid
      moduleError.status.setDataValidity(DataValidity::ok);
      moduleError.message.setDataValidity(DataValidity::ok);
      writeAll();

      if(moduleError.status == StatusOutput::Status::FAULT) {
        decrementDataFaultCounter();
      }
    }
  }

  /********************************************************************************************************************/

  ReadbackHandler::ReadbackHandler(std::shared_ptr<Motor> motor, ModuleGroup* owner, const std::string& name,
      const std::string& description, const std::string& triggerPath, DeviceModule* deviceModule)
  : ApplicationModule::ApplicationModule(owner, name, description),
    trigger{this, triggerPath, "Trigger to initiate read-out from HW", std::unordered_set<std::string>{"MOT_TRIG"}},
    _motor{std::move(motor)}, _motorIsDummy{false}, _deviceModule{deviceModule} {
    deviceBecameFunctional =
        VoidInput(this, deviceModule->getDeviceManager().deviceBecameFunctional.getModel().getFullyQualifiedPath(), "");
    deviceStatus = ScalarPollInput<int32_t>(this, deviceModule->getModel().getFullyQualifiedPath() + "/status", "", "");
  }

  /********************************************************************************************************************/

  void ReadbackHandler::readConstData() {
    if(_motor && !_motorIsDummy) {
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
    if(_motor->get()->hasHWReferenceSwitches()) {
      negativeEndSwitch.available = true;
      positiveEndSwitch.available = true;

      negativeEndSwitch.isActive = _motor->get()->isNegativeReferenceActive();
      positiveEndSwitch.isActive = _motor->get()->isPositiveReferenceActive();

      positiveEndSwitch.positionInSteps = _motor->get()->getPositiveEndReferenceInSteps();
      negativeEndSwitch.positionInSteps = _motor->get()->getNegativeEndReferenceInSteps();
      positiveEndSwitch.position = _motor->get()->recalculateStepsInUnits(positiveEndSwitch.positionInSteps);
      negativeEndSwitch.position = _motor->get()->recalculateStepsInUnits(negativeEndSwitch.positionInSteps);

      positiveEndSwitch.tolerance = _motor->get()->getTolerancePositiveEndSwitch();
      negativeEndSwitch.tolerance = _motor->get()->getToleranceNegativeEndSwitch();
    }
    else {
      positiveEndSwitch.available = false;
      negativeEndSwitch.available = false;

      positiveEndSwitch.isActive = 0;
      negativeEndSwitch.isActive = 0;
      positiveEndSwitch.isActive.setDataValidity(DataValidity::faulty);
      negativeEndSwitch.isActive.setDataValidity(DataValidity::faulty);

      positiveEndSwitch.positionInSteps.setDataValidity(DataValidity::faulty);
      negativeEndSwitch.positionInSteps.setDataValidity(DataValidity::faulty);

      positiveEndSwitch.position.setDataValidity(DataValidity::faulty);
      negativeEndSwitch.position.setDataValidity(DataValidity::faulty);

      positiveEndSwitch.tolerance = 0.F;
      negativeEndSwitch.tolerance = 0.F;
      positiveEndSwitch.tolerance.setDataValidity(DataValidity::faulty);
      negativeEndSwitch.tolerance.setDataValidity(DataValidity::faulty);
    }
  }

  /********************************************************************************************************************/

  bool ReadbackHandler::motorIsDummy() {
    if(!_motor) return _motorIsDummy;

    try {
      // Throws if dummy is used
      std::ignore = _motor->get()->getSafeCurrentLimit();
    }
    catch(ChimeraTK::logic_error&) {
      _motorIsDummy = true;
    }

    return _motorIsDummy;
  }

  /********************************************************************************************************************/

  void ReadbackHandler::tryMotorRenew() {
    try {
      std::ignore = motorIsDummy();
      readConstData();
      moduleError.status = StatusOutput::Status::OK;
      moduleError.message = "";
      moduleError.setCurrentVersionNumber({});
      _spiErrorCounter = 0;
    }
    catch(ChimeraTK::runtime_error& e) {
      if(std::string(moduleError.message) != e.what()) {
        setStatusFromException(e);
      }
      _deviceModule->reportException(e.what());
    }
  }

  /********************************************************************************************************************/

  void ReadbackHandler::tryReadingFromMotor() {
    constexpr unsigned int SPI_RETRY_COUNT{5};

    // Don't event try reading if there is no motor
    if(!_motor->isOpen()) {
      return;
    }

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
      readEndSwitchData();

      _receiveTimer.measureOnce();
      auto rt = std::chrono::duration_cast<std::chrono::microseconds>(_receiveTimer.getMeasurementResult());
      actualReceiveTime = static_cast<float>(rt.count()) / 1000.F;

      // We have apparently communicated successfully. Reset the error counter
      _spiErrorCounter = 0;
    }
    catch(ChimeraTK::runtime_error& e) {
      if(_spiErrorCounter > SPI_RETRY_COUNT) {
        _motor->close();
        setStatusFromException(e);
        _deviceModule->reportException(e.what());

        // Reset counter so that there is a longer period in trying to read after re-open
        _spiErrorCounter = 0;
      }
      else {
        _spiErrorCounter++;
      }
    }
  }

  /********************************************************************************************************************/

  void ReadbackHandler::setStatusFromException(const std::exception& e) {
    moduleError.status = StatusOutput::Status::FAULT;
    moduleError.message = e.what();
    moduleError.setCurrentVersionNumber({});
  }
} // namespace ChimeraTK::MotorDriver
