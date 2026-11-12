// SPDX-FileCopyrightText: Deutsches Elektronen-Synchrotron DESY, MSK, ChimeraTK Project <chimeratk-support@desy.de>
// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <ChimeraTK/ApplicationCore/ApplicationCore.h>
#include <ChimeraTK/ApplicationCore/Logger.h>
#include <ChimeraTK/MotorDriverCard/StepperMotor.h>

namespace ChimeraTK::MotorDriver {
  class Motor {
   public:
    explicit Motor(StepperMotorParameters parameters) : _parameters(std::move(parameters)) {}

    void renew() {
      logger(Logger::Severity::debug, "Motor") << "Creating new motor object for " << toString() << std::endl;
      // Drop our reference to the StepperMotor. This should free the motor and the factory should give us a new one
      _motor.reset();
      try {
        _motor = StepperMotorFactory::instance().create(_parameters);
      }
      catch(std::exception& e) {
        logger(Logger::Severity::error, "Motor")
            << "Failed to create motor object for " << toString() << ": " << e.what() << std::endl;
        _open = false;
        throw;
      }
      _open = true;
      logger(Logger::Severity::info, "Motor")
          << "Motor object for " << toString() << " created, motor is open" << std::endl;
    }

    void close() {
      if(_open) {
        logger(Logger::Severity::warning, "Motor")
            << "Closing motor " << toString() << ", all control commands will be rejected until the device recovers"
            << std::endl;
      }
      _motor.reset();
      _open = false;
    }

    std::shared_ptr<StepperMotor> get() { return _motor; }

    [[nodiscard]] bool isOpen() const { return _open; }

    [[nodiscard]] std::string toString() const {
      return _parameters.deviceName + ":" + _parameters.moduleName + ":" + std::to_string(_parameters.driverId);
    }

    [[nodiscard]] const StepperMotorParameters& getMotorParameters() const { return _parameters; }

   private:
    std::shared_ptr<StepperMotor> _motor;
    StepperMotorParameters _parameters;
    bool _open{false};
  };
} // namespace ChimeraTK::MotorDriver
