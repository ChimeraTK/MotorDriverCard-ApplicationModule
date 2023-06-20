// SPDX-FileCopyrightText: Deutsches Elektronen-Synchrotron DESY, MSK, ChimeraTK Project <chimeratk-support@desy.de>
// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <ChimeraTK/ApplicationCore/ApplicationCore.h>
#include <ChimeraTK/MotorDriverCard/StepperMotor.h>

namespace ChimeraTK::MotorDriver {
  class Motor {
   public:
    explicit Motor(StepperMotorParameters parameters) : _parameters(std::move(parameters)) {}

    void renew() {
      // Drop our reference to the StepperMotor. This should free the motor and the factory should give us a new one
      _motor.reset();
      _motor = StepperMotorFactory::instance().create(_parameters);
      _open = true;
    }

    void close() {
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
