#pragma once

#include <ChimeraTK/ApplicationCore/ApplicationCore.h>
#include <ChimeraTK/MotorDriverCard/StepperMotor.h>

namespace ChimeraTK { namespace MotorDriver {
  class Motor {
   public:
    Motor(const StepperMotorParameters& parameters) : _parameters(parameters) { renew(); }

    void renew() {
      // Drop reference to
      _motor.reset();
      _motor = StepperMotorFactory::instance().create(_parameters);
      _open = true;
    }

    void close() {
      _motor.reset();
      _open = false;
    }

    std::shared_ptr<StepperMotor> get() { return _motor; }

    bool isOpen() { return _open; }

    std::string toString() {
      return _parameters.deviceName + ":" + _parameters.moduleName + ":" + std::to_string(_parameters.driverId);
    }

   private:
    std::shared_ptr<StepperMotor> _motor;
    StepperMotorParameters _parameters;
    bool _open{false};
  };
}} // namespace ChimeraTK::MotorDriver
