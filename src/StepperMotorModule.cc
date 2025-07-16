// SPDX-FileCopyrightText: Deutsches Elektronen-Synchrotron DESY, MSK, ChimeraTK Project <chimeratk-support@desy.de>
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "StepperMotorModule.h"

namespace ChimeraTK::MotorDriver {

  StepperMotorModule::StepperMotorModule(ModuleGroup* owner, const std::string& name, const std::string& description,
      const StepperMotorParameters& motorParameters, const std::string& triggerPath, const std::string& initScriptPath,
      const std::unordered_set<std::string>& tags)
  : ModuleGroup(owner, name, description, tags), motor{std::make_shared<Motor>(motorParameters)},
    // Driver ID is only used to to get two different devices
    motorProxyDevice{this,
        "(logicalNameMap?map=motorProxy.xlmap&target=" + motor->getMotorParameters().deviceName +
            "&monitorRegister=" + motor->getMotorParameters().moduleName + "/WORD_PROJ_VERSION" +
            "&driverId=" + std::to_string(motor->getMotorParameters().driverId) + ")",
        triggerPath},
    ctrlInputHandler{
        this, "controlInput", "Handles the control input to the motor driver.", motor, triggerPath, &motorProxyDevice},
    readbackHandler{motor, this, "readback", "Signals read from the motor driver", triggerPath, &motorProxyDevice} {
    if(!initScriptPath.empty()) {
      initHandler = std::make_unique<ScriptedInitHandler>(this, "ExternalScript", "", initScriptPath, motorProxyDevice);
    }

    // Add our own initialisation handler here so it is called after the init script handler that may or may not have
    // been passed to the StepperMotorModule
    motorProxyDevice.addInitialisationHandler([this](auto& device) {
      try {
        // Poke at the motor. We already know that the underlying transport works when we get called, so this is to do
        // some SPI communication and see if this layer works. If not, hold the init process for a while and then notify
        // the device module that something isn't right. This is mainly to prevent excessive spamming
        device.open();
        motor->renew();
      }
      catch(ChimeraTK::runtime_error&) {
        boost::this_thread::sleep_for(boost::chrono::seconds(5));
        throw;
      }
    });
  }

} // namespace ChimeraTK::MotorDriver
