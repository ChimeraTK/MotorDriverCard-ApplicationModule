/*
 * StepperMotorModule.cc
 *
 *  Created on: Jul 17, 2018
 *      Author: ckampm
 */

#include "StepperMotorModule.h"


namespace ChimeraTK {
namespace MotorDriver {

  StepperMotorModule::StepperMotorModule(ModuleGroup* owner, const std::string& name, const std::string& description,
      const StepperMotorParameters& motorParameters, const std::string& triggerPath,
      const std::unordered_set<std::string>& tags)
  : ModuleGroup(owner, name, description, tags), motor{std::make_shared<Motor>(motorParameters)},
    ctrlInputHandler{this, "controlInput", "Handles the control input to the motor driver.", motor},
    readbackHandler{motor, this, "readback", "Signals read from the motor driver", triggerPath} {}

}
}
