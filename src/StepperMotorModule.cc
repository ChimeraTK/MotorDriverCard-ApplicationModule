/*
 * StepperMotorModule.cc
 *
 *  Created on: Jul 17, 2018
 *      Author: ckampm
 */

#include "StepperMotorModule.h"
#include <memory>


namespace ChimeraTK {
namespace MotorDriver {

StepperMotorModule::StepperMotorModule(EntityOwner *owner, const std::string &name, const std::string &description,
                                   const StepperMotorParameters &motorParameters)
  : ModuleGroup(owner, name, description),
  motor{StepperMotorFactory::instance().create(motorParameters)},
  ctrlInputHandler{this, "controlInput", "Handles the control input to the motor driver.", motor},
  readbackHandler{motor, this, "readback", "Signals read from the motor driver"}{}

}
}
