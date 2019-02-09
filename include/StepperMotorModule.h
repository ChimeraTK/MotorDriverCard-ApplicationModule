/*
 * StepperMotorModule.h
 *
 *  Created on: Jul 16, 2018
 *      Author: ckampm
 */

#ifndef INCLUDE_STEPPERMOTORMODULE_H_
#define INCLUDE_STEPPERMOTORMODULE_H_

#include <ChimeraTK/ApplicationCore/ApplicationCore.h>
#include <ChimeraTK/MotorDriverCard/StepperMotor.h>

#include "StepperMotorCtrl.h"
#include "StepperMotorReadback.h"

#include <memory>

namespace ChimeraTK {
namespace MotorDriver {

struct StepperMotorModule : public ModuleGroup {
  StepperMotorModule(EntityOwner *owner, const std::string &name, const std::string &description,
                    const StepperMotorParameters &motorParameters);

  std::shared_ptr<StepperMotor> motor;
  ControlInputHandler ctrlInputHandler;
  ReadbackHandler readbackHandler;
};
} // namespace MotorDriver
} // namespace ChimeraTK


#endif /* INCLUDE_STEPPERMOTORMODULE_H_ */
