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
#include "Motor.h"

#include <memory>

namespace ChimeraTK { namespace MotorDriver {

  struct StepperMotorModule : public ModuleGroup {
    StepperMotorModule(EntityOwner* owner, const std::string& name, const std::string& description,
        const StepperMotorParameters& motorParameters, const std::unordered_set<std::string>& tags = {});

    std::shared_ptr<Motor> motor;
    ControlInputHandler ctrlInputHandler;
    ReadbackHandler readbackHandler;
  };
}} // namespace ChimeraTK::MotorDriver

#endif /* INCLUDE_STEPPERMOTORMODULE_H_ */
