/*
 * MotorDriver.h
 *
 *  Created on: Jul 16, 2018
 *      Author: ckampm
 */

#ifndef INCLUDE_MOTORDRIVER_H_
#define INCLUDE_MOTORDRIVER_H_

#include <ChimeraTK/ApplicationCore/ApplicationCore.h>
//#include <ChimeraTK/ReadAnyGroup.h>
#include <ChimeraTK/MotorDriverCard/StepperMotor.h>

#include "ControlInput.h"
#include "Readback.h"

#include <memory>

namespace ctkmot = ChimeraTK::MotorDriver;


struct MotorDriver : public ctk::ModuleGroup {
  MotorDriver(ctk::EntityOwner *owner, const std::string &name, const std::string &description,
                    const ctkmot::StepperMotorParameters &motorParameters);

  std::shared_ptr<ctkmot::StepperMotor> motor;
  ControlInputHandler ctrlInputHandler;
  ReadbackHandler readbackHandler;
};


#endif /* INCLUDE_MOTORDRIVER_H_ */
