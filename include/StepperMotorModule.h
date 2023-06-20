// SPDX-FileCopyrightText: Deutsches Elektronen-Synchrotron DESY, MSK, ChimeraTK Project <chimeratk-support@desy.de>
// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "Motor.h"
#include "StepperMotorCtrl.h"
#include "StepperMotorReadback.h"

#include <ChimeraTK/ApplicationCore/ApplicationCore.h>
#include <ChimeraTK/ApplicationCore/ScriptedInitialisationHandler.h>
#include <ChimeraTK/MotorDriverCard/StepperMotor.h>

#include <memory>

namespace ChimeraTK::MotorDriver {
  struct StepperMotorModule : public ModuleGroup {
    StepperMotorModule(ModuleGroup* owner, const std::string& name, const std::string& description,
        const StepperMotorParameters& motorParameters, const std::string& triggerPath,
        const std::string& initScriptPath = {}, const std::unordered_set<std::string>& tags = {});

    std::shared_ptr<Motor> motor;
    DeviceModule motorProxyDevice;
    std::unique_ptr<ScriptedInitHandler> initHandler;
    ControlInputHandler ctrlInputHandler;
    ReadbackHandler readbackHandler;
  };
} // namespace ChimeraTK::MotorDriver
