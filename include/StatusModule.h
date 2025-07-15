// SPDX-FileCopyrightText: Deutsches Elektronen-Synchrotron DESY, MSK, ChimeraTK Project <chimeratk-support@desy.de>
// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include <ChimeraTK/ApplicationCore/ApplicationModule.h>
#include <ChimeraTK/ApplicationCore/ScalarAccessor.h>

// This module makes sure that all the status information about the motor never changes in validity.
// It should always be DataValidity::ok since it is information about the device.
namespace ChimeraTK {
  struct StatusModule : public ApplicationModule {
    explicit StatusModule(ModuleGroup* owner);

    ScalarOutput<std::string> message{this, "../ControlInput/Notification/message", "n/a", "", {"MOTOR"}};
    ScalarOutput<Boolean> hasMessage{this, "../ControlInput/Notification/hasMessage", "n/a", "", {"MOTOR"}};
    ScalarOutput<std::string> motorState{this, "../Readback/Status/state", "n/a", "", {"MOTOR"}};

    ScalarPushInput<std::string> messageIn{this, "message", "n/a", ""};
    ScalarPushInput<std::string> motorStateIn{this, "motorState", "n/a", ""};

    DataValidity getDataValidity() const override;
    void mainLoop() override;
  };
} // namespace ChimeraTK
