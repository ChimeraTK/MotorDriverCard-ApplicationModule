// SPDX-FileCopyrightText: Deutsches Elektronen-Synchrotron DESY, MSK, ChimeraTK Project <chimeratk-support@desy.de>
// SPDX-License-Identifier: LGPL-3.0-or-later
#include "StatusModule.h"

ChimeraTK::StatusModule::StatusModule(ModuleGroup* owner)
: ApplicationModule(owner, "StatusPropagator", "Module to propagate states") {}

ChimeraTK::DataValidity ChimeraTK::StatusModule::getDataValidity() const {
  return DataValidity::ok;
}

void ChimeraTK::StatusModule::mainLoop() {
  auto group = readAnyGroup();
  TransferElementID id{};
  do {
    if(!id.isValid() || id == messageIn.getId()) {
      if(std::string(message) != std::string(messageIn)) {
        info() << "Motor notification message changed to '" << std::string(messageIn) << "'" << std::endl;
      }
      message.setAndWrite(std::string(messageIn));
      hasMessage.setAndWrite(!std::string(message).empty());
    }

    if(!id.isValid() || id == motorStateIn.getId()) {
      if(std::string(motorState) != std::string(motorStateIn)) {
        info() << "Motor state changed to '" << std::string(motorStateIn) << "'" << std::endl;
      }
      motorState.writeIfDifferent(std::string(motorStateIn));
    }
    id = group.readAny();
  } while(true);
}
