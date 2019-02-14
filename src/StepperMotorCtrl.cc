/*
 * StepperMotorCtrl.cc
 *
 *  Created on: Sep 13, 2018
 *      Author: ckampm
 */

#include "StepperMotorCtrl.h"
#include "mtca4u/MotorDriverCard/MotorDriverException.h"


namespace ChimeraTK {
namespace MotorDriver {


ControlInputHandler::ControlInputHandler(EntityOwner *owner, const std::string &name, const std::string &description, std::shared_ptr<StepperMotor> motor)
    : ApplicationModule(owner, name, description),
      _motor(motor)
{
  // If motor has HW reference switches,
  // calibration is supported
  if(_motor->hasHWReferenceSwitches()){
      control.calibrationCtrl = CalibrationCommands{&control, "calibrationControl", "Calibration commands", true, {"MOTOR"}};
  }
}

void ControlInputHandler::createFunctionMap(std::shared_ptr<StepperMotor> motor){

  funcMap[control.enable.getId()]              = [this]{enableCallback();};
  funcMap[control.disable.getId()]             = [this]{disableCallback();};
  funcMap[control.start.getId()]               = [this]{startCallback();};
  funcMap[control.stop.getId()]                = [this, motor]{ if(control.stop){motor->stop();} };
  funcMap[control.emergencyStop.getId()]       = [this, motor]{ if(control.emergencyStop){ motor->emergencyStop();} };
  funcMap[control.resetError.getId()]          = [      motor]{ motor->resetError(); };
  funcMap[control.enableAutostart.getId()]     = [this, motor]{ motor->setAutostart(control.enableAutostart);};
  funcMap[control.enableFullStepping.getId()]  = [this, motor]{ motor->enableFullStepping(control.enableFullStepping); };


  funcMap[positionSetpoint.positionInSteps.getId()]         = [this, motor]{ _motor->setTargetPosition(positionSetpoint.positionInSteps); };
  funcMap[positionSetpoint.position.getId()]                = [this, motor]{ _motor->setTargetPosition(positionSetpoint.position); };
  funcMap[positionSetpoint.relativePositionInSteps.getId()] = [this, motor]{ motor->moveRelativeInSteps(positionSetpoint.relativePositionInSteps); };
  funcMap[positionSetpoint.relativePosition.getId()]        = [this, motor]{ motor->moveRelative(positionSetpoint.relativePosition); };

  funcMap[referenceSettings.positionInSteps.getId()]  = [this, motor]{ motor->setActualPositionInSteps(referenceSettings.positionInSteps); };
  funcMap[referenceSettings.position.getId()]         = [this, motor]{ motor->setActualPosition(referenceSettings.position); };
  funcMap[referenceSettings.encoderPosition.getId()]  = [this, motor]{ motor->setActualEncoderPosition(referenceSettings.encoderPosition); };
  funcMap[referenceSettings.axisTranslationInSteps.getId()]    = [this, motor]{ motor->translateAxisInSteps(referenceSettings.axisTranslationInSteps); };
  funcMap[referenceSettings.axisTranslation.getId()]           = [this, motor]{ motor->translateAxis(referenceSettings.axisTranslation); };

  funcMap[swLimits.enable.getId()]               = [this, motor]{ motor->setSoftwareLimitsEnabled(swLimits.enable); };
  funcMap[swLimits.maxPosition.getId()]          = [this, motor]{ motor->setMaxPositionLimit(swLimits.maxPosition); };
  funcMap[swLimits.minPosition.getId()]        = [this, motor]{ motor->setMinPositionLimit(swLimits.minPosition); };
  funcMap[swLimits.maxPositionInSteps.getId()] = [this, motor]{ motor->setMaxPositionLimitInSteps(swLimits.maxPositionInSteps); };
  funcMap[swLimits.minPositionInSteps.getId()] = [this, motor]{ motor->setMinPositionLimitInSteps(swLimits.minPositionInSteps); };

  funcMap[userLimits.current.getId()] = [this, motor]{ motor->setUserCurrentLimit(userLimits.current); };
  funcMap[userLimits.speed.getId()]   = [this, motor]{ motor->setUserSpeedLimit(userLimits.speed); };
}


void ControlInputHandler::prepare(){
    createFunctionMap(_motor);

    if(_motor->hasHWReferenceSwitches()){
      appendCalibrationToMap();
    }
}

void ControlInputHandler::mainLoop() {

  inputGroup = this->readAnyGroup();

  while(true){

    notification.message = "";
    notification.hasMessage = 0;

    auto changedVarId = inputGroup.readAny();

    //FIXME Keep this only as long as we rely on the dummy for tests
    try{
      funcMap.at(changedVarId)();
    }
    catch(mtca4u::MotorDriverException &e){
      notification.message = "Exception: " + std::string(e.what());
    }

    if(std::string("").compare(notification.message)){
      notification.hasMessage = 1;
    }

    dummySignals.dummyMotorStop = control.stop || control.emergencyStop;
    dummySignals.dummyMotorTrigger++;

    writeAll();
  }
}

void ControlInputHandler::enableCallback(){
  if(control.enable){
    _motor->setEnabled(true);
  }
}

void ControlInputHandler::disableCallback(){
  if(control.disable){
    _motor->setEnabled(false);
  }
}

void ControlInputHandler::startCallback(){

  if(control.start){
    if(_motor->isSystemIdle()){
      _motor->start();
    }
    else{
      notification.message = "WARNING: Called startMotor while motor is not in IDLE state.";
    }
  }
}

void ControlInputHandler::appendCalibrationToMap(){

  funcMap[control.calibrationCtrl.calibrateMotor.getId()]     = [this]{calibrateCallback();};
  funcMap[control.calibrationCtrl.determineTolerance.getId()] = [this]{determineToleranceCallback();};
}

void ControlInputHandler::calibrateCallback(){
  if(control.calibrationCtrl.calibrateMotor){
    if(_motor->isSystemIdle()){
      _motor->calibrate();
    }
    else{
      notification.message = "WARNING: Called calibrateMotor while motor is not in IDLE state.";
    }
  }
}

void ControlInputHandler::determineToleranceCallback(){
  if(control.calibrationCtrl.determineTolerance){
    if(_motor->isSystemIdle()){
      _motor->determineTolerance();
    }
    else{
      notification.message = "WARNING: Called determineTolerance while motor is not in IDLE state.";
    }
  }
}
}
}
