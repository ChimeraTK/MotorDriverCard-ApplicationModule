// SPDX-FileCopyrightText: Deutsches Elektronen-Synchrotron DESY, MSK, ChimeraTK Project <chimeratk-support@desy.de>
// SPDX-License-Identifier: LGPL-3.0-or-later

// Define a name for the test module.
#define BOOST_TEST_MODULE testMotorDriverCard_ApplicationModule
// Only after defining the name include the unit test header.
#include "StepperMotorModule.h"

#include <ChimeraTK/ApplicationCore/ApplicationCore.h>
#include <ChimeraTK/ApplicationCore/TestFacility.h>

#include <boost/test/included/unit_test.hpp>

// Includes to use to dummy motor
#include "mtca4u/MotorDriverCard/MotorControlerDummy.h"
#include "mtca4u/MotorDriverCard/MotorDriverCard.h"
#include "mtca4u/MotorDriverCard/MotorDriverCardFactory.h"

#include <memory>

namespace ctk = ChimeraTK;

constexpr auto STEPPER_MOTOR_DEVICE_NAME = "STEPPER-MOTOR-DUMMY";
constexpr auto STEPPER_MOTOR_DEVICE_CONFIG_FILE = "VT21-MotorDriverCardConfig.xml";

// Test server
struct TestServer : public ctk::Application {
  TestServer() : ctk::Application("MotorDriverCard_ApplicationModule_TestServer") {
    motor =
        std::make_unique<ctk::MotorDriver::StepperMotorModule>(this, "Motor", "", parameters, "/Motor/Readback/tick");
  }
  ~TestServer() override { shutdown(); }

  // Motor instance and parameters
  ctk::MotorDriver::StepperMotorParameters parameters = {
      ctk::MotorDriver::StepperMotorType::LINEAR, STEPPER_MOTOR_DEVICE_NAME, {}, 0U, STEPPER_MOTOR_DEVICE_CONFIG_FILE};
  std::unique_ptr<ctk::MotorDriver::StepperMotorModule> motor;
};

/**
 * TestFixture struct
 *
 * Only holds a MotorControlerDummy to mimic motor functionality.
 * The Application and TestFacility have to be created in each test case,
 * so that we can simulate different init scenarios (depending on the calibration state)
 * of the StepperMotorModule.
 */
struct TestFixture {
  TestFixture() {
    mtca4u::MotorDriverCardFactory::instance().setDummyMode();
    ChimeraTK::setDMapFilePath("./MD22_on_DAMC2.dmap");

    _motorDriverCard = mtca4u::MotorDriverCardFactory::instance().createMotorDriverCard(
        STEPPER_MOTOR_DEVICE_NAME, {}, STEPPER_MOTOR_DEVICE_CONFIG_FILE);
    _motorControlerDummy =
        boost::dynamic_pointer_cast<mtca4u::MotorControlerDummy>(_motorDriverCard->getMotorControler(0));
    _motorControlerDummy->resetInternalStateToDefaults();
  }

  void calibrateFull(uint32_t calibrationTime = 123456U, int negativeReference = -1000, int positiveReference = 1000) {
    // Set the calibration time to a nonzero value and proper end switch positions
    // -> Motor instance should be initialized with CalibrationMode::SIMPLE
    _motorControlerDummy->setCalibrationTime(calibrationTime);
    _motorControlerDummy->setPositiveReferenceSwitchCalibration(positiveReference);
    _motorControlerDummy->setNegativeReferenceSwitchCalibration(negativeReference);
  }

  boost::shared_ptr<mtca4u::MotorDriverCard> _motorDriverCard;
  boost::shared_ptr<mtca4u::MotorControlerDummy> _motorControlerDummy;
};

using namespace boost::unit_test_framework;

/**********************************************************************************************************************/

BOOST_FIXTURE_TEST_CASE(testMoving, TestFixture) {
  std::cout << "testMoving" << std::endl;

  TestServer testServer;
  ChimeraTK::TestFacility testFacility{testServer};
  testFacility.runApplication();

  auto trigger = testFacility.getVoid("/Motor/Readback/tick");
  auto motorState = testFacility.getScalar<std::string>("/Motor/Readback/Status/state");

  // Enable the dummy motor
  // FIXME: If the dummy is enabled here or not does not make a difference. The test still passes. Check the test code.
  _motorControlerDummy->setEnabled(true);
  _motorControlerDummy->setMotorCurrentEnabled(true);

  BOOST_CHECK(motorState.dataValidity() == ctk::DataValidity::ok);

  // Trigger readout loop once, so the motor device "wakes up" and actually initializes itself
  testServer.motor->motorProxyDevice.reportException("Exception trigger from test to toggle deviceBecameFunctional");
  testFacility.stepApplication();
  // Get rid of all the intermediate changes due to exception reporting
  motorState.readLatest();

  // Application should start with disabled motor (initial value being send)
  // Directly check. Initial values are already received.
  BOOST_CHECK_EQUAL(static_cast<std::string>(motorState), "disabled");

  testFacility.writeScalar<int>("Motor/ControlInput/ReferenceSettings/positionInSteps", 0);
  testFacility.stepApplication();

  // Enable stepper motor and set a reference position
  // (the latter should yield ChimeraTK::MotorDriver::CalibrationMode::SIMPLE)
  testFacility.getVoid("Motor/ControlInput/Control/enable").write();
  testFacility.stepApplication();
  motorState.read();

  testFacility.writeScalar<ChimeraTK::Boolean>("Motor/ControlInput/Control/enableAutostart", true);
  testFacility.stepApplication();
  trigger.write();
  testFacility.stepApplication();

  BOOST_CHECK_EQUAL(testFacility.readScalar<int>("Motor/Readback/Status/autostartEnabled"), 1);
  BOOST_CHECK_EQUAL(static_cast<std::string>(motorState), "idle");
  BOOST_CHECK_EQUAL(testFacility.readScalar<ChimeraTK::Boolean>("Motor/ControlInput/Notification/hasMessage"), false);
  BOOST_CHECK_EQUAL(testFacility.readScalar<int>("Motor/Readback/Status/calibrationMode"),
      static_cast<int>(ChimeraTK::MotorDriver::CalibrationMode::SIMPLE));

  // Apply a position setpoint
  int targetPosition{1000};
  testFacility.writeScalar<int>("Motor/ControlInput/PositionSetpoint/positionInSteps", targetPosition);

  testFacility.stepApplication();
  trigger.write();
  testFacility.stepApplication();
  motorState.read();

  // Target value should have been applied and module transitioned ito state "moving"
  BOOST_CHECK_EQUAL(testFacility.readScalar<ChimeraTK::Boolean>("Motor/ControlInput/Notification/hasMessage"), false);
  BOOST_CHECK_EQUAL(testFacility.readScalar<int>("Motor/Readback/Position/targetValueInSteps"), targetPosition);
  BOOST_CHECK_EQUAL(static_cast<std::string>(motorState), "moving");

  // Move the dummy
  // Afterwards target position should be read back as actual value and system in state "idle"
  _motorControlerDummy->moveTowardsTarget(1.F);

  trigger.write();
  testFacility.stepApplication();
  motorState.readNonBlocking();

  trigger.write();
  testFacility.stepApplication();
  motorState.readNonBlocking();

  BOOST_CHECK_EQUAL(testFacility.readScalar<int>("Motor/Readback/Position/actualValueInSteps"), targetPosition);
  BOOST_CHECK_EQUAL(static_cast<std::string>(motorState), "idle");
}

/**********************************************************************************************************************/

BOOST_FIXTURE_TEST_CASE(testStartupWithSimpleCalibration, TestFixture) {
  std::cout << "testStartup (calibration mode SIMPLE)" << std::endl;

  // Enable he dummy motor
  _motorControlerDummy->setEnabled(true);
  _motorControlerDummy->setMotorCurrentEnabled(true);

  // Set the calibration time to a nonzero value, limits min/max
  // -> Motor instance should be initialized with CalibrationMode::SIMPLE
  calibrateFull(123456U, std::numeric_limits<int>::min(), std::numeric_limits<int>::max());

  TestServer testServer;
  ChimeraTK::TestFacility testFacility{testServer};
  testFacility.runApplication();

  auto trigger = testFacility.getVoid("Motor/Readback/tick");
  auto motorState = testFacility.getScalar<std::string>("Motor/Readback/Status/state");

  BOOST_CHECK(motorState.dataValidity() == ctk::DataValidity::ok);

  // Trigger readout loop once, so the motor device "wakes up" and actually initializes itself
  testServer.motor->motorProxyDevice.reportException("Exception trigger from test to toggle deviceBecameFunctional");
  testFacility.stepApplication();
  motorState.readLatest();

  trigger.write();
  testFacility.stepApplication();

  // Application should start with disabled motor
  BOOST_CHECK_EQUAL(testFacility.readScalar<int>("Motor/Readback/Status/calibrationMode"),
      static_cast<int>(ChimeraTK::MotorDriver::CalibrationMode::SIMPLE));
}

BOOST_FIXTURE_TEST_CASE(testStartup, TestFixture) {
  std::cout << "testStartup (calibration mode FULL)" << std::endl;

  // Enable he dummy motor
  _motorControlerDummy->setEnabled(true);
  _motorControlerDummy->setMotorCurrentEnabled(true);

  // Set the calibration time to a nonzero value and proper end switch positions
  // -> Motor instance should be initialized with CalibrationMode::FULL
  calibrateFull();

  TestServer testServer;
  ChimeraTK::TestFacility testFacility{testServer};
  testFacility.runApplication();

  auto trigger = testFacility.getVoid("Motor/Readback/tick");
  auto motorState = testFacility.getScalar<std::string>("Motor/Readback/Status/state");

  trigger.write();
  testFacility.stepApplication();
  motorState.read();

  // Application should start with disabled motor
  BOOST_CHECK_EQUAL(testFacility.readScalar<int>("Motor/Readback/Status/calibrationMode"),
      static_cast<int>(ChimeraTK::MotorDriver::CalibrationMode::FULL));
}

BOOST_FIXTURE_TEST_CASE(testFullStepping, TestFixture) {
  std::cout << "testFullStepping" << std::endl;

  _motorControlerDummy->setEnabled(true);
  _motorControlerDummy->setMotorCurrentEnabled(true);

  calibrateFull(123456U);

  TestServer testServer;
  ChimeraTK::TestFacility testFacility{testServer};
  testFacility.runApplication();

  auto trigger = testFacility.getVoid("/Motor/Readback/tick");
  auto motorState = testFacility.getScalar<std::string>("/Motor/Readback/Status/state");
  auto message = testFacility.getScalar<std::string>("/Motor/ControlInput/Notification/message");

  trigger.write();
  testFacility.stepApplication();

  // Throw away any messages and state changes until now
  motorState.readLatest();
  message.readLatest();

  // Make sure we are some kind of calibrated
  BOOST_TEST(testFacility.readScalar<int>("Motor/Readback/Status/calibrationMode") !=
      static_cast<int>(ChimeraTK::MotorDriver::CalibrationMode::NONE));

  // Enable Fullstepping
  testFacility.writeScalar<ChimeraTK::Boolean>("/Motor/ControlInput/Control/enableFullStepping", true);
  testFacility.stepApplication();

  // Enable the motor and autostart
  testFacility.getVoid("Motor/ControlInput/Control/enable").write();
  testFacility.writeScalar<ChimeraTK::Boolean>("Motor/ControlInput/Control/enableAutostart", true);

  testFacility.stepApplication();

  trigger.write();
  testFacility.stepApplication();

  BOOST_CHECK(motorState.readNonBlocking() == true);
  BOOST_TEST(std::string(motorState) == "idle");

  trigger.write();
  testFacility.stepApplication();

  BOOST_CHECK(motorState.readNonBlocking() == false);
  BOOST_TEST(std::string(motorState) == "idle");
  message.readLatest();

  int targetPosition{23};
  testFacility.writeScalar<int>("Motor/ControlInput/PositionSetpoint/positionInSteps", targetPosition);
  testFacility.stepApplication();

  BOOST_CHECK(message.readNonBlocking() == true);
  BOOST_TEST(std::string(message) == "");
  BOOST_CHECK(motorState.readNonBlocking() == true);
  BOOST_TEST(std::string(motorState) == "moving");

  trigger.write();
  testFacility.stepApplication();

  BOOST_CHECK(motorState.readNonBlocking() == true);
  BOOST_CHECK(std::string(motorState) == "idle");
}

BOOST_FIXTURE_TEST_CASE(testCalibrationEmergencyStop, TestFixture) {
  std::cout << "testCalibrationEmergencyStop" << std::endl;

  TestServer testServer;
  ChimeraTK::TestFacility testFacility{testServer};
  testFacility.runApplication();

  auto trigger = testFacility.getVoid("/Motor/Readback/tick");
  auto motorState = testFacility.getScalar<std::string>("/Motor/Readback/Status/state");

  // Enable the dummy motor
  // FIXME: If the dummy is enabled here or not does not make a difference. The test still passes. Check the test code.
  _motorControlerDummy->setEnabled(true);
  _motorControlerDummy->setMotorCurrentEnabled(true);

  // Remove all calibration on the controller
  calibrateFull(0, std::numeric_limits<int>::min(), std::numeric_limits<int>::max());

  // This is used to make the move below stop before the endswitch
  // The default end switch position in the dummy is at +/-10000
  // while POS_BEYOND_POSITIVE_ENDSWITCH is 50000. So the moveTowardsTarget(1.) will hit the endswitch and the
  // emergency stop will not have any effect at all - I think.
  _motorControlerDummy->setPositiveEndSwitch(60000);
  _motorControlerDummy->setNegativeEndSwitch(-60000);

  BOOST_CHECK(motorState.dataValidity() == ctk::DataValidity::ok);

  // Trigger readout loop once, so the motor device "wakes up" and actually initializes itself
  testServer.motor->motorProxyDevice.reportException("Exception trigger from test to toggle deviceBecameFunctional");
  testFacility.stepApplication();
  // Get rid of all the intermediate changes due to exception reporting
  motorState.readLatest();

  // Application should start with disabled motor (initial value being send)
  // Directly check. Initial values are already received.
  BOOST_CHECK_EQUAL(static_cast<std::string>(motorState), "disabled");

  // Enable stepper motor and set a reference position
  // (the latter should yield ChimeraTK::MotorDriver::CalibrationMode::SIMPLE)
  testFacility.getVoid("Motor/ControlInput/Control/enable").write();
  testFacility.writeScalar<ChimeraTK::Boolean>("Motor/ControlInput/Control/enableAutostart", true);
  testFacility.stepApplication();
  motorState.readLatest();
  trigger.write();
  testFacility.stepApplication();

  BOOST_CHECK_EQUAL(testFacility.readScalar<int>("Motor/Readback/Status/autostartEnabled"), 1);
  BOOST_CHECK_EQUAL(static_cast<std::string>(motorState), "idle");
  BOOST_CHECK_EQUAL(testFacility.readScalar<ChimeraTK::Boolean>("Motor/ControlInput/Notification/hasMessage"), false);

  testFacility.getVoid("Motor/ControlInput/Control/calibrate").write();
  testFacility.stepApplication();
  trigger.write();
  testFacility.stepApplication();
  trigger.write();
  testFacility.stepApplication();
  motorState.read();

  // Move the dummy
  // Afterwards target position should be read back as actual value and system in state "idle"
  BOOST_CHECK_EQUAL(static_cast<std::string>(motorState), "calibrating");

  int i = 0;
  do {
    if(i < 50) {
      _motorControlerDummy->moveTowardsTarget(i / 100.F);
    }
    trigger.write();
    if(i == 50) {
      std::cout << "Performing emergency stop" << std::endl;
      testFacility.getVoid("Motor/ControlInput/Control/emergencyStop").write();
    }
    testFacility.stepApplication();
    i++;
  } while(!motorState.readNonBlocking() && (std::string)motorState == std::string{"calibrating"});

  BOOST_CHECK_EQUAL(static_cast<std::string>(motorState), "error");

  BOOST_CHECK_EQUAL(testFacility.readScalar<int>("Motor/Readback/Status/calibrationMode"),
      static_cast<int>(ChimeraTK::MotorDriver::CalibrationMode::NONE));
}
