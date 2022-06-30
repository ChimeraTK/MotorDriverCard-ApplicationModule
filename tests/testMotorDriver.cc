// Define a name for the test module.
#define BOOST_TEST_MODULE testMotorDriverCard_ApplicationModule
// Only after defining the name include the unit test header.
#include <boost/test/included/unit_test.hpp>

#include "StepperMotorModule.h"

#include <ChimeraTK/ApplicationCore/ApplicationCore.h>
#include <ChimeraTK/ApplicationCore/TestFacility.h>

// Includes to use to dummy motor
#include "mtca4u/MotorDriverCard/MotorControlerDummy.h"
#include "mtca4u/MotorDriverCard/MotorDriverCard.h"
#include "mtca4u/MotorDriverCard/MotorDriverCardFactory.h"

#include <memory>

namespace ctk = ChimeraTK;

static const std::string stepperMotorDeviceName("STEPPER-MOTOR-DUMMY");
static const std::string stepperMotorDeviceConfigFile("VT21-MotorDriverCardConfig.xml");
static const std::string dmapPath(".");
static const std::string moduleName("");

// Test server
struct TestServer : public ctk::Application {
  TestServer() : ctk::Application("MotorDriverCard_ApplicationModule_TestServer") {}
  ~TestServer() override { shutdown(); }

  ctk::ControlSystemModule cs{};

  // Motor instance and parameters
  ctk::MotorDriver::StepperMotorParameters parameters = {
      ctk::MotorDriver::StepperMotorType::LINEAR, stepperMotorDeviceName, moduleName, 0U, stepperMotorDeviceConfigFile};
  std::unique_ptr<ctk::MotorDriver::StepperMotorModule> motor;

  void defineConnections() override {
    motor = std::make_unique<ctk::MotorDriver::StepperMotorModule>(this, "Motor", "", parameters);

    cs["Trigger"]("tick") >> motor->readbackHandler("tick");
    motor->findTag("MOTOR|MOT_DIAG").connectTo(cs["Motor"]);

    //      cs.dump();
    //      dumpConnections();
  }
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
        stepperMotorDeviceName, moduleName, stepperMotorDeviceConfigFile);
    _motorControlerDummy =
        boost::dynamic_pointer_cast<mtca4u::MotorControlerDummy>(_motorDriverCard->getMotorControler(0));
  }

  boost::shared_ptr<mtca4u::MotorDriverCard> _motorDriverCard;
  boost::shared_ptr<mtca4u::MotorControlerDummy> _motorControlerDummy;
};

using namespace boost::unit_test_framework;

/**********************************************************************************************************************/

BOOST_FIXTURE_TEST_CASE(testMoving, TestFixture) {
  std::cout << "testMoving" << std::endl;

  TestServer testServer;
  ChimeraTK::TestFacility testFacility;
  testFacility.runApplication();

  auto trigger = testFacility.getScalar<uint64_t>("Trigger/tick");
  auto motorState = testFacility.getScalar<std::string>("Motor/readback/status/state");

  // Enable he dummy motor
  // FIXME: If the dummy is enabled here or not does not make a difference. The test still passes. Check the test code.
  _motorControlerDummy->setEnabled(true);
  _motorControlerDummy->setMotorCurrentEnabled(true);

  // Application should start with disabled motor (initial value being send)
  motorState.read(); // read initial value
  BOOST_CHECK_EQUAL(static_cast<std::string>(motorState), "disabled");

  // Enable stepper motor and set a reference position
  // (the latter should yield ChimeraTK::MotorDriver::CalibrationMode::SIMPLE)
  testFacility.writeScalar<int>("Motor/controlInput/control/enable", 1);
  testFacility.writeScalar<int>("Motor/controlInput/control/enableAutostart", 1);
  testFacility.writeScalar<int>("Motor/controlInput/referenceSettings/positionInSteps", 0);

  testFacility.stepApplication();
  trigger.write();
  testFacility.stepApplication();
  motorState.read();

  BOOST_CHECK_EQUAL(testFacility.readScalar<int>("Motor/readback/status/autostartEnabled"), 1);
  BOOST_CHECK_EQUAL(static_cast<std::string>(motorState), "idle");
  BOOST_CHECK_EQUAL(testFacility.readScalar<int>("Motor/controlInput/notification/hasMessage"), 0);
  BOOST_CHECK_EQUAL(testFacility.readScalar<int>("Motor/readback/status/calibrationMode"),
      static_cast<int>(ChimeraTK::MotorDriver::CalibrationMode::SIMPLE));

  // Apply a position setpoint
  int targetPosition{1000};
  testFacility.writeScalar<int>("Motor/controlInput/positionSetpoint/positionInSteps", targetPosition);

  testFacility.stepApplication();
  trigger.write();
  testFacility.stepApplication();
  motorState.read();

  // Target value should have been applied and module transitioned ito state "moving"
  BOOST_CHECK_EQUAL(testFacility.readScalar<int>("Motor/controlInput/notification/hasMessage"), 0);
  BOOST_CHECK_EQUAL(testFacility.readScalar<int>("Motor/readback/position/targetValueInSteps"), targetPosition);
  BOOST_CHECK_EQUAL(static_cast<std::string>(motorState), "moving");

  // Move the dummy
  // Afterwards target postion should be read back as actual value and system in state "idle"
  _motorControlerDummy->moveTowardsTarget(1.f);

  trigger.write();
  testFacility.stepApplication();
  motorState.read();

  BOOST_CHECK_EQUAL(testFacility.readScalar<int>("Motor/readback/position/actualValueInSteps"), targetPosition);
  BOOST_CHECK_EQUAL(static_cast<std::string>(motorState), "idle");
}

/**********************************************************************************************************************/

BOOST_FIXTURE_TEST_CASE(testStartupWithSimpleCalibration, TestFixture) {
  std::cout << "testStartup (calibration mode SIMPLE)" << std::endl;

  // Enable he dummy motor
  _motorControlerDummy->setEnabled(true);
  _motorControlerDummy->setMotorCurrentEnabled(true);

  // Set the calibration time to a nonzero value
  // -> Motor instance should be initialized with CalibrationMode::SIMPLE
  _motorControlerDummy->setCalibrationTime(123456U);

  TestServer testServer;
  ChimeraTK::TestFacility testFacility;
  testFacility.runApplication();

  auto trigger = testFacility.getScalar<uint64_t>("Trigger/tick");
  auto motorState = testFacility.getScalar<std::string>("Motor/readback/status/state");

  trigger.write();
  testFacility.stepApplication();
  motorState.read();

  // Application should start with disabled motor
  // We currently have to test for FULL instead of SIMPLE, because the MotorControlerDummy does
  // not yet support storage of the calibration  (TODO, See MotorDriverCard issue #9).
  //BOOST_CHECK_EQUAL(testFacility.readScalar<int>("Motor/readback/status/calibrationMode"), static_cast<int>(ChimeraTK::MotorDriver::CalibrationMode::FULL));
}

// TODO Add test for CalibrationMode::FULL, when MotorControlerDummy supports this
//      (See MotorDriverCard issue #9).
//BOOST_FIXTURE_TEST_CASE(testStartup, TestFixture){
//  std::cout << "testStartup (calibration mode FULL)" << std::endl;

//  // Enable he dummy motor
//  _motorControlerDummy->setEnabled(true);
//  _motorControlerDummy->setMotorCurrentEnabled(true);

//  // Set the calibration time to a nonzero value and proper end switch positions
//  // -> Motor instance should be initialized with CalibrationMode::SIMPLE
//  _motorControlerDummy->setCalibrationTime(123456U);
//  _motorControlerDummy->setPositiveReferenceSwitchCalibration(1000);
//  _motorControlerDummy->setNegativeReferenceSwitchCalibration(-1000);

//  TestServer testServer;
//  ChimeraTK::TestFacility testFacility;
//  testFacility.runApplication();

//  auto  trigger = testFacility.getScalar<uint64_t>("Trigger/tick");
//  auto motorState  = testFacility.getScalar<std::string>("Motor/readback/status/state");

//  trigger.write();
//  testFacility.stepApplication();
//  motorState.read();

//  // Application should start with disabled motor
//  BOOST_CHECK_EQUAL(testFacility.readScalar<int>("Motor/readback/status/calibrationMode"),
//                                                 static_cast<int>(ChimeraTK::MotorDriver::CalibrationMode::FULL));

//}
