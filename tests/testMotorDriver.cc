// Define a name for the test module.
#define BOOST_TEST_MODULE testMotorDriverCard-ApplicationModule
// Only after defining the name include the unit test header.
#include <boost/test/included/unit_test.hpp>

#include <ChimeraTK/MotorDriverCard/StepperMotorModule.h>

#include <ChimeraTK/ApplicationCore/ApplicationCore.h>
#include <ChimeraTK/ApplicationCore/TestFacility.h>
//#include <ChimeraTK/ApplicationCore/PeriodicTrigger.h>

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


/**
 * TestFixture struct
 *
 * Provides a test Application containing the StepperMotorModule to be tested
 * and a MotorControlerDummy to mimic motor functionality
 */
struct TestFixture {
  TestFixture(){
    testFacility.runApplication();

    boost::shared_ptr<mtca4u::MotorDriverCard> mdc{mtca4u::MotorDriverCardFactory::instance()
          .createMotorDriverCard(stepperMotorDeviceName, moduleName, stepperMotorDeviceConfigFile)};

    _motorControlerDummy = boost::dynamic_pointer_cast<mtca4u::MotorControlerDummy>(
               mdc->getMotorControler(0));
  }

  // Test server
  struct TestServer : public ctk::Application {

    TestServer()
      : ctk::Application("MotorDriverCard_ApplicationModule_TestServer"){ mtca4u::MotorDriverCardFactory::instance().setDummyMode(); }
    ~TestServer() override { shutdown(); }

    ctk::ControlSystemModule cs{};
    //ctk::PeriodicTrigger trigger{this, "trigger", ""};

    // Motor instance and parameters
    ctk::MotorDriver::StepperMotorParameters parameters
      = {ctk::MotorDriver::StepperMotorType::LINEAR, stepperMotorDeviceName, moduleName, 0U, stepperMotorDeviceConfigFile};
    std::unique_ptr<ctk::MotorDriver::StepperMotorModule> motor;

    void defineConnections() override {

      //ChimeraTK::setDMapFilePath("MD22_on_DAMC2.dmap");

      motor = std::make_unique<ctk::MotorDriver::StepperMotorModule>(this, "Motor", "", parameters);

      cs["Trigger"]("tick") >> motor->readbackHandler("tick");
      motor->findTag("MOTOR|MOT_DIAG").connectTo(cs["Motor"]);

//      cs.dump();
//      dumpConnections();
    }

  } testServer;

  ChimeraTK::TestFacility testFacility;

  boost::shared_ptr<mtca4u::MotorControlerDummy> _motorControlerDummy;

  void applyInputAndTriggerReadback(void){
    testFacility.stepApplication();
    testFacility.writeScalar<uint64_t>("Trigger/tick", 1);
    testFacility.stepApplication();
  }

};

using namespace boost::unit_test_framework;


/**********************************************************************************************************************/

/// A template test case
BOOST_FIXTURE_TEST_CASE(testMotorDriver, TestFixture){
  std::cout << "testMotorDriver" << std::endl;

  auto  trigger = testFacility.getScalar<uint64_t>("Trigger/tick");
  auto motorState  = testFacility.getScalar<std::string>("Motor/readback/status/state");

  // Enable he dummy motor
  _motorControlerDummy->setEnabled(true);
  _motorControlerDummy->setMotorCurrentEnabled(true);

  trigger.write();
  testFacility.stepApplication();
  motorState.read();

  // Application should start with disabled motor
  BOOST_CHECK_EQUAL(static_cast<std::string>(motorState), "disabled");

  // Enable stepper motor and set a reference position
  // (the latter should yield ChimeraTK::MotorDriver::CalibrationMode::SIMPLE)
  testFacility.writeScalar<int>("Motor/controlInput/control/enable", 1);
  testFacility.writeScalar<int>("Motor/controlInput/control/enableAutostart", 1);
  testFacility.writeScalar<int>("Motor/controlInput/referenceSettings/positionInSteps", 0);


  applyInputAndTriggerReadback();
  motorState.read();

  BOOST_CHECK_EQUAL(testFacility.readScalar<int>("Motor/readback/status/autostartEnabled"), 1);
  BOOST_CHECK_EQUAL(static_cast<std::string>(motorState), "idle");
  BOOST_CHECK_EQUAL(testFacility.readScalar<int>("Motor/controlInput/notification/hasMessage"), 0);
  BOOST_CHECK_EQUAL(testFacility.readScalar<int>("Motor/readback/status/calibrationMode"),
                                                 static_cast<int>(ChimeraTK::MotorDriver::CalibrationMode::SIMPLE));

  // Apply a position setpoint
  int targetPosition{1000};
  testFacility.writeScalar<int>("Motor/controlInput/positionSetpoint/positionInSteps", targetPosition);

  applyInputAndTriggerReadback();
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

