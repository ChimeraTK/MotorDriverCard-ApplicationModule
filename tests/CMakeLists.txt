# CMakeLists.txt for test modules
CMAKE_MINIMUM_REQUIRED(VERSION 3.16)


# Add BOOST dependencies
FIND_PACKAGE(Boost REQUIRED COMPONENTS thread system unit_test_framework)

# Add all tests residing in the "tests" directory
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} testExecutables)
foreach( testExecutableSrcFile ${testExecutables})
  #NAME_WE means the base name without path and (longest) extension
  get_filename_component(executableName ${testExecutableSrcFile} NAME_WE)
  add_executable(${executableName} ${testExecutableSrcFile})

  # Need to explicitly link to MotorDriverCard here because of the dummy motor
  target_link_libraries(${executableName} ${PROJECT_NAME}
                                         ChimeraTK::ChimeraTK-ApplicationCore
                                         ChimeraTK::ChimeraTK-MotorDriverCard-ApplicationModule
                                         ChimeraTK::mtca4u-MotorDriverCard)
  add_test(
    NAME ${executableName}
    COMMAND ${CMAKE_CURRENT_BINARY_DIR}/${executableName}
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  )
endforeach( testExecutableSrcFile )

FILE(COPY MD22_on_DAMC2.dmap
          DFMC_MD22_test.map
          VT21-MotorDriverCardConfig.xml
          motorProxy.xlmap
     DESTINATION ${PROJECT_BINARY_DIR})
