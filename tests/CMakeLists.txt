# CMakeLists.txt for test modules
CMAKE_MINIMUM_REQUIRED(VERSION 3.5.1)


# Add BOOST dependencies
FIND_PACKAGE(Boost COMPONENTS thread system unit_test_framework REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/include)

# Add all tests residing in the "tests" directory
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} testExecutables)
foreach( testExecutableSrcFile ${testExecutables})
  #NAME_WE means the base name without path and (longest) extension
  get_filename_component(excutableName ${testExecutableSrcFile} NAME_WE)
  add_executable(${excutableName} ${testExecutableSrcFile})
  set_target_properties(${excutableName} 
    PROPERTIES 
    INCLUDE_DIRECTORIES "${mtca4u-MotorDriverCard_INCLUDE_DIRS}"
    LINK_FLAGS "${CMAKE_LINK_FLAGS}  ${ChimeraTK-MotorDriverCard-ApplicationModule_LINK_FLAGS} ${ChimeraTK-ApplicationCore_LINK_FLAGS}"
  )
  target_link_libraries(${excutableName} ${PROJECT_NAME}
                                           ${ChimeraTK-ApplicationCore_LIBRARIES}
                                           ${ChimeraTK-MotorDriverCard-ApplicationModule_LIBRARIES} )
  add_test(
    NAME ${excutableName}
    COMMAND ${CMAKE_CURRENT_BINARY_DIR}/${excutableName}
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  )
endforeach( testExecutableSrcFile )

FILE(COPY MD22_on_DAMC2.dmap
          DFMC_MD22_test.map
          VT21-MotorDriverCardConfig.xml
     DESTINATION ${PROJECT_BINARY_DIR})
