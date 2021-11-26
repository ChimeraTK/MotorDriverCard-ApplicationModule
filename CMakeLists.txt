project(ChimeraTK-MotorDriverCard-ApplicationModule)
cmake_minimum_required(VERSION 2.8.0)

set(${PROJECT_NAME}_MAJOR_VERSION 00)
set(${PROJECT_NAME}_MINOR_VERSION 04)
set(${PROJECT_NAME}_PATCH_VERSION 00)
include(cmake/set_version_numbers.cmake)

include(cmake/add_dependency.cmake)
add_dependency(ChimeraTK-ApplicationCore 01.00 REQUIRED)
add_dependency(mtca4u-MotorDriverCard 01.12 REQUIRED)

INCLUDE(${CMAKE_SOURCE_DIR}/cmake/set_default_build_to_release.cmake)
INCLUDE(${CMAKE_SOURCE_DIR}/cmake/set_default_flags.cmake)


SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# Install the header files, put them in the same directory as
# the library as they are closely related
install(DIRECTORY include/ DESTINATION include/ChimeraTK/MotorDriverCard)


# The shared library
set(SRC StepperMotorModule.cc StepperMotorCtrl.cc StepperMotorReadback.cc)
foreach(SOURCE ${SRC})
    set(LIBRARY_SOURCES ${LIBRARY_SOURCES} ${CMAKE_CURRENT_SOURCE_DIR}/src/${SOURCE})
endforeach()

add_library(${PROJECT_NAME} SHARED ${LIBRARY_SOURCES})
set_target_properties(${PROJECT_NAME} PROPERTIES 
                                       VERSION ${${PROJECT_NAME}_FULL_LIBRARY_VERSION}
                                       SOVERSION ${${PROJECT_NAME}_SOVERSION}
                                       LINK_FLAGS "${ChimeraTK-ApplicationCore_LINK_FLAGS} ${mtca4u-MotorDriverCard_LINK_FLAGS}")

target_include_directories(${PROJECT_NAME} PUBLIC include)
target_link_libraries(${PROJECT_NAME}
                        ${ChimeraTK-ApplicationCore_LIBRARIES}
                        ${mtca4u-MotorDriverCard_LIBRARIES})

# Process tests:
ENABLE_TESTING()
add_subdirectory(${CMAKE_SOURCE_DIR}/tests)


install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)

# Create the configuration files
set(${PROJECT_NAME}_INCLUDE_DIRS "${CMAKE_INSTALL_PREFIX}/include ${mtca4u-MotorDriverCard_INCLUDE_DIRS}")
set(${PROJECT_NAME}_LIBRARIES "${mtca4u-MotorDriverCard_LIBRARIES}")
set(${PROJECT_NAME}_LIBRARY_DIRS "${CMAKE_INSTALL_PREFIX}/lib")
set(${PROJECT_NAME}_LINKER_FLAGS "-Wl,-rpath=${CMAKE_INSTALL_PREFIX}/lib,--enable-new-dtags ${mtca4u-MotorDriverCard_LINKER_FLAGS}")
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/create_cmake_config_files.cmake)
