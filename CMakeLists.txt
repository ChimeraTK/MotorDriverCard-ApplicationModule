project(ChimeraTK-MotorDriverCard-ApplicationModule)
cmake_minimum_required(VERSION 3.16)

include(GNUInstallDirs)

set(${PROJECT_NAME}_MAJOR_VERSION 00)
set(${PROJECT_NAME}_MINOR_VERSION 08)
set(${PROJECT_NAME}_PATCH_VERSION 07)
include(cmake/set_version_numbers.cmake)

find_package(ChimeraTK-ApplicationCore 01.00 REQUIRED)
find_package(mtca4u-MotorDriverCard 01.12 REQUIRED)

include(cmake/set_default_build_to_release.cmake)
include(cmake/set_default_flags.cmake)
include(cmake/enable_code_style_check.cmake)
include(cmake/enable_doxygen_documentation.cmake)


# The shared library
set(SRC StepperMotorModule.cc StepperMotorCtrl.cc StepperMotorReadback.cc StatusModule.cc)
foreach(SOURCE ${SRC})
    set(LIBRARY_SOURCES ${LIBRARY_SOURCES} ${CMAKE_CURRENT_SOURCE_DIR}/src/${SOURCE})
endforeach()
file(GLOB LIBRARY_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h)
set(LIBRARY_SOURCES ${LIBRARY_SOURCES} ${LIBRARY_HEADERS})

add_library(${PROJECT_NAME} SHARED ${LIBRARY_SOURCES})
set_target_properties(${PROJECT_NAME} PROPERTIES 
                                       VERSION ${${PROJECT_NAME}_FULL_LIBRARY_VERSION}
                                       SOVERSION ${${PROJECT_NAME}_SOVERSION})

target_include_directories(${PROJECT_NAME} PRIVATE include
                                           PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                                                  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(${PROJECT_NAME} PUBLIC ChimeraTK::ChimeraTK-ApplicationCore
                                      PRIVATE ChimeraTK::mtca4u-MotorDriverCard)

## Tests

# Process tests:
ENABLE_TESTING()
add_subdirectory(${CMAKE_SOURCE_DIR}/tests)

## Installation

# Install the header files, put them in the same directory as
# the library as they are closely related
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ChimeraTK/MotorDriverCard)

install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

# Create the configuration files
set(PROVIDES_EXPORTED_TARGETS 1)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/create_cmake_config_files.cmake)
