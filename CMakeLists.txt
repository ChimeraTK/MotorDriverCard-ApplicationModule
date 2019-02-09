project(ChimeraTK-MotorDriverCard-ApplicationModule)
cmake_minimum_required(VERSION 2.8.0)

set(${PROJECT_NAME}_MAJOR_VERSION 00)
set(${PROJECT_NAME}_MINOR_VERSION 01)
set(${PROJECT_NAME}_PATCH_VERSION 00)
include(cmake/set_version_numbers.cmake)

include(cmake/add_dependency.cmake)
add_dependency(ChimeraTK-ApplicationCore 01.00 REQUIRED)
add_dependency(mtca4u-MotorDriverCard 01.09 REQUIRED)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -fPIC -Wextra -Wshadow -pedantic -Wuninitialized -Weffc++")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0 --coverage")
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ChimeraTK-ApplicationCore_CXX_FLAGS} ${mtca4u-MotorDriverCard_CXX_FLAGS}" )


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
                                       SOVERSION ${${PROJECT_NAME}_SOVERSION})

target_include_directories(${PROJECT_NAME} PUBLIC include)
target_link_libraries(${PROJECT_NAME}
                        ${ChimeraTK-ApplicationCore_LIBRARIES}
                        ${MotorDriverCard_LIBRARIES})

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)

# Create the configuration files
set(${PROJECT_NAME}_INCLUDE_DIRS "${CMAKE_INSTALL_PREFIX}/include")
set(${PROJECT_NAME}_LIBRARIES "")
set(${PROJECT_NAME}_LIBRARY_DIRS "${CMAKE_INSTALL_PREFIX}/lib")
set(${PROJECT_NAME}_LINKER_FLAGS "-Wl,-rpath=${CMAKE_INSTALL_PREFIX}/lib,--enable-new-dtags ${mtca4u-MotorDriverCard_LINKER_FLAGS}")
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/create_cmake_config_files.cmake)
