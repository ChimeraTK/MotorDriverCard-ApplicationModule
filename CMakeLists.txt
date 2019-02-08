project(ChimeraTK-StepperMotorModule)
cmake_minimum_required(VERSION 2.8.0)

set(${PROJECT_NAME}_MAJOR_VERSION 00)
set(${PROJECT_NAME}_MINOR_VERSION 00)
set(${PROJECT_NAME}_PATCH_VERSION 00)
include(cmake/set_version_numbers.cmake)

include(cmake/add_dependency.cmake)
add_dependency(ChimeraTK-ApplicationCore 01.00 REQUIRED)
add_dependency(mtca4u-MotorDriverCard 01.09 REQUIRED)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -fPIC -Wextra -Wshadow -pedantic -Wuninitialized -Weffc++")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0 --coverage")
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")


#set(LIBRARY_HEADER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
install(DIRECTORY include/ DESTINATION include/ChimeraTK)


set(SRC StepperMotorModule.cc ControlInput.cc Readback.cc)
foreach(SOURCE ${SRC})
    set(LIBRARY_SOURCES ${LIBRARY_SOURCES} ${CMAKE_CURRENT_SOURCE_DIR}/src/${SOURCE})
endforeach()

add_library(ChimeraTK-StepperMotorModule SHARED ${LIBRARY_SOURCES})
target_include_directories(ChimeraTK-StepperMotorModule PUBLIC include)
target_link_libraries(ChimeraTK-StepperMotorModule
                        ${ChimeraTK-ApplicationCore_LIBRARIES}
                        ${MotorDriverCard_LIBRARIES})

install(TARGETS ChimeraTK-StepperMotorModule LIBRARY DESTINATION lib)


set(${PROJECT_NAME}_INCLUDE_DIRS "${CMAKE_INSTALL_PREFIX}/include")
set(${PROJECT_NAME}_LIBRARIES "")
set(${PROJECT_NAME}_LIBRARY_DIRS "${CMAKE_INSTALL_PREFIX}/lib")
set(${PROJECT_NAME}_LINKER_FLAGS "-Wl,-rpath=${CMAKE_INSTALL_PREFIX}/lib,--enable-new-dtags ${mtca4u-MotorDriverCard_LINKER_FLAGS}")
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/create_cmake_config_files.cmake)
