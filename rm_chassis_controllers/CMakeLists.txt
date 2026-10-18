cmake_minimum_required(VERSION 3.10)
project(rm_chassis_controllers)

option(RM_BUILD_BIPEDAL "Build bipedal wheel controllers with external TinyMPC" OFF)
# Remove the obsolete dependency switch from existing build directories.
unset(RM_BIPEDAL_WITH_TINYMPC CACHE)
message(STATUS "RM_BUILD_BIPEDAL=${RM_BUILD_BIPEDAL}")

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror -Wno-enum-compare)

## Find catkin macros and libraries
find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        rm_msgs

        rm_common

        controller_interface
        effort_controllers
        tf2_geometry_msgs
        nav_msgs
        angles
        dynamic_reconfigure
)

find_package(Eigen3 REQUIRED)
# find_package(Boost REQUIRED COMPONENTS filesystem system)
# find_package(Threads REQUIRED)

set(DYNAMIC_CONFIGS cfg/PowerLimit.cfg)
if(RM_BUILD_BIPEDAL)
  list(APPEND DYNAMIC_CONFIGS cfg/LQRWeight.cfg)
endif()
generate_dynamic_reconfigure_options(${DYNAMIC_CONFIGS})

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        INCLUDE_DIRS
        include
        ${EIGEN3_INCLUDE_DIR}
        LIBRARIES
        CATKIN_DEPENDS
        roscpp
        rm_msgs

        rm_common

        effort_controllers
        tf2_geometry_msgs
        nav_msgs
        angles
        dynamic_reconfigure
        LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
        include
        include/rm_chassis_controllers
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

set(BIPEDAL_SOURCES
        src/bipedal_wheel_controller/estimation/velocity_kf.cpp
        src/bipedal_wheel_controller/dynamics/model10.cpp
        src/bipedal_wheel_controller/dynamics/design10.cpp
        src/bipedal_wheel_controller/mpc/mpc_backend.cpp
        src/bipedal_wheel_controller/controller.cpp
        src/bipedal_wheel_controller/controller_mode/mode_base.cpp
        src/bipedal_wheel_controller/controller_mode/mode_manager.cpp
        src/bipedal_wheel_controller/controller_mode/normal.cpp
        src/bipedal_wheel_controller/controller_mode/recover.cpp
        src/bipedal_wheel_controller/controller_mode/protect.cpp
        src/bipedal_wheel_controller/controller_mode/upstairs.cpp
        src/bipedal_wheel_controller/controller_mode/sit_down.cpp
        src/bipedal_wheel_controller/controller_mode/stand_up.cpp
        src/bipedal_wheel_controller/series_legged_vmc_controller.cpp
        src/bipedal_wheel_controller/vmc/VMC.cpp
        src/bipedal_wheel_controller/vmc/leg_conv.c
        src/bipedal_wheel_controller/vmc/leg_conv_t.c
        src/bipedal_wheel_controller/vmc/leg_pos.c
        src/bipedal_wheel_controller/vmc/leg_spd.c
)

## Declare cpp executables
add_library(${PROJECT_NAME} SHARED
        src/chassis_base.cpp
        src/omni.cpp
        src/sentry.cpp
        src/swerve.cpp
        src/balance.cpp
)

## Specify libraries to link executable targets against
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        # ${Boost_LIBRARIES}
        # Threads::Threads
)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

if(RM_BUILD_BIPEDAL)
  # An enabled bipedal build always requires the externally built SDK.
  find_package(RmTinyMPC 0.2 CONFIG REQUIRED)
  if(NOT RmTinyMPC_UPSTREAM_REVISION STREQUAL "023f36bd5b27267e1a96ed47a774d120b5707f13")
    message(FATAL_ERROR "The MPC backend requires the verified TinyMPC revision 023f36bd5b27267e1a96ed47a774d120b5707f13")
  endif()
  if(NOT "${RmTinyMPC_EIGEN_VERSION}" VERSION_EQUAL "${EIGEN3_VERSION_STRING}")
    message(FATAL_ERROR "TinyMPC and rm_chassis_controllers must use the same Eigen version")
  endif()
  include(CMakePushCheckState)
  include(CheckCXXSourceCompiles)
  cmake_push_check_state(RESET)
  set(CMAKE_REQUIRED_LIBRARIES RmTinyMPC::tinympc)
  unset(RM_TINYMPC_LINK_COMPATIBLE CACHE)
  check_cxx_source_compiles("#include <tinympc/tiny_api.hpp>
    static_assert(EIGEN_WORLD_VERSION == 3 && EIGEN_MAJOR_VERSION == 3 && EIGEN_MINOR_VERSION == 7,
                  \"Expected workspace Eigen 3.3.7\");
    int main() { TinySettings settings{}; return tiny_set_default_settings(&settings); }"
    RM_TINYMPC_LINK_COMPATIBLE)
  cmake_pop_check_state()
  if(NOT RM_TINYMPC_LINK_COMPATIBLE)
    message(FATAL_ERROR "External TinyMPC headers/library failed the C++14 consumer check")
  endif()

  add_library(rm_bipedal_controllers SHARED ${BIPEDAL_SOURCES})
  target_link_libraries(rm_bipedal_controllers PRIVATE ${PROJECT_NAME} ${catkin_LIBRARIES} RmTinyMPC::tinympc)
  add_dependencies(rm_bipedal_controllers ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
endif()

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
        TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(RM_BUILD_BIPEDAL)
  install(TARGETS rm_bipedal_controllers
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif()

# Mark cpp header files for installation
install(
        DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN "bipedal_wheel_controller" EXCLUDE
)
if(RM_BUILD_BIPEDAL)
  install(DIRECTORY include/${PROJECT_NAME}/bipedal_wheel_controller/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/bipedal_wheel_controller
    FILES_MATCHING PATTERN "*.h")
endif()

# Mark other files for installation
install(
        DIRECTORY test
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(
        FILES rm_chassis_controllers_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(chassis_plugin_load_test test/chassis_plugin_load.test test/chassis_plugin_load_test.cpp)
  if(TARGET chassis_plugin_load_test)
    target_compile_definitions(chassis_plugin_load_test PRIVATE RM_TEST_BUILD_BIPEDAL=$<BOOL:${RM_BUILD_BIPEDAL}>)
    target_link_libraries(chassis_plugin_load_test ${catkin_LIBRARIES})
    add_dependencies(chassis_plugin_load_test ${PROJECT_NAME})
    if(RM_BUILD_BIPEDAL)
      add_dependencies(chassis_plugin_load_test rm_bipedal_controllers)
    endif()
  endif()
  if(RM_BUILD_BIPEDAL)
    catkin_add_gtest(bipedal_mpc_backend_test test/bipedal_mpc_backend_test.cpp)
    if(TARGET bipedal_mpc_backend_test)
      target_link_libraries(bipedal_mpc_backend_test rm_bipedal_controllers ${catkin_LIBRARIES} RmTinyMPC::tinympc)
      add_dependencies(bipedal_mpc_backend_test ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
    endif()
    add_rostest_gtest(bipedal_observation_test test/bipedal_observation.test test/bipedal_observation_test.cpp)
    if(TARGET bipedal_observation_test)
      target_link_libraries(bipedal_observation_test rm_bipedal_controllers ${PROJECT_NAME} ${catkin_LIBRARIES})
      add_dependencies(bipedal_observation_test ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
    endif()
  endif()
endif()
