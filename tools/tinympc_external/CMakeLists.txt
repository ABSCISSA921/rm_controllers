cmake_minimum_required(VERSION 3.15)
project(RmTinyMPC VERSION 0.2.0 LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(TINYMPC_SOURCE_DIR "" CACHE PATH "External TinyMPC checkout")
option(RM_TINYMPC_EIGEN33_COMPAT "Apply two Eigen 3.3 compatibility substitutions to a build copy" ON)
set(RM_TINYMPC_REVISION "023f36bd5b27267e1a96ed47a774d120b5707f13")

if(NOT EXISTS "${TINYMPC_SOURCE_DIR}/src/tinympc/tiny_api.cpp")
  message(FATAL_ERROR "Set TINYMPC_SOURCE_DIR to an external TinyMPC checkout")
endif()
if(CMAKE_BINARY_DIR STREQUAL CMAKE_SOURCE_DIR)
  message(FATAL_ERROR "Use an external, out-of-source build directory")
endif()
find_package(Git REQUIRED)
execute_process(COMMAND "${GIT_EXECUTABLE}" rev-parse HEAD
  WORKING_DIRECTORY "${TINYMPC_SOURCE_DIR}"
  OUTPUT_VARIABLE actual_revision OUTPUT_STRIP_TRAILING_WHITESPACE
  RESULT_VARIABLE revision_status)
if(NOT revision_status EQUAL 0 OR NOT actual_revision STREQUAL RM_TINYMPC_REVISION)
  message(FATAL_ERROR "TinyMPC must be checked out at ${RM_TINYMPC_REVISION}")
endif()
execute_process(COMMAND "${GIT_EXECUTABLE}" status --porcelain --untracked-files=no
  WORKING_DIRECTORY "${TINYMPC_SOURCE_DIR}"
  OUTPUT_VARIABLE tracked_changes OUTPUT_STRIP_TRAILING_WHITESPACE
  RESULT_VARIABLE status_result)
if(NOT status_result EQUAL 0 OR NOT tracked_changes STREQUAL "")
  message(FATAL_ERROR "Use an unmodified TinyMPC checkout; compatibility changes are made only in the build copy")
endif()

find_package(Eigen3 3.3.7 EXACT REQUIRED CONFIG)
set(upstream_copy "${CMAKE_CURRENT_BINARY_DIR}/upstream/tinympc")
file(COPY "${TINYMPC_SOURCE_DIR}/src/tinympc" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/upstream")
function(replace_required relative_path old_text new_text)
  set(path "${upstream_copy}/${relative_path}")
  file(READ "${path}" contents)
  string(FIND "${contents}" "${old_text}" match_position)
  if(match_position EQUAL -1)
    message(FATAL_ERROR "Expected upstream expression missing in ${relative_path}")
  endif()
  string(REPLACE "${old_text}" "${new_text}" contents "${contents}")
  file(WRITE "${path}" "${contents}")
endfunction()
if(RM_TINYMPC_EIGEN33_COMPAT)
  replace_required(admm.cpp "s(Eigen::placeholders::last)" "s(s.size() - 1)")
  replace_required(codegen.cpp "mat.reshaped<RowMajor>()[i]" "mat(i / mat.cols(), i % mat.cols())")
endif()

# Avoid per-solve console output in the controller loop.
replace_required(admm.cpp
  [=[            std::cout << "Solver converged in " << solver->work->iter << " iterations" << std::endl;
]=] "")

file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/include")
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/include/Eigen.h"
  "#pragma once\n#include <Eigen/Core>\n#include <Eigen/LU>\n")

add_library(tinympc STATIC
  "${upstream_copy}/admm.cpp"
  "${upstream_copy}/tiny_api.cpp"
  "${upstream_copy}/codegen.cpp"
  "${upstream_copy}/rho_benchmark.cpp")
add_library(RmTinyMPC::tinympc ALIAS tinympc)
set_target_properties(tinympc PROPERTIES
  CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES
  POSITION_INDEPENDENT_CODE YES OUTPUT_NAME rm_tinympc)
target_include_directories(tinympc PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/upstream>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/rm_tinympc>")
target_link_libraries(tinympc PUBLIC Eigen3::Eigen)

enable_testing()
add_executable(tinympc_eigen_smoke smoke.cpp)
set_target_properties(tinympc_eigen_smoke PROPERTIES CXX_STANDARD 14 CXX_STANDARD_REQUIRED YES)
target_link_libraries(tinympc_eigen_smoke PRIVATE tinympc)
add_test(NAME tinympc_eigen_smoke COMMAND tinympc_eigen_smoke)

configure_package_config_file(RmTinyMPCConfig.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/RmTinyMPCConfig.cmake"
  INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/RmTinyMPC")
write_basic_package_version_file("${CMAKE_CURRENT_BINARY_DIR}/RmTinyMPCConfigVersion.cmake"
  VERSION "${PROJECT_VERSION}" COMPATIBILITY SameMajorVersion)
install(TARGETS tinympc EXPORT RmTinyMPCTargets ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}")
install(DIRECTORY "${upstream_copy}/" DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/rm_tinympc/tinympc"
  FILES_MATCHING PATTERN "*.hpp")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/include/Eigen.h"
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/rm_tinympc")
install(FILES "${TINYMPC_SOURCE_DIR}/LICENSE" DESTINATION "${CMAKE_INSTALL_DATADIR}/RmTinyMPC")
install(EXPORT RmTinyMPCTargets NAMESPACE RmTinyMPC::
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/RmTinyMPC")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/RmTinyMPCConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/RmTinyMPCConfigVersion.cmake"
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/RmTinyMPC")
