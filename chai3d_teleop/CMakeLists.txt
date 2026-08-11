cmake_minimum_required(VERSION 3.16)

project(chai3d_flexiv_device_bridge LANGUAGES C CXX)

set(CHAI3D_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../chai3d" CACHE PATH
    "Path to the manips-sai-org/chai3d checkout")
get_filename_component(CHAI3D_ROOT "${CHAI3D_ROOT}" ABSOLUTE)

if(NOT EXISTS "${CHAI3D_ROOT}/CMakeLists.txt")
  message(FATAL_ERROR
    "CHAI3D was not found at ${CHAI3D_ROOT}. Clone it first or pass "
    "-DCHAI3D_ROOT=/absolute/path/to/chai3d")
endif()

# Build the CHAI3D static library without its graphics examples.
set(BUILD_EXAMPLES OFF CACHE BOOL "Build CHAI3D examples" FORCE)
set(CMAKE_EXPORT_PACKAGE_REGISTRY OFF)
set(CMAKE_EXPORT_NO_PACKAGE_REGISTRY ON CACHE BOOL
    "Do not write CHAI3D to the user package registry" FORCE)
add_subdirectory("${CHAI3D_ROOT}" "${CMAKE_BINARY_DIR}/chai3d" EXCLUDE_FROM_ALL)

add_executable(chai3d_device_stream cpp/chai3d_device_stream.cpp)
target_compile_features(chai3d_device_stream PRIVATE cxx_std_17)
target_compile_options(chai3d_device_stream PRIVATE -Wall -Wextra -Wpedantic)
target_compile_definitions(chai3d_device_stream PRIVATE LINUX)
target_include_directories(chai3d_device_stream SYSTEM PRIVATE
  "${CHAI3D_ROOT}/src"
  "${CHAI3D_ROOT}/external/glew/include"
  "${CHAI3D_ROOT}/external/DHD/include")
target_link_directories(chai3d_device_stream PRIVATE
  "${CHAI3D_ROOT}/external/DHD/lib/lin-x86_64")
target_link_libraries(chai3d_device_stream PRIVATE
  chai3d
  ${OPENGL_LIBRARIES}
  ${OPENAL_LIBRARY}
  drd
  usb-1.0
  rt
  pthread
  dl)

# Flexiv's Python RDK intentionally exposes only NRT control.  The true 1 kHz
# torque/OSC demos therefore use the official C++ RDK and are opt-in so a
# normal CHAI3D build remains possible before RDK dependencies are installed.
option(BUILD_FLEXIV_RT_OSC "Build 1 kHz Flexiv torque OSC demos" OFF)
if(BUILD_FLEXIV_RT_OSC)
  find_package(flexiv_rdk 1.9 REQUIRED)
  get_filename_component(FLEXIV_RDK_PREFIX
    "${flexiv_rdk_DIR}/../../.." ABSOLUTE)
  foreach(target IN ITEMS flexiv_7dof_torque_osc flexiv_9dof_torque_osc)
    add_executable(${target} "cpp/${target}.cpp")
    target_compile_features(${target} PRIVATE cxx_std_17)
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
    target_include_directories(${target} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/cpp")
    target_link_libraries(${target} PRIVATE flexiv::flexiv_rdk)
    # The official RDK archive is static but several bundled dependencies are
    # shared libraries. Embed this workspace-local dependency directory so the
    # demos also start from the UI/VS Code without manual LD_LIBRARY_PATH.
    set_target_properties(${target} PROPERTIES
      BUILD_RPATH "${FLEXIV_RDK_PREFIX}/lib"
      INSTALL_RPATH "${FLEXIV_RDK_PREFIX}/lib")
    # DT_RPATH is intentionally used here because libfastrtps has a bundled
    # indirect dependency (foonathan_memory) in the same non-system prefix;
    # DT_RUNPATH is not searched for indirect dependencies by glibc.
    target_link_options(${target} PRIVATE "LINKER:--disable-new-dtags")
  endforeach()
endif()
