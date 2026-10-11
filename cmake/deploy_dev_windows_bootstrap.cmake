# Compatibility bootstrap injected immediately after deploy_dev's project().
#
# The official OpenCV Windows package ships module targets backed by
# opencv_world. deploy_dev requests only a subset during its top-level
# find_package() call, but later links opencv_highgui explicitly. Resolve all
# modules used by the project up front so those imported targets exist.
if(NOT WIN32)
    return()
endif()

get_filename_component(_deploy_dev_bridge_root "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
set(_deploy_dev_compat_dir "${_deploy_dev_bridge_root}/cpp_compat")
include_directories(BEFORE "${_deploy_dev_compat_dir}")

find_package(OpenCV CONFIG REQUIRED COMPONENTS
    core
    imgproc
    imgcodecs
    calib3d
    videoio
    highgui
)

# yaml-cpp 0.8 exports yaml-cpp::yaml-cpp. deploy_dev links the historical
# un-namespaced target, so expose a directory-scoped compatibility alias.
find_package(yaml-cpp CONFIG REQUIRED)
if(TARGET yaml-cpp::yaml-cpp AND NOT TARGET yaml-cpp)
    add_library(yaml-cpp ALIAS yaml-cpp::yaml-cpp)
endif()
find_package(Eigen3 CONFIG REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenVINO CONFIG REQUIRED)

# communication/serial_port.cpp includes iostream only in its Linux branch,
# although the Windows stubs also log through std::cerr.
add_compile_options(
    $<$<COMPILE_LANGUAGE:CXX>:/EHsc>
    $<$<COMPILE_LANGUAGE:CXX>:/FIiostream>
    $<$<COMPILE_LANGUAGE:CXX>:/FIdeploy_dev_windows_compat.hpp>
)

link_libraries(ws2_32)

# Declare the bridge before deploy_dev's own subdirectories. CMake resolves
# its plain target dependencies (detection, sp_core, etc.) after those targets
# are declared later in the same configure pass.
enable_testing()
add_subdirectory(
    "${_deploy_dev_bridge_root}/cpp_bridge"
    "${CMAKE_BINARY_DIR}/talos_bridge"
)

# Replace the POSIX-oriented runtime tuning translation unit after deploy_dev
# has declared all targets. Its original atomic rename keeps the source file
# open and writes to /tmp, both of which are incompatible with Windows.
function(_deploy_dev_apply_windows_sources)
    if(TARGET sp_standard_config)
        set_property(TARGET sp_standard_config PROPERTY SOURCES
            "${CMAKE_SOURCE_DIR}/pipeline/sp_standard_config.cpp"
            "${_deploy_dev_compat_dir}/runtime_tuning_windows.cpp"
        )
    endif()
    if(TARGET debug_web)
        set_property(TARGET debug_web PROPERTY SOURCES
            "${_deploy_dev_compat_dir}/debug_web_publisher_windows.cpp"
        )
    endif()
endfunction()
cmake_language(DEFER CALL _deploy_dev_apply_windows_sources)