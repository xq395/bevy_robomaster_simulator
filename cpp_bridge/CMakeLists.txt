add_library(talos_async_source STATIC
    talos_ipc.cpp
    talos_async_source.cpp
)
target_include_directories(talos_async_source PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_features(talos_async_source PUBLIC cxx_std_17)
target_link_libraries(talos_async_source PUBLIC opencv_core opencv_imgproc)

add_executable(talos_sp_bridge
    talos_sp_bridge.cpp
    web_debug_sink.cpp
)
target_include_directories(talos_sp_bridge PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}
)
target_compile_features(talos_sp_bridge PRIVATE cxx_std_17)
target_link_libraries(talos_sp_bridge PRIVATE
    talos_async_source
    detection
    sp_core
    fire_gate
    mpc_planner
    sp_standard_config
    telemetry
    debug_web
    Threads::Threads
)

add_executable(talos_async_source_test talos_async_source_test.cpp)
target_compile_features(talos_async_source_test PRIVATE cxx_std_17)
target_link_libraries(talos_async_source_test PRIVATE talos_async_source)
add_test(NAME talos_async_source COMMAND talos_async_source_test)

add_executable(aim_command_guard_test aim_command_guard_test.cpp)
target_compile_features(aim_command_guard_test PRIVATE cxx_std_17)
add_test(NAME aim_command_guard COMMAND aim_command_guard_test)

add_executable(single_shot_fire_latch_test single_shot_fire_latch_test.cpp)
target_compile_features(single_shot_fire_latch_test PRIVATE cxx_std_17)
add_test(NAME single_shot_fire_latch COMMAND single_shot_fire_latch_test)

add_executable(target_lock_test target_lock_test.cpp)
target_compile_features(target_lock_test PRIVATE cxx_std_17)
add_test(NAME target_lock COMMAND target_lock_test)
