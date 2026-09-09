cmake_minimum_required(VERSION 3.16)
project(robomaster_smc_controller VERSION 0.1.0 LANGUAGES C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

option(GIMBAL_BUILD_TESTS "Build host regression tests" ON)
option(GIMBAL_BUILD_EXAMPLES "Build platform-independent examples" ON)
option(GIMBAL_BUILD_SIM "Build synthetic host simulation" ON)
option(GIMBAL_SANITIZE "Enable host address and undefined behavior sanitizers" OFF)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
  if(GIMBAL_SANITIZE)
    add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address,undefined)
  endif()
endif()

add_library(gimbal_smc STATIC src/gimbal_smc.c)
target_include_directories(gimbal_smc PUBLIC include)
target_link_libraries(gimbal_smc PUBLIC m)
add_library(gimbal_motor STATIC src/gimbal_motor.c)
target_include_directories(gimbal_motor PUBLIC include)
target_link_libraries(gimbal_motor PUBLIC m)
add_library(gimbal_pitch STATIC src/gimbal_pitch.c)
target_include_directories(gimbal_pitch PUBLIC include)
target_link_libraries(gimbal_pitch PUBLIC m)

if(GIMBAL_BUILD_TESTS)
  enable_testing()
  add_executable(test_smc tests/test_smc.c)
  target_link_libraries(test_smc PRIVATE gimbal_smc)
  add_test(NAME smc COMMAND test_smc)
  add_executable(test_motor tests/test_motor.c)
  target_link_libraries(test_motor PRIVATE gimbal_motor)
  add_test(NAME motor COMMAND test_motor)
endif()

if(GIMBAL_BUILD_EXAMPLES)
  add_library(gimbal_example STATIC examples/gimbal_controller_example.c
    examples/gimbal_pitch_example.c)
  target_include_directories(gimbal_example PUBLIC examples)
  target_link_libraries(gimbal_example PUBLIC gimbal_smc gimbal_motor gimbal_pitch)
  if(GIMBAL_BUILD_TESTS)
    add_executable(test_example tests/test_example.c)
    target_link_libraries(test_example PRIVATE gimbal_example)
    add_test(NAME example COMMAND test_example)
    add_executable(test_pitch tests/test_pitch.c)
    target_link_libraries(test_pitch PRIVATE gimbal_example)
    add_test(NAME pitch COMMAND test_pitch)
  endif()
endif()

if(GIMBAL_BUILD_SIM)
  add_executable(simulate sim/simulate.c)
  target_link_libraries(simulate PRIVATE gimbal_smc)
  add_executable(validate_open_models sim/validate_open_models.c)
  target_link_libraries(validate_open_models PRIVATE gimbal_smc gimbal_motor)
  add_executable(validate_pitch sim/validate_pitch.c)
  target_link_libraries(validate_pitch PRIVATE gimbal_smc gimbal_motor gimbal_pitch)
  if(GIMBAL_BUILD_TESTS)
    add_test(NAME simulation COMMAND simulate ${CMAKE_CURRENT_BINARY_DIR}/simulation.csv)
    add_test(NAME open_model_validation COMMAND validate_open_models
      ${CMAKE_CURRENT_SOURCE_DIR}/sim/open_models/profiles.csv
      ${CMAKE_CURRENT_BINARY_DIR}/open_model_metrics.csv
      ${CMAKE_CURRENT_BINARY_DIR}/open_model_trace.csv)
    add_test(NAME pitch_validation COMMAND validate_pitch
      ${CMAKE_CURRENT_SOURCE_DIR}/sim/open_models/profiles.csv
      ${CMAKE_CURRENT_BINARY_DIR}/pitch_metrics.csv
      ${CMAKE_CURRENT_BINARY_DIR}/pitch_trace.csv)
    add_test(NAME pitch_validator_negative COMMAND validate_pitch --self-test
      ${CMAKE_CURRENT_SOURCE_DIR}/sim/open_models/profiles.csv)
    find_package(Python3 COMPONENTS Interpreter QUIET)
    if(Python3_Interpreter_FOUND)
      add_test(NAME open_model_provenance COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tools/derive_open_model_profiles.py --check)
    endif()
  endif()
endif()

# Assertions are the test oracle, including in Release builds.
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
  foreach(test_target test_smc test_motor test_example test_pitch)
    if(TARGET ${test_target})
      target_compile_options(${test_target} PRIVATE -UNDEBUG)
    endif()
  endforeach()
endif()
