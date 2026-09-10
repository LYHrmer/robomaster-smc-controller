cmake_minimum_required(VERSION 3.16)
project(robomaster_smc_controller VERSION 0.1.0 LANGUAGES C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

option(GIMBAL_BUILD_TESTS "Build host regression tests" ON)
option(GIMBAL_BUILD_EXAMPLES "Build platform-independent examples" ON)
option(GIMBAL_BUILD_SIM "Build synthetic host simulation" ON)
option(GIMBAL_SANITIZE "Enable host address and undefined behavior sanitizers" OFF)
option(GIMBAL_BUILD_IDENTIFICATION "Enable optional host Python/NumPy identification tests" OFF)

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
add_library(gimbal_rls STATIC src/gimbal_rls.c)
target_include_directories(gimbal_rls PUBLIC include)
target_link_libraries(gimbal_rls PUBLIC m)
add_library(gimbal_identification STATIC src/gimbal_identification.c)
target_include_directories(gimbal_identification PUBLIC include)
target_link_libraries(gimbal_identification PUBLIC gimbal_rls m)

if(GIMBAL_BUILD_TESTS)
  enable_testing()
  add_executable(test_smc tests/test_smc.c)
  target_link_libraries(test_smc PRIVATE gimbal_smc)
  add_test(NAME smc COMMAND test_smc)
  add_executable(test_motor tests/test_motor.c)
  target_link_libraries(test_motor PRIVATE gimbal_motor)
  add_test(NAME motor COMMAND test_motor)
  add_executable(test_rls tests/test_rls.c)
  target_link_libraries(test_rls PRIVATE gimbal_rls)
  add_test(NAME rls COMMAND test_rls)
  add_executable(test_rls_endurance tests/test_rls_endurance.c)
  target_link_libraries(test_rls_endurance PRIVATE gimbal_rls)
  add_test(NAME rls_endurance COMMAND test_rls_endurance)
  add_executable(test_identification_online tests/test_identification_online.c)
  target_link_libraries(test_identification_online PRIVATE gimbal_identification)
  add_test(NAME identification_online COMMAND test_identification_online)
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
  add_executable(validate_online_identification sim/validate_online_identification.c)
  target_link_libraries(validate_online_identification PRIVATE gimbal_smc gimbal_motor gimbal_identification)
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
    add_test(NAME online_identification_closed_loop COMMAND validate_online_identification
      ${CMAKE_CURRENT_BINARY_DIR}/online_identification_metrics.csv)
    find_package(Python3 COMPONENTS Interpreter QUIET)
    if(Python3_Interpreter_FOUND)
      add_test(NAME open_model_provenance COMMAND ${Python3_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/tools/derive_open_model_profiles.py --check)
    endif()
  endif()
endif()

# Offline identification remains optional; the MCU libraries do not use Python.
if(GIMBAL_BUILD_IDENTIFICATION)
  if(CMAKE_CROSSCOMPILING OR NOT GIMBAL_BUILD_TESTS)
    message(FATAL_ERROR "Identification checks require a native host build with GIMBAL_BUILD_TESTS=ON")
  endif()
  find_package(Python3 3.9 COMPONENTS Interpreter REQUIRED)
  execute_process(COMMAND ${Python3_EXECUTABLE} -c "import numpy"
    RESULT_VARIABLE GIMBAL_NUMPY_STATUS OUTPUT_QUIET ERROR_QUIET)
  if(NOT GIMBAL_NUMPY_STATUS EQUAL 0)
    message(FATAL_ERROR "Install requirements-identification.txt into the selected Python3 interpreter")
  endif()
  add_test(NAME identification_checks COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_identification.py)
  add_test(NAME identified_model_closed_loop COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/tools/validate_identification.py
    --output-dir ${CMAKE_CURRENT_BINARY_DIR}/identification --cc ${CMAKE_C_COMPILER})
endif()

# Assertions are the test oracle, including in Release builds.
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
  foreach(test_target test_smc test_motor test_example test_pitch test_rls test_rls_endurance test_identification_online)
    if(TARGET ${test_target})
      target_compile_options(${test_target} PRIVATE -UNDEBUG)
    endif()
  endforeach()
endif()
