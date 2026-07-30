# Host-side digipeater test build.
# Only depends on the C++ standard library and test harness header.
cmake_minimum_required(VERSION 3.16)
project(test_digipeater LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Smoke tests
add_executable(test_digipeater_smoke
    test_digipeater_smoke.cpp
)

target_include_directories(test_digipeater_smoke PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/..)

# libaprsroute routes.json n-N routing conformance tests
add_executable(test_digipeater_nn
    test_digipeater_nn.cpp
)

target_include_directories(test_digipeater_nn PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/..)

# KISS capture digipeater command-line tool
add_executable(tnc_digipeat
    tnc_digipeat.cpp
)

target_include_directories(tnc_digipeat PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/..)
set_property(TARGET tnc_digipeat PROPERTY CXX_STANDARD 17)

# libaprsroute routes.json explicit-routing conformance tests
# (preempt_front, substitute_explicit, traceless, reject/trap limits,
# route_self, strict, and combined-option cases).
add_executable(test_digipeater_explicit
    test_digipeater_explicit.cpp
)

target_include_directories(test_digipeater_explicit PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/..)

enable_testing()
add_test(NAME digipeater_smoke COMMAND test_digipeater_smoke)
add_test(NAME digipeater_nn COMMAND test_digipeater_nn)
add_test(NAME digipeater_explicit COMMAND test_digipeater_explicit)
