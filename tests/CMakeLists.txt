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

# KISS capture digipeater command-line tool
add_executable(tnc_digipeat
    tnc_digipeat.cpp
)

target_include_directories(tnc_digipeat PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/..)
set_property(TARGET tnc_digipeat PROPERTY CXX_STANDARD 17)

# libaprsroute routes.json explicit-routing conformance tests
# (hand-written edge cases for can_repeat and SSID-aware mycall handling).
add_executable(test_digipeater_explicit
    test_digipeater_explicit.cpp
)

target_include_directories(test_digipeater_explicit PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/..)

# Full libaprsroute routes.json conformance runner: loads all 256 vectors from
# test_routes_data.hpp, maps each vector's options to our routing_mode flags,
# runs through DigipeaterCore::can_repeat + rewrite_frame, and compares the
# output at the byte level against the reference routed_packet.
add_executable(test_digipeater_conformance
    test_digipeater_conformance.cpp
)

target_include_directories(test_digipeater_conformance PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/..)

enable_testing()
add_test(NAME digipeater_smoke COMMAND test_digipeater_smoke)
add_test(NAME digipeater_explicit COMMAND test_digipeater_explicit)
add_test(NAME digipeater_conformance COMMAND test_digipeater_conformance)
