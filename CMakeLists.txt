project(Project)
cmake_minimum_required(VERSION 3.22)

set(TARGET_OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/target/thumbv7em-none-eabihf/debug/robottonzo)
add_custom_target(robottonzo DEPENDS ${TARGET_OUTPUT})