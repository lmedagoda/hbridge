
PROJECT(hbridge-test)

cmake_minimum_required(VERSION 2.6)

find_package(PkgConfig)

pkg_check_modules(HBRIDGE REQUIRED "hbridge")
include_directories(${HBRIDGE_INCLUDE_DIRS})
link_directories(${HBRIDGE_LIBRARY_DIRS})

#add definitions, compiler switches, etc.
ADD_DEFINITIONS(-Wall -O2 -g)

ADD_EXECUTABLE(hbridge-test test.cpp)
TARGET_LINK_LIBRARIES(hbridge-test hbridge boost_unit_test_framework canbus)

INSTALL(TARGETS hbridge-test
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib)
