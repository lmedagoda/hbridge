
pkg_check_modules(CANBUS REQUIRED "canbus")
INCLUDE_DIRECTORIES(${CANBUS_INCLUDE_DIRS})
LINK_DIRECTORIES(${CANBUS_LIBRARY_DIRS})

ADD_EXECUTABLE(hbridge-test test.cpp)
TARGET_LINK_LIBRARIES(hbridge-test hbridge boost_unit_test_framework ${CANBUS_LIBRARIES})

