pkg_check_modules(CANBUS REQUIRED "canbus")
INCLUDE_DIRECTORIES(${CANBUS_INCLUDE_DIRS})
LINK_DIRECTORIES(${CANBUS_LIBRARY_DIRS})

find_package( Boost COMPONENTS unit_test_framework thread)
if (Boost_FOUND)
    message(STATUS "boost/unit found, compiling unit test")
#     ADD_EXECUTABLE(hbridge-test test.cpp)
#     include_directories(${Boost_INCLUDE_DIRS})
#     TARGET_LINK_LIBRARIES(hbridge-test hbridge ${Boost_LIBRARIES} ${CANBUS_LIBRARIES})

#     ADD_EXECUTABLE(pwm_test pwm_test.cpp)
#     include_directories(${Boost_INCLUDE_DIRS})
#     TARGET_LINK_LIBRARIES(pwm_test hbridge ${Boost_LIBRARIES} ${CANBUS_LIBRARIES})

else(Boost_FOUND)
    message(STATUS "boost/unit not found, disabled unit test")
endif (Boost_FOUND)

ADD_EXECUTABLE(new_driver_test new_driver_test.cpp)
include_directories(${Boost_INCLUDE_DIRS})
TARGET_LINK_LIBRARIES(new_driver_test hbridge ${Boost_LIBRARIES} ${CANBUS_LIBRARIES})

# ADD_EXECUTABLE(sandbox_test sandbox_test.cpp)
# include_directories(${Boost_INCLUDE_DIRS})
# TARGET_LINK_LIBRARIES(sandbox_test sandbox_lib stubs hbridge ${Boost_LIBRARIES} ${CANBUS_LIBRARIES})


