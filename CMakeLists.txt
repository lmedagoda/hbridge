PROJECT(hbridge)

cmake_minimum_required(VERSION 2.6)
ADD_DEFINITIONS(-Wall)

find_package(PkgConfig)

pkg_check_modules(BASE REQUIRED "base-types")
INCLUDE_DIRECTORIES(${BASE_INCLUDE_DIRS})
LINK_DIRECTORIES(${BASE_LIBRARY_DIRS})

pkg_check_modules(CANBUS_DRIVER REQUIRED "canbus")
INCLUDE_DIRECTORIES(${CANBUS_DRIVER_INCLUDE_DIRS})
LINK_DIRECTORIES(${CANBUS_DRIVER_LIBRARY_DIRS})

ADD_LIBRARY(hbridge SHARED HBridgeDriver.cpp)
TARGET_LINK_LIBRARIES(hbridge ${CANBUS_DRIVER_LIBRARIES} ${BASE_LIBRARIES})

CONFIGURE_FILE(Doxyfile.in Doxyfile @ONLY)
ADD_CUSTOM_TARGET(doc doxygen Doxyfile)
INSTALL(TARGETS hbridge
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib)
INSTALL(FILES HBridge.hpp HBridgeDriver.hpp DESTINATION include)
INSTALL(FILES firmware/protocol.h DESTINATION include/firmware)

CONFIGURE_FILE(hbridge.pc.in hbridge.pc @ONLY)
INSTALL(FILES ${CMAKE_BINARY_DIR}/hbridge.pc DESTINATION lib/pkgconfig)

add_subdirectory(test)


add_subdirectory(src)