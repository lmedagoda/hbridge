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


CONFIGURE_FILE(Doxyfile.in Doxyfile @ONLY)
ADD_CUSTOM_TARGET(doc doxygen Doxyfile)

CONFIGURE_FILE(hbridge.pc.in hbridge.pc @ONLY)
INSTALL(FILES ${CMAKE_BINARY_DIR}/hbridge.pc DESTINATION lib/pkgconfig)

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR})

add_subdirectory(firmware/stubs)
add_subdirectory(firmware/mainboard/stubs)

add_subdirectory(src)

add_subdirectory(test)

