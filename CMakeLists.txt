
PROJECT(hbridge)

cmake_minimum_required(VERSION 2.6)

#if you don't want the full compiler output, remove the following line
SET(CMAKE_VERBOSE_MAKEFILE ON)

#add definitions, compiler switches, etc.
ADD_DEFINITIONS(-Wall -O2)

#list all source files here
#ADD_EXECUTABLE(driver main.cpp interface.cpp)
find_package(PkgConfig)

pkg_check_modules(DFKI_TYPES REQUIRED "dfki-types")
INCLUDE_DIRECTORIES(${DFKI_TYPES_INCLUDE_DIRS})
LINK_DIRECTORIES(${DFKI_TYPES_LIBRARY_DIRS})

ADD_LIBRARY(hbridge SHARED Protocol.cpp)
TARGET_LINK_LIBRARIES(hbridge ${DFKI_TYPES_LIBS})

#need to link to some other libraries ? just add them here
#TARGET_LINK_LIBRARIES(driver png jpeg)
 
CONFIGURE_FILE(Doxyfile.in Doxyfile @ONLY)
ADD_CUSTOM_TARGET(doc doxygen Doxyfile)
INSTALL(TARGETS hbridge
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib)
INSTALL(FILES Protocol.hpp DESTINATION include)
INSTALL(FILES firmware/protocol.h DESTINATION include/firmware)

CONFIGURE_FILE(hbridge.pc.in hbridge.pc @ONLY)
INSTALL(FILES ${CMAKE_BINARY_DIR}/hbridge.pc DESTINATION lib/pkgconfig)
