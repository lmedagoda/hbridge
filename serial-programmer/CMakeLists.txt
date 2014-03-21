PROJECT(serial-programmer)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})

cmake_minimum_required(VERSION 2.6)
ADD_DEFINITIONS(-Wall)

ADD_EXECUTABLE(stm32prog stm32prog.cc stm32proto.cc imagereader.cc)

#find_package(LibElf)

#if(LIBELF_FOUND)
#  add_definitions(-DHAVE_LIBELF)
#  include_directories(${LIBELF_INCLUDE_DIR})
#  LINK_DIRECTORIES(${LIBELF_LIBRARY_DIRS})
#  TARGET_LINK_LIBRARIES(stm32prog ${LIBELF_LIBRARY})
#  message(STATUS "libelf found, enabling reading elf images")
#else()
  message(STATUS "libelf missing, disabling reading elf images")
  #endif()

