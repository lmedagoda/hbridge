# - Try to find libelf lib
# Once done this will define
#
#  LIBELF_FOUND - system has libelf lib
#  LIBELF_INCLUDE_DIR - the libelf include directory
#  LIBELF_LIBRARY - the libelf library
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.

if (LIBELF_INCLUDE_DIR)

  # in cache already
  set(LIBELF_FOUND TRUE)

else (LIBELF_INCLUDE_DIR)

find_path(LIBELF_INCLUDE_DIR NAMES libelf.h
  PATH_SUFFIXES
     HINTS
     ${INCLUDE_INSTALL_DIR}
   )

set(LIBELF_NAMES ${LIBELF_NAMES} elf)
find_library(LIBELF_LIBRARY NAMES ${LIBELF_NAMES}
	$(LIBRARY_INSTALL_DIR)
	)

# handle the QUIETLY and REQUIRED arguments and set AMD_FOUND to TRUE if
# all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(LIBELF  DEFAULT_MSG LIBELF_LIBRARY LIBELF_INCLUDE_DIR)

mark_as_advanced(LIBELF_INCLUDE_DIR LIBELF_LIBRARY)

endif(LIBELF_INCLUDE_DIR)

