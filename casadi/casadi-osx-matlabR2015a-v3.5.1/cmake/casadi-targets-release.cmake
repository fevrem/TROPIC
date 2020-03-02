#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "casadi" for configuration "Release"
set_property(TARGET casadi APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(casadi PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/casadi/libcasadi.dylib"
  IMPORTED_SONAME_RELEASE "@rpath/libcasadi.dylib"
  )

list(APPEND _IMPORT_CHECK_TARGETS casadi )
list(APPEND _IMPORT_CHECK_FILES_FOR_casadi "${_IMPORT_PREFIX}/casadi/libcasadi.dylib" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
