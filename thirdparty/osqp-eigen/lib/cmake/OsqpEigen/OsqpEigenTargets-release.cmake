#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "OsqpEigen::OsqpEigen" for configuration "Release"
set_property(TARGET OsqpEigen::OsqpEigen APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(OsqpEigen::OsqpEigen PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libOsqpEigen.so.0.8.1"
  IMPORTED_SONAME_RELEASE "libOsqpEigen.so.0.8.1"
  )

list(APPEND _cmake_import_check_targets OsqpEigen::OsqpEigen )
list(APPEND _cmake_import_check_files_for_OsqpEigen::OsqpEigen "${_IMPORT_PREFIX}/lib/libOsqpEigen.so.0.8.1" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
