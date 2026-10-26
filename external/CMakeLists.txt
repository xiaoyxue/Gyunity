cmake_minimum_required (VERSION 3.5...3.27)
project(ext_build)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/build/runtime")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/build/")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/build/lib")
# tbb
set(TBB_BUILD_SHARED          ON  CACHE BOOL " " FORCE)
set(TBB_BUILD_STATIC          OFF CACHE BOOL " " FORCE)
set(TBB_BUILD_TESTS           OFF CACHE BOOL " " FORCE)
set(TBB_BUILD_TBBMALLOC       OFF CACHE BOOL " " FORCE)
set(TBB_BUILD_TBBMALLOC_PROXY OFF CACHE BOOL " " FORCE)
add_subdirectory(tbb)
if(MSVC)
  # TBB's C++ options must not be passed to its MASM sources.
  get_target_property(TBB_CXX_OPTIONS tbb_interface INTERFACE_COMPILE_OPTIONS)
  set_property(TARGET tbb_interface PROPERTY INTERFACE_COMPILE_OPTIONS
    "$<$<COMPILE_LANGUAGE:CXX>:${TBB_CXX_OPTIONS}>")
  get_target_property(TBB_CXX_FLAGS tbb COMPILE_FLAGS)
  separate_arguments(TBB_CXX_FLAGS NATIVE_COMMAND "${TBB_CXX_FLAGS}")
  set_property(TARGET tbb PROPERTY COMPILE_FLAGS "")
  target_compile_options(tbb PRIVATE "$<$<COMPILE_LANGUAGE:CXX>:${TBB_CXX_FLAGS}>")
endif()
set_property(TARGET tbb tbb_def_files PROPERTY FOLDER "dependencies")
set(TBB_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/tbb/include)
#set(TBB_INCLUDE_DIRS ${TBB_INCLUDE_DIRS} PARENT_SCOPE)


# spdlog
add_subdirectory(spdlog)
set(SPDLOG_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/spdlog/include)
#set(SPDLOG_INCLUDE_DIRS ${SPDLOG_INCLUDE_DIRS} PARENT_SCOPE)
# fmt
set(FMT_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/spdlog/include/spdlog)
#set(FMT_INCLUDE_DIRS ${FMT_INCLUDE_DIRS} PARENT_SCOPE)

# googletest
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
add_subdirectory(googletest)
set(GOOGLE_TEST_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/googletest/googletest/include)
set(GOOGLE_MOCK_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/googletest/googlemock/include)

# Keep EXR dependencies local and reproducible without changing the older libraries.
set(_gyunity_shared_libraries "${BUILD_SHARED_LIBS}")
set(BUILD_SHARED_LIBS ON)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(IMATH_INSTALL OFF CACHE BOOL "" FORCE)
set(IMATH_INSTALL_PKG_CONFIG OFF CACHE BOOL "" FORCE)
set(OPENEXR_INSTALL OFF CACHE BOOL "" FORCE)
set(OPENEXR_INSTALL_PKG_CONFIG OFF CACHE BOOL "" FORCE)
set(OPENEXR_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
set(OPENEXR_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(OPENEXR_BUILD_PYTHON OFF CACHE BOOL "" FORCE)
set(OPENEXR_FORCE_INTERNAL_DEFLATE ON CACHE BOOL "" FORCE)
set(OPENEXR_FORCE_INTERNAL_OPENJPH ON CACHE BOOL "" FORCE)
set(OPENEXR_USE_TBB OFF CACHE BOOL "" FORCE)
add_subdirectory(imath)
# OpenEXR calls find_package before CMake generates Imath's build-tree export.
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/imath/config/ImathTargets.cmake"
  "# Imath targets are already defined by the parent dependency build.\n")
add_subdirectory(openexr)
foreach(_exr_target Iex IlmThread OpenEXRCore OpenEXR OpenEXRUtil)
  if(TARGET ${_exr_target})
    set_target_properties(${_exr_target} PROPERTIES
      RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/build/runtime")
  endif()
endforeach()
set(BUILD_SHARED_LIBS "${_gyunity_shared_libraries}")
unset(_gyunity_shared_libraries)

include(cmake/ide.cmake)
# ext headers
message("Create header project...")
add_custom_target(ext_headers SOURCES ${SPD_ALL_HEADERS} ${TBB_ALL_HEADERS} ${TINY_OBJ_LOADER_ALL_HEADERS} ${STB_HEADERS} ${GOOGLE_TEST_ALL_HEADERS})