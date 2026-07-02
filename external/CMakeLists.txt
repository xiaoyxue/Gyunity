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

include(cmake/ide.cmake)
# ext headers
message("Create header project...")
add_custom_target(ext_headers SOURCES ${SPD_ALL_HEADERS} ${TBB_ALL_HEADERS} ${TINY_OBJ_LOADER_ALL_HEADERS} ${STB_HEADERS} ${GOOGLE_TEST_ALL_HEADERS})