cmake_minimum_required(VERSION 3.14)
project(argmap)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Define standard installation directories (GNU)
include(GNUInstallDirs)

# Set default output directories
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")

set(target_name test_argmap)

option(NTL_TESTS "enable NTL tests and fetch NTL too" OFF)

# Fetch deps
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.13.0.zip
)

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

FetchContent_Declare(
  ntl
  URL https://github.com/libntl/ntl/archive/refs/tags/v11.5.1.zip
)

set(NTL_DEP "")

if (NTL_TESTS)
    FetchContent_GetProperties(ntl)
    if (NOT ntl_POPULATED)
        FetchContent_Populate(ntl)
        message(STATUS "Building NTL ... (${ntl_SOURCE_DIR})")
        execute_process(
                        COMMAND ./configure PREFIX="${ntl_SOURCE_DIR}/install"
                        WORKING_DIRECTORY ${ntl_SOURCE_DIR}/src
        )
        execute_process(
                        COMMAND make -j
                        WORKING_DIRECTORY ${ntl_SOURCE_DIR}/src
        )
        execute_process(
                        COMMAND make install
                        WORKING_DIRECTORY ${ntl_SOURCE_DIR}/src
        )
        add_library(ntl OBJECT IMPORTED GLOBAL)
        set_target_properties(ntl PROPERTIES IMPORTED_OBJECTS "${ntl_SOURCE_DIR}/install/lib/libntl.a")
        include_directories(${ntl_SOURCE_DIR}/include)
        message(STATUS "Finished building NTL")
    endif()
    set(NTL_DEP ntl)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DNTL_TESTS")
endif()

# End of fetch

enable_testing()

add_executable(${target_name} ${target_name}.cpp)
target_link_libraries(${target_name} GTest::gtest_main ${NTL_DEP})

include(GoogleTest)
gtest_discover_tests(${target_name})
