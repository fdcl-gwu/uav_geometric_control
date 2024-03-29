cmake_minimum_required(VERSION 2.8.2)

project(fdcl_controller)
set (CMAKE_CXX_STANDARD 11)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/libraries/eigen)
include_directories(${PROJECT_SOURCE_DIR}/libraries/fdcl_param/include)

add_subdirectory(${PROJECT_SOURCE_DIR}/libraries/fdcl_param fdcl_param)

## Controller
add_library(controller 
    src/fdcl_control.cpp
    src/fdcl_matrix_utils.cpp
)
target_link_libraries(controller
    PUBLIC fdcl_param
)

## Main executable
add_executable(example 
    src/main.cpp
)
target_link_libraries(example
    PUBLIC controller fdcl_param
)



## Following are for unit testing ##############################################
# Trivial example using gtest and gmock
include(CTest)

if (CMAKE_VERSION VERSION_LESS 3.2)
    set(UPDATE_DISCONNECTED_IF_AVAILABLE "")
else()
    set(UPDATE_DISCONNECTED_IF_AVAILABLE "UPDATE_DISCONNECTED 1")
endif()

include(DownloadProject.cmake)
download_project(PROJ                googletest
                 GIT_REPOSITORY      https://github.com/google/googletest.git
                 GIT_TAG             main
                 ${UPDATE_DISCONNECTED_IF_AVAILABLE}
)

# Prevent GoogleTest from overriding our compiler/linker options
# when building with Visual Studio
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

add_subdirectory(${googletest_SOURCE_DIR} ${googletest_BINARY_DIR})

# When using CMake 2.8.11 or later, header path dependencies
# are automatically added to the gtest and gmock targets.
# For earlier CMake versions, we have to explicitly add the
# required directories to the header search path ourselves.
if (CMAKE_VERSION VERSION_LESS 2.8.11)
    include_directories("${gtest_SOURCE_DIR}/include"
                        "${gmock_SOURCE_DIR}/include")
endif()

add_executable(test_all 
    tests/test_all.cpp
    tests/test_matrix_utils.cpp
    tests/test_controller.cpp
)
target_link_libraries(test_all 
    gtest 
    gmock_main
    controller
)
add_test(NAME unit_test COMMAND test_all)
