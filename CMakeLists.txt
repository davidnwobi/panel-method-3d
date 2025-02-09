# CMakeList.txt : CMake project for Inviscid3d, include source and define
cmake_minimum_required(VERSION 3.21)
project(Inviscid3d VERSION 0.6 LANGUAGES C CXX)  
# project specific logic here.

set (CMAKE_CXX_STANDARD 20)
set (CMAKE_CXX_STANDARD_REQUIRED True)
set (CMAKE_EXPORT_COMPILE_COMMANDS on)
cmake_policy(SET CMP0135 NEW)

add_compile_definitions(SOURCE_DIR="${CMAKE_SOURCE_DIR}")
#add_compile_definitions(ANALYSIS_DIR="${CMAKE_SOURCE_DIR}/out_cpp")
add_compile_definitions(ANALYSIS_DIR="/home/okwuchukwu-nwobi/Projects/PanelMethods/python/out_cpp")
#add_compile_options(-Werror -Wall -Wextra -pedantic-errors -Wno-conversion -Wno-sign-conversion -Wno-shorten-64-to-32 -Wno-sign-compare -g -O3)
add_compile_options(-O3 -march=native)

# EXTERNAL PACKAGES 
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(PCL 1.3 REQUIRED)


# NOTE: PanelMethods Library
file(
  GLOB_RECURSE 
  SOURCES 
  "${PROJECT_SOURCE_DIR}/src/*.cpp" 
  "${PROJECT_SOURCE_DIR}/src/surface/*.cpp"
  "${PROJECT_SOURCE_DIR}/src/panel_geo/*.cpp"
  "${PROJECT_SOURCE_DIR}/src/panel_method/*.cpp"
)

message (STATUS "Sources are: " ${SOURCES})
add_library(
  PanelMethods 
  ${SOURCES}
)

set_target_properties(
  PanelMethods 
  PROPERTIES 
  LINKER_LANGUAGE 
  CXX
)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

target_include_directories(PanelMethods PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(PanelMethods PUBLIC Eigen3::Eigen )


add_library(mat_reader ${PROJECT_SOURCE_DIR}/src/mat_reader/mat_reader.cpp) 
target_include_directories (mat_reader PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(mat_reader Eigen3::Eigen ${PCL_LIBRARIES})


# NOTE: Main
add_executable (main main.cpp)
target_link_libraries(main PUBLIC PanelMethods mat_reader )


if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET main PROPERTY CXX_STANDARD 20)
endif()



#  TODO: Add tests and install targets if needed.

 include(FetchContent)
 FetchContent_Declare(
   googletest
   URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
   DOWNLOAD_EXTRACT_TIMESTAMP TRUE
 )
 # For Windows: Prevent overriding the parent project's compiler/linker settings
 set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
 FetchContent_MakeAvailable(googletest)

 enable_testing()

 #SURFACE
 add_executable(surface_test tests/surface/surface_test.cpp)
 target_link_libraries(surface_test GTest::gtest_main PanelMethods)

 #PANEL GEOMETRY
 add_executable(panel_geo_test tests/panel_geo/panel_geo_test.cpp)
 target_link_libraries(panel_geo_test GTest::gtest_main PanelMethods)

 #SINGULARITIES
 add_executable(singularity_test tests/singularity/singularity_test.cpp)
 target_link_libraries(singularity_test GTest::gtest_main PanelMethods)

 add_executable(utils_test tests/utils/utils_test.cpp)
 target_link_libraries(utils_test GTest::gtest_main PanelMethods)

 add_executable(central_difference_test tests/central_difference_test.cpp)
 target_link_libraries(central_difference_test GTest::gtest_main PanelMethods)

 add_executable(test_case_validation_wing tests/test_case_validation_wing/test_case_validation_wing.cpp)
 target_link_libraries(test_case_validation_wing GTest::gtest_main PanelMethods)

add_executable(octree_test tests/octree/octree_test.cpp)
target_link_libraries(octree_test PUBLIC ${PCL_LIBRARIES} Eigen3::Eigen GTest::gtest_main)
target_include_directories(octree_test 
  PUBLIC 
  ${PROJECT_SOURCE_DIR}/include 
)


 include(GoogleTest)
 gtest_discover_tests(surface_test panel_geo_test, singularity_test, utils_test, test_case_validation_wing, octree_test)



