cmake_minimum_required(VERSION 3.8)
project(maniptools)

# GoogleTest requires at least C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

option(INSTALL_GTEST OFF)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
FetchContent_MakeAvailable(googletest)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Include directory for Eigen3 and internal includes
include_directories(/usr/include/eigen3)
include_directories(${PROJECT_SOURCE_DIR}/include)

# Manipulator toolset; twists and other LA functions
add_library(maniptools 
  src/skew.cpp 
  src/TwistJoint.cpp 
  src/ManipNDof.cpp
  src/JointControllerLerp.cpp 
  src/JointControllerSnap.cpp
  src/Manip5Dof.cpp)


add_executable(my_test src/my_test.cpp)
target_link_libraries(my_test maniptools)

install(TARGETS
  maniptools
  my_test
  DESTINATION ${PROJECT_SOURCE_DIR}/bin/
)

enable_testing()

add_executable(maniptools_test 
    test/manipndof_test.cpp
    test/skew_test.cpp
    test/twist_test.cpp
)
target_link_libraries(maniptools_test GTest::gtest_main maniptools)

include(GoogleTest)
gtest_discover_tests(maniptools_test)
