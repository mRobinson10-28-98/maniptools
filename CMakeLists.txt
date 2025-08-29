cmake_minimum_required(VERSION 3.8)
project(maniptools)

# GoogleTest requires at least C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

option(INSTALL_GTEST OFF)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
)
FetchContent_MakeAvailable(googletest)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Include directory for Eigen3 and internal includes
include_directories(/usr/include/eigen3)
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/include/JointController)

# Manipulator toolset; twists and other LA functions
add_library(maniptools 
  src/Common.cpp
  src/DualNumberTools.cpp
  src/TwistJoint.cpp 
  src/ManipNDof.cpp
  src/JointController/JointControllerBase.cpp
  src/JointController/JointControllerSnap.cpp
  src/JointController/JointControllerStepResponse.cpp
  src/Manip5Dof.cpp
  src/SimClock.cpp
)


add_executable(my_test src/my_test.cpp)
target_link_libraries(my_test maniptools)
add_executable(manip5dof_tracecircle src/manip5dof_tracecircle.cpp)
target_link_libraries(manip5dof_tracecircle maniptools)

install(TARGETS
  my_test
  DESTINATION ${PROJECT_SOURCE_DIR}/bin
)

install(TARGETS
  maniptools
  DESTINATION ${PROJECT_SOURCE_DIR}/lib
)

enable_testing()

add_executable(maniptools_test 
    test/manipndof_test.cpp
    test/skew_test.cpp
    test/twist_test.cpp
    test/simclock_test.cpp
    test/joint_controller_test.cpp
    test/dual_numbers_test.cpp
)
target_link_libraries(maniptools_test GTest::gtest_main maniptools)
target_include_directories(maniptools_test PUBLIC ${PROJECT_SOURCE_DIR}/test)

include(GoogleTest)
gtest_discover_tests(maniptools_test)
