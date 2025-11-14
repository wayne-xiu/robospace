cmake_minimum_required(VERSION 3.18)

# Fetch Catch2
include(FetchContent)
FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG v3.5.1
)
FetchContent_MakeAvailable(Catch2)

# Test executable (Catch2WithMain provides main())
add_executable(robospace-tests
    cpp/test_transform.cpp
    cpp/test_rotation.cpp
    cpp/test_se3.cpp
    cpp/test_SE3.cpp
    cpp/test_so3.cpp
    cpp/test_SO3.cpp
    cpp/test_model_basic.cpp
    cpp/test_industrial_robots.cpp
    cpp/test_frame.cpp
    cpp/test_kinematic_tree.cpp
    cpp/test_robot.cpp
    cpp/test_urdf_parser.cpp
    cpp/test_forward_kinematics.cpp
    cpp/test_units.cpp
    cpp/test_jacobian.cpp
    cpp/test_inverse_kinematics.cpp
)

target_link_libraries(robospace-tests PRIVATE
    robospace-math
    robospace-model
    robospace-kinematics
    Catch2::Catch2WithMain
)

# Enable testing
include(CTest)
include(Catch)
catch_discover_tests(robospace-tests)

# Copy test data files to build directory
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/test_data
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

# Set C++ standard
set_target_properties(robospace-tests PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)
