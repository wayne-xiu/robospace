# Core C++ library

# Math module
set(MATH_SOURCES
    ${CMAKE_SOURCE_DIR}/src/math/se3_algebra.cpp
    ${CMAKE_SOURCE_DIR}/src/math/SE3.cpp
    ${CMAKE_SOURCE_DIR}/src/math/so3_algebra.cpp
    ${CMAKE_SOURCE_DIR}/src/math/SO3.cpp
)

set(MATH_HEADERS
    ${CMAKE_SOURCE_DIR}/include/robospace/math/se3_algebra.hpp
    ${CMAKE_SOURCE_DIR}/include/robospace/math/SE3.hpp
    ${CMAKE_SOURCE_DIR}/include/robospace/math/so3_algebra.hpp
    ${CMAKE_SOURCE_DIR}/include/robospace/math/SO3.hpp
)

add_library(robospace-math ${MATH_SOURCES} ${MATH_HEADERS})
target_include_directories(robospace-math PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(robospace-math PUBLIC Eigen3::Eigen)

# Set C++ standard for this target
set_target_properties(robospace-math PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    POSITION_INDEPENDENT_CODE ON
)

# Model module
set(MODEL_SOURCES
    ${CMAKE_SOURCE_DIR}/src/model/entity.cpp
    ${CMAKE_SOURCE_DIR}/src/model/dh_params.cpp
    ${CMAKE_SOURCE_DIR}/src/model/joint.cpp
    ${CMAKE_SOURCE_DIR}/src/model/link.cpp
    ${CMAKE_SOURCE_DIR}/src/model/frame.cpp
    ${CMAKE_SOURCE_DIR}/src/model/kinematic_tree.cpp
    ${CMAKE_SOURCE_DIR}/src/model/tool.cpp
    ${CMAKE_SOURCE_DIR}/src/model/robot.cpp
    ${CMAKE_SOURCE_DIR}/src/model/urdf_parser.cpp
)

set(MODEL_HEADERS
    ${CMAKE_SOURCE_DIR}/include/robospace/model/entity.hpp
    ${CMAKE_SOURCE_DIR}/include/robospace/model/dh_params.hpp
    ${CMAKE_SOURCE_DIR}/include/robospace/model/joint.hpp
    ${CMAKE_SOURCE_DIR}/include/robospace/model/link.hpp
    ${CMAKE_SOURCE_DIR}/include/robospace/model/frame.hpp
    ${CMAKE_SOURCE_DIR}/include/robospace/model/kinematic_tree.hpp
    ${CMAKE_SOURCE_DIR}/include/robospace/model/tool.hpp
    ${CMAKE_SOURCE_DIR}/include/robospace/model/robot.hpp
    ${CMAKE_SOURCE_DIR}/include/robospace/model/urdf_parser.hpp
)

add_library(robospace-model ${MODEL_SOURCES} ${MODEL_HEADERS})
target_include_directories(robospace-model PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(robospace-model PUBLIC
    Eigen3::Eigen
    robospace-math
    tinyxml2
    nlohmann_json::nlohmann_json
)
set_target_properties(robospace-model PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    POSITION_INDEPENDENT_CODE ON
)

# Kinematics module
set(KINEMATICS_SOURCES
    ${CMAKE_SOURCE_DIR}/src/kinematics/ik_solver.cpp
    ${CMAKE_SOURCE_DIR}/src/kinematics/numerical_ik.cpp
    ${CMAKE_SOURCE_DIR}/src/kinematics/analytical_ik_base.cpp
    ${CMAKE_SOURCE_DIR}/src/kinematics/spherical_wrist_ik.cpp
)

set(KINEMATICS_HEADERS
    ${CMAKE_SOURCE_DIR}/include/robospace/kinematics/ik_solver.hpp
    ${CMAKE_SOURCE_DIR}/include/robospace/kinematics/numerical_ik.hpp
    ${CMAKE_SOURCE_DIR}/include/robospace/kinematics/analytical_ik_base.hpp
    ${CMAKE_SOURCE_DIR}/include/robospace/kinematics/spherical_wrist_ik.hpp
)

add_library(robospace-kinematics ${KINEMATICS_SOURCES} ${KINEMATICS_HEADERS})
target_include_directories(robospace-kinematics PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(robospace-kinematics PUBLIC
    Eigen3::Eigen
    robospace-math
    robospace-model
)
set_target_properties(robospace-kinematics PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    POSITION_INDEPENDENT_CODE ON
)

# Main robospace library (combines all modules)
add_library(robospace-core INTERFACE)
target_link_libraries(robospace-core INTERFACE
    robospace-math
    robospace-model
    robospace-kinematics
)
add_library(robospace ALIAS robospace-core)

# When we add more modules, we'll create a combined library:
# add_library(robospace INTERFACE)
# target_link_libraries(robospace INTERFACE
#     robospace-math
#     robospace-model
#     robospace-kinematics
# )

# Installation
install(TARGETS robospace-math robospace-model robospace-kinematics
    EXPORT robospaceTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)
