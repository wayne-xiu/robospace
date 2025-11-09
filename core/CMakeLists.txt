# Core C++ library

# Math module
set(MATH_SOURCES
    ${CMAKE_SOURCE_DIR}/src/math/transform.cpp
    ${CMAKE_SOURCE_DIR}/src/math/rotation.cpp
    ${CMAKE_SOURCE_DIR}/src/math/se3.cpp
    ${CMAKE_SOURCE_DIR}/src/math/SE3.cpp
    ${CMAKE_SOURCE_DIR}/src/math/so3.cpp
    ${CMAKE_SOURCE_DIR}/src/math/SO3.cpp
)

set(MATH_HEADERS
    ${CMAKE_SOURCE_DIR}/include/robospace/math/transform.hpp
    ${CMAKE_SOURCE_DIR}/include/robospace/math/rotation.hpp
    ${CMAKE_SOURCE_DIR}/include/robospace/math/se3.hpp
    ${CMAKE_SOURCE_DIR}/include/robospace/math/SE3.hpp
    ${CMAKE_SOURCE_DIR}/include/robospace/math/so3.hpp
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
    ${CMAKE_SOURCE_DIR}/src/model/dh_params.cpp
    ${CMAKE_SOURCE_DIR}/src/model/joint.cpp
    ${CMAKE_SOURCE_DIR}/src/model/link.cpp
    ${CMAKE_SOURCE_DIR}/src/model/frame.cpp
)

set(MODEL_HEADERS
    ${CMAKE_SOURCE_DIR}/include/robospace/model/dh_params.hpp
    ${CMAKE_SOURCE_DIR}/include/robospace/model/joint.hpp
    ${CMAKE_SOURCE_DIR}/include/robospace/model/link.hpp
    ${CMAKE_SOURCE_DIR}/include/robospace/model/frame.hpp
)

add_library(robospace-model ${MODEL_SOURCES} ${MODEL_HEADERS})
target_include_directories(robospace-model PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(robospace-model PUBLIC
    Eigen3::Eigen
    robospace-math
)
set_target_properties(robospace-model PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    POSITION_INDEPENDENT_CODE ON
)

# Kinematics module (will be populated later in Phase 1)
# add_library(robospace-kinematics ...)

# Main robospace library (combines all modules)
add_library(robospace-core INTERFACE)
target_link_libraries(robospace-core INTERFACE
    robospace-math
    robospace-model
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
install(TARGETS robospace-math robospace-model
    EXPORT robospaceTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)
