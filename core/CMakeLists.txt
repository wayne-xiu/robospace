# Core C++ library

# Math module
set(MATH_SOURCES
    math/transform.cpp
    math/rotation.cpp
    math/se3.cpp
    math/SE3.cpp
    math/so3.cpp
    math/SO3.cpp
)

set(MATH_HEADERS
    math/transform.hpp
    math/rotation.hpp
    math/se3.hpp
    math/SE3.hpp
    math/so3.hpp
    math/SO3.hpp
)

add_library(robospace-math ${MATH_SOURCES} ${MATH_HEADERS})
target_include_directories(robospace-math PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(robospace-math PUBLIC Eigen3::Eigen)

# Set C++ standard for this target
set_target_properties(robospace-math PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    POSITION_INDEPENDENT_CODE ON
)

# Model module (will be populated in Phase 1)
# add_library(robospace-model ...)

# Kinematics module (will be populated in Phase 1)
# add_library(robospace-kinematics ...)

# Main robospace library (combines all modules)
# For now, this is just an alias to robospace-math
add_library(robospace ALIAS robospace-math)

# When we add more modules, we'll create a combined library:
# add_library(robospace INTERFACE)
# target_link_libraries(robospace INTERFACE
#     robospace-math
#     robospace-model
#     robospace-kinematics
# )

# Installation
install(TARGETS robospace-math
    EXPORT robospaceTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)
