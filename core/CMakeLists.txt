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
