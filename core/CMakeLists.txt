# Core C++ library

# Math module
add_library(robospace-math INTERFACE)
target_include_directories(robospace-math INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(robospace-math INTERFACE Eigen3::Eigen)

# Model module (will be populated in Phase 1)
# add_library(robospace-model ...)

# Kinematics module (will be populated in Phase 1)
# add_library(robospace-kinematics ...)

# Main robospace library (combines all modules)
add_library(robospace INTERFACE)
target_link_libraries(robospace INTERFACE
    robospace-math
    # robospace-model
    # robospace-kinematics
)

# Installation
install(TARGETS robospace robospace-math
    EXPORT robospaceTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)
