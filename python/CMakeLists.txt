# Python bindings for robospace

# Find Python
find_package(Python COMPONENTS Interpreter Development REQUIRED)

# Create Python module
pybind11_add_module(_robospace_core
    bindings/module.cpp
    bindings/math_bindings.cpp
    bindings/model_bindings.cpp
    bindings/kinematics_bindings.cpp
)

# Link against robospace libraries
target_link_libraries(_robospace_core PRIVATE
    robospace-math
    robospace-model
    robospace-kinematics
    Eigen3::Eigen
)

# Set output directory
set_target_properties(_robospace_core PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/robospace
)

# Install Python module
install(TARGETS _robospace_core
    LIBRARY DESTINATION robospace
)
