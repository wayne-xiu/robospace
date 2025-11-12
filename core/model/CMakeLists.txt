# Model library
set(MODEL_SOURCES
    ${CMAKE_SOURCE_DIR}/src/model/entity.cpp
    ${CMAKE_SOURCE_DIR}/src/model/dh_params.cpp
    ${CMAKE_SOURCE_DIR}/src/model/joint.cpp
    ${CMAKE_SOURCE_DIR}/src/model/link.cpp
    ${CMAKE_SOURCE_DIR}/src/model/frame.cpp
    ${CMAKE_SOURCE_DIR}/src/model/kinematic_tree.cpp
    ${CMAKE_SOURCE_DIR}/src/model/robot.cpp
)

add_library(robospace-model ${MODEL_SOURCES})

target_include_directories(robospace-model
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(robospace-model
    PUBLIC
        Eigen3::Eigen
        robospace-math
)

target_compile_features(robospace-model PUBLIC cxx_std_17)
