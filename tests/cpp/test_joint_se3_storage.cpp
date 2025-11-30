#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <robospace/model/joint.hpp>
#include <robospace/model/dh_params.hpp>
#include <robospace/model/kinematic_tree.hpp>
#include <robospace/math/SE3.hpp>
#include <cmath>

using namespace robospace;
using namespace robospace::model;
using namespace robospace::math;

TEST_CASE("Joint: SE3 precomputation from DH parameters", "[joint][se3][storage]") {
    // Create a joint with DH parameters
    Joint joint("test_joint", JointType::REVOLUTE, 0, 1);
    DHParams dh(M_PI/2, 0.5, 0.1, M_PI/4, DHConvention::STANDARD);
    joint.set_dh_params(dh);

    SECTION("Before precomputation") {
        REQUIRE_FALSE(joint.has_dh_se3());
    }

    SECTION("After manual precomputation") {
        joint.precompute_dh_se3();

        REQUIRE(joint.has_dh_se3());

        // Verify precomputed SE3 matches DH transform at q=0
        SE3 dh_transform = dh.transform(0.0, false);
        SE3 precomputed = joint.dh_se3_base();

        // Check matrices are equal
        REQUIRE_THAT((dh_transform.matrix() - precomputed.matrix()).norm(),
                     Catch::Matchers::WithinAbs(0.0, 1e-12));
    }
}

TEST_CASE("Joint: SE3 precomputation for different DH conventions", "[joint][se3][storage]") {
    SECTION("Standard DH") {
        Joint joint("std_joint", JointType::REVOLUTE, 0, 1);
        DHParams dh(0.0, 0.42500, 0.0, 0.0, DHConvention::STANDARD);
        joint.set_dh_params(dh);
        joint.precompute_dh_se3();

        REQUIRE(joint.has_dh_se3());

        // At q=0, should be pure translation along X by 'a'
        SE3 T = joint.dh_se3_base();
        REQUIRE_THAT(T.translation().x(), Catch::Matchers::WithinAbs(0.42500, 1e-6));
        REQUIRE_THAT(T.translation().y(), Catch::Matchers::WithinAbs(0.0, 1e-6));
        REQUIRE_THAT(T.translation().z(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    }

    SECTION("Modified DH") {
        Joint joint("mod_joint", JointType::REVOLUTE, 0, 1);
        DHParams dh(M_PI/2, 0.0, 0.0, 0.0, DHConvention::MODIFIED);
        joint.set_dh_params(dh);
        joint.precompute_dh_se3();

        REQUIRE(joint.has_dh_se3());

        // Rotation includes alpha rotation
        SE3 T = joint.dh_se3_base();
        REQUIRE_THAT(T.rotation()(1, 1), Catch::Matchers::WithinAbs(0.0, 1e-6));
    }
}

TEST_CASE("Joint: SE3 precomputation for prismatic joints", "[joint][se3][storage]") {
    Joint joint("prismatic", JointType::PRISMATIC, 0, 1);
    DHParams dh(0.0, 0.0, 0.2, 0.0, DHConvention::STANDARD);
    joint.set_dh_params(dh);
    joint.precompute_dh_se3();

    REQUIRE(joint.has_dh_se3());

    // At q=0, should have d offset in Z
    SE3 T = joint.dh_se3_base();
    REQUIRE_THAT(T.translation().z(), Catch::Matchers::WithinAbs(0.2, 1e-6));
}

TEST_CASE("Joint: No precomputation for joints without DH parameters", "[joint][se3][storage]") {
    Joint joint("urdf_style", JointType::REVOLUTE, 0, 1);
    // Don't set DH parameters

    joint.precompute_dh_se3();

    // Should not have precomputed SE3
    REQUIRE_FALSE(joint.has_dh_se3());
}

TEST_CASE("KinematicTree: Automatic SE3 precomputation on add_joint", "[kinematic_tree][se3][storage]") {
    KinematicTree tree;

    tree.add_link(Link("base"));
    tree.add_link(Link("link1"));

    // Create joint with DH parameters
    Joint joint("joint1", JointType::REVOLUTE, 0, 1);
    DHParams dh(0.0, 1.0, 0.0, 0.0, DHConvention::STANDARD);
    joint.set_dh_params(dh);

    // Add to tree - should automatically precompute
    tree.add_joint(joint);

    // Retrieve and verify
    const Joint& stored_joint = tree.joint(0);
    REQUIRE(stored_joint.has_dh_se3());

    // Verify correctness
    SE3 expected = dh.transform(0.0, false);
    SE3 actual = stored_joint.dh_se3_base();
    REQUIRE_THAT((expected.matrix() - actual.matrix()).norm(),
                 Catch::Matchers::WithinAbs(0.0, 1e-12));
}

TEST_CASE("Joint: SE3 consistency with multiple DH configurations", "[joint][se3][storage]") {
    // Test various DH parameter combinations
    std::vector<DHParams> test_configs = {
        {0.0, 0.0, 0.089159, 0.0, DHConvention::STANDARD},  // UR5 J1
        {M_PI/2, 0.0, 0.0, 0.0, DHConvention::STANDARD},     // UR5 J2
        {0.0, 0.42500, 0.0, 0.0, DHConvention::STANDARD},    // UR5 J3
        {M_PI/2, 0.0, 0.290, M_PI/2, DHConvention::MODIFIED} // Complex modified DH
    };

    for (const auto& dh : test_configs) {
        Joint joint("test", JointType::REVOLUTE, 0, 1);
        joint.set_dh_params(dh);
        joint.precompute_dh_se3();

        // Verify precomputed matches runtime computation
        SE3 runtime = dh.transform(0.0, false);
        SE3 precomputed = joint.dh_se3_base();

        REQUIRE_THAT((runtime.matrix() - precomputed.matrix()).norm(),
                     Catch::Matchers::WithinAbs(0.0, 1e-12));
    }
}
