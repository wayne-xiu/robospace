#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <robospace/model/joint.hpp>
#include <robospace/model/dh_params.hpp>

using namespace robospace::model;
using namespace robospace::math;

// ============================================================================
// Axis Direction Tests (RoboDK "Joint Senses" values 1-6)
// ============================================================================

TEST_CASE("Joint: Set axis direction", "[model][joint][industrial]") {
    Joint j("test", JointType::REVOLUTE, 0, 1);

    REQUIRE(j.axis_direction() == 1);  // Default

    j.set_axis_direction(-1);
    REQUIRE(j.axis_direction() == -1);

    j.set_axis_direction(1);
    REQUIRE(j.axis_direction() == 1);

    // Any positive value should set to +1
    j.set_axis_direction(5);
    REQUIRE(j.axis_direction() == 1);
}

TEST_CASE("Joint: Effective angle with axis direction", "[model][joint][industrial]") {
    Joint j("test", JointType::REVOLUTE, 0, 1);
    Eigen::VectorXd all_q(6);
    all_q.setZero();

    // Normal direction (+1)
    j.set_axis_direction(1);
    double q_eff = j.get_effective_angle(M_PI/4, all_q);
    REQUIRE_THAT(q_eff, Catch::Matchers::WithinAbs(M_PI/4, 1e-10));

    // Inverted direction (-1)
    j.set_axis_direction(-1);
    q_eff = j.get_effective_angle(M_PI/4, all_q);
    REQUIRE_THAT(q_eff, Catch::Matchers::WithinAbs(-M_PI/4, 1e-10));
}

// ============================================================================
// Joint Coupling Tests (RoboDK "Joint Senses" 7th value)
// ============================================================================

TEST_CASE("Joint: Add coupling", "[model][joint][industrial]") {
    Joint j("test", JointType::REVOLUTE, 0, 1);

    REQUIRE_FALSE(j.has_coupling());

    j.add_coupling(1, -1.0);  // Fanuc J2-J3 style
    REQUIRE(j.has_coupling());
    REQUIRE(j.coupling_terms().size() == 1);
    REQUIRE(j.coupling_terms()[0].from_joint_id == 1);
    REQUIRE(j.coupling_terms()[0].coefficient == -1.0);
}

TEST_CASE("Joint: Clear coupling", "[model][joint][industrial]") {
    Joint j("test", JointType::REVOLUTE, 0, 1);
    j.add_coupling(1, -1.0);

    REQUIRE(j.has_coupling());

    j.clear_coupling();
    REQUIRE_FALSE(j.has_coupling());
}

TEST_CASE("Joint: Effective angle with coupling", "[model][joint][industrial]") {
    Joint j("test", JointType::REVOLUTE, 2, 3);  // J3
    j.add_coupling(1, -1.0);  // Coupled to J2 with coefficient -1

    Eigen::VectorXd all_q(6);
    all_q << 0, M_PI/6, M_PI/4, 0, 0, 0;  // J2=30°, J3=45°

    // J3_effective = J3 + (-1) * J2 = 45° - 30° = 15°
    double q_eff = j.get_effective_angle(all_q(2), all_q);
    REQUIRE_THAT(q_eff, Catch::Matchers::WithinAbs(M_PI/12, 1e-10));
}

TEST_CASE("Joint: Effective angle with axis direction AND coupling", "[model][joint][industrial]") {
    // Fanuc style: J3 has inverted axis and coupling to J2
    Joint j("J3", JointType::REVOLUTE, 2, 3);
    j.set_axis_direction(-1);
    j.add_coupling(1, -1.0);

    Eigen::VectorXd all_q(6);
    all_q << 0, M_PI/6, M_PI/4, 0, 0, 0;  // J2=30°, J3=45°

    // Formula: q_eff = axis_direction * q_input + coef * all_q[from_joint]
    // J3_effective = -1 * 45° + (-1.0) * 30° = -45° - 30° = -75° = -5π/12
    double q_eff = j.get_effective_angle(all_q(2), all_q);
    REQUIRE_THAT(q_eff, Catch::Matchers::WithinAbs(-5.0*M_PI/12, 1e-10));
}

TEST_CASE("Joint: Multiple coupling terms", "[model][joint][industrial]") {
    // Hypothetical wrist coupling
    Joint j("J6", JointType::REVOLUTE, 5, 6);
    j.add_coupling(3, 0.5);   // Coupled to J4 with coef 0.5
    j.add_coupling(4, -0.3);  // Coupled to J5 with coef -0.3

    Eigen::VectorXd all_q(6);
    all_q << 0, 0, 0, M_PI/3, M_PI/2, M_PI/4;  // J4=60°, J5=90°, J6=45°

    // J6_effective = J6 + 0.5*J4 + (-0.3)*J5
    //              = 45° + 0.5*60° + (-0.3)*90°
    //              = 45° + 30° - 27° = 48°
    double q_eff = j.get_effective_angle(all_q(5), all_q);
    double expected = M_PI/4 + 0.5*(M_PI/3) - 0.3*(M_PI/2);
    REQUIRE_THAT(q_eff, Catch::Matchers::WithinAbs(expected, 1e-10));
}

TEST_CASE("Joint: Ignore near-zero coupling", "[model][joint][industrial]") {
    Joint j("test", JointType::REVOLUTE, 0, 1);

    j.add_coupling(1, 0.0);       // Should be ignored
    j.add_coupling(2, 1e-12);     // Should be ignored (< 1e-10)

    REQUIRE_FALSE(j.has_coupling());
}

// ============================================================================
// DH Offset Tests (calibration offsets)
// ============================================================================

TEST_CASE("DHParams: Offset in constructor", "[model][dh][industrial]") {
    DHParams dh(M_PI/2, 0.1, 0.2, 0.0, DHConvention::MODIFIED, 0.01);

    REQUIRE(dh.offset == 0.01);
}

TEST_CASE("DHParams: Transform with offset (revolute)", "[model][dh][industrial]") {
    // Modified DH with offset
    DHParams dh(0, 0.5, 0, 0, DHConvention::MODIFIED, 0.1);  // 0.1 rad offset

    // q=0, but with offset, effective q = 0.1
    SE3 T = dh.transform(0.0, false);

    // Should get rotation by 0.1 rad
    double expected_cos = std::cos(0.1);
    double expected_sin = std::sin(0.1);

    REQUIRE_THAT(T.rotation()(0, 0), Catch::Matchers::WithinAbs(expected_cos, 1e-10));
    REQUIRE_THAT(T.rotation()(1, 0), Catch::Matchers::WithinAbs(expected_sin, 1e-10));
}

TEST_CASE("DHParams: Transform with offset (prismatic)", "[model][dh][industrial]") {
    // Modified DH with prismatic offset
    DHParams dh(0, 0.1, 0, 0, DHConvention::MODIFIED, 0.05);  // 0.05m offset

    // q=0, but with offset, effective d = 0.05
    SE3 T = dh.transform(0.0, true);

    // Translation should be [0.1, 0, 0.05]
    REQUIRE_THAT(T.translation()(0), Catch::Matchers::WithinAbs(0.1, 1e-10));
    REQUIRE_THAT(T.translation()(2), Catch::Matchers::WithinAbs(0.05, 1e-10));
}

// ============================================================================
// Real Robot Configurations (from user's RoboDK data)
// ============================================================================

TEST_CASE("Industrial Robots: Fanuc axis directions and coupling", "[model][robot][fanuc]") {
    // Fanuc: [1, 1, -1, -1, -1, -1, -1]
    // Values 1-6: axis directions
    // Value 7: J2-J3 coupling = -1

    std::vector<Joint> joints;
    for (int i = 0; i < 6; ++i) {
        joints.emplace_back("J" + std::to_string(i+1), JointType::REVOLUTE, i, i+1);
    }

    // Set axis directions (Fanuc pattern)
    joints[0].set_axis_direction(1);
    joints[1].set_axis_direction(1);
    joints[2].set_axis_direction(-1);
    joints[3].set_axis_direction(-1);
    joints[4].set_axis_direction(-1);
    joints[5].set_axis_direction(-1);

    // Set J2-J3 coupling (7th value = -1)
    joints[2].add_coupling(1, -1.0);

    // Verify
    REQUIRE(joints[0].axis_direction() == 1);
    REQUIRE(joints[1].axis_direction() == 1);
    REQUIRE(joints[2].axis_direction() == -1);
    REQUIRE(joints[2].has_coupling());
    REQUIRE(joints[2].coupling_terms()[0].coefficient == -1.0);
}

TEST_CASE("Industrial Robots: KUKA axis directions, no coupling", "[model][robot][kuka]") {
    // KUKA: [-1, 1, 1, -1, 1, -1, 0]
    // Value 7 = 0 means no coupling

    std::vector<Joint> joints;
    for (int i = 0; i < 6; ++i) {
        joints.emplace_back("J" + std::to_string(i+1), JointType::REVOLUTE, i, i+1);
    }

    joints[0].set_axis_direction(-1);
    joints[1].set_axis_direction(1);
    joints[2].set_axis_direction(1);
    joints[3].set_axis_direction(-1);
    joints[4].set_axis_direction(1);
    joints[5].set_axis_direction(-1);

    // No coupling added (7th value = 0)

    REQUIRE(joints[0].axis_direction() == -1);
    REQUIRE(joints[1].axis_direction() == 1);
    REQUIRE_FALSE(joints[2].has_coupling());
}

TEST_CASE("Industrial Robots: ABB all normal, no coupling", "[model][robot][abb]") {
    // ABB: [1, 1, 1, 1, 1, 1, 1]
    // All default, no inversions, no coupling

    std::vector<Joint> joints;
    for (int i = 0; i < 6; ++i) {
        joints.emplace_back("J" + std::to_string(i+1), JointType::REVOLUTE, i, i+1);
    }

    // All default (+1), no coupling
    for (auto& j : joints) {
        REQUIRE(j.axis_direction() == 1);
        REQUIRE_FALSE(j.has_coupling());
    }
}

TEST_CASE("Industrial Robots: Comau pattern", "[model][robot][comau]") {
    // Comau: [-1, 1, 1, -1, 1, -1, 0]

    std::vector<Joint> joints;
    for (int i = 0; i < 6; ++i) {
        joints.emplace_back("J" + std::to_string(i+1), JointType::REVOLUTE, i, i+1);
    }

    joints[0].set_axis_direction(-1);
    joints[1].set_axis_direction(1);
    joints[2].set_axis_direction(1);
    joints[3].set_axis_direction(-1);
    joints[4].set_axis_direction(1);
    joints[5].set_axis_direction(-1);

    REQUIRE(joints[0].axis_direction() == -1);
    REQUIRE(joints[3].axis_direction() == -1);
    REQUIRE(joints[5].axis_direction() == -1);
    REQUIRE_FALSE(joints[2].has_coupling());
}

TEST_CASE("Industrial Robots: Universal Robots (UR) all normal", "[model][robot][ur]") {
    // UR: [1, 1, 1, 1, 1, 1, 1]

    std::vector<Joint> joints;
    for (int i = 0; i < 6; ++i) {
        joints.emplace_back("J" + std::to_string(i+1), JointType::REVOLUTE, i, i+1);
    }

    for (auto& j : joints) {
        REQUIRE(j.axis_direction() == 1);
        REQUIRE_FALSE(j.has_coupling());
    }
}

TEST_CASE("Industrial Robots: UF pattern", "[model][robot][uf]") {
    // UF: [1, 1, 1, 1, 1, 1, 0]

    std::vector<Joint> joints;
    for (int i = 0; i < 6; ++i) {
        joints.emplace_back("J" + std::to_string(i+1), JointType::REVOLUTE, i, i+1);
    }

    // All normal directions, no coupling
    for (auto& j : joints) {
        REQUIRE(j.axis_direction() == 1);
        REQUIRE_FALSE(j.has_coupling());
    }
}
