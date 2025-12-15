/**
 * FK Validation Tests
 *
 * Validates robospace FK implementation against external reference data.
 * Reference data uses deg/mm units; we convert to rad/m for comparison.
 */

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <robospace/model/robot.hpp>
#include <robospace/math/SE3.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <cmath>

using namespace robospace::model;
using namespace robospace::math;
using json = nlohmann::json;

namespace {

constexpr double DEG2RAD = M_PI / 180.0;
constexpr double MM2M = 0.001;

// Tolerances for FK validation
constexpr double POSITION_TOL_M = 0.001;    // 1 mm
constexpr double ROTATION_TOL_RAD = 0.002;  // ~0.1 degrees

/**
 * Create UR5 robot from MDH parameters.
 *
 * MDH parameters (alpha deg, a mm, theta deg, d mm):
 *   J1: 0, 0, 0, 89.2
 *   J2: 90, 0, 180, 0
 *   J3: 0, 425, 0, 0
 *   J4: 0, 392, 0, 109.3
 *   J5: -90, 0, 0, 94.75
 *   J6: 90, 0, 180, 82.5
 */
Robot create_ur5_mdh() {
    Robot robot("ur5_mdh");

    // Add links
    robot.add_link(Link("base_link"));
    robot.add_link(Link("shoulder_link"));
    robot.add_link(Link("upper_arm_link"));
    robot.add_link(Link("forearm_link"));
    robot.add_link(Link("wrist_1_link"));
    robot.add_link(Link("wrist_2_link"));
    robot.add_link(Link("wrist_3_link"));

    // Helper to create joint with MDH params
    auto make_mdh_joint = [](const std::string& name, int parent, int child,
                             double alpha_deg, double a_mm, double theta_deg, double d_mm) {
        Joint joint(name, JointType::REVOLUTE, parent, child);
        DHParams dh(
            alpha_deg * DEG2RAD,  // alpha in rad
            a_mm * MM2M,          // a in m
            d_mm * MM2M,          // d in m
            theta_deg * DEG2RAD,  // theta offset in rad
            DHConvention::MODIFIED
        );
        joint.set_dh_params(dh);
        joint.set_limits(-2*M_PI, 2*M_PI);
        return joint;
    };

    // Add joints with MDH parameters
    robot.add_joint(make_mdh_joint("shoulder_pan_joint",  0, 1,   0,   0,   0,  89.2));
    robot.add_joint(make_mdh_joint("shoulder_lift_joint", 1, 2,  90,   0, 180,   0.0));
    robot.add_joint(make_mdh_joint("elbow_joint",         2, 3,   0, 425,   0,   0.0));
    robot.add_joint(make_mdh_joint("wrist_1_joint",       3, 4,   0, 392,   0, 109.3));
    robot.add_joint(make_mdh_joint("wrist_2_joint",       4, 5, -90,   0,   0,  94.75));
    robot.add_joint(make_mdh_joint("wrist_3_joint",       5, 6,  90,   0, 180,  82.5));

    return robot;
}

struct FKTestCase {
    int id;
    std::string type;
    Eigen::VectorXd joints_rad;
    Eigen::Matrix4d pose_m;
};

struct FKReferenceData {
    std::string robot_name;
    int dof;
    std::vector<FKTestCase> test_cases;
};

FKReferenceData load_reference_data(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open FK reference file: " + filepath);
    }

    json data = json::parse(file);
    FKReferenceData ref;

    ref.robot_name = data["metadata"]["robot_name"];
    ref.dof = data["metadata"]["dof"];

    for (const auto& tc : data["test_cases"]) {
        FKTestCase test_case;
        test_case.id = tc["id"];
        test_case.type = tc["type"];

        // Convert joints from deg to rad
        std::vector<double> joints_deg = tc["joints"];
        test_case.joints_rad.resize(joints_deg.size());
        for (size_t i = 0; i < joints_deg.size(); ++i) {
            test_case.joints_rad(i) = joints_deg[i] * DEG2RAD;
        }

        // Convert pose from mm to m (4x4 matrix stored row-major)
        test_case.pose_m = Eigen::Matrix4d::Identity();
        auto pose_rows = tc["pose"];
        for (int row = 0; row < 4; ++row) {
            for (int col = 0; col < 4; ++col) {
                double val = pose_rows[row][col];
                // Convert translation columns (col 3, rows 0-2) from mm to m
                if (col == 3 && row < 3) {
                    val *= MM2M;
                }
                test_case.pose_m(row, col) = val;
            }
        }

        ref.test_cases.push_back(test_case);
    }

    return ref;
}

double rotation_error(const Eigen::Matrix3d& R1, const Eigen::Matrix3d& R2) {
    // Compute rotation error as angle of R1^T * R2
    Eigen::Matrix3d R_diff = R1.transpose() * R2;
    // angle = acos((trace(R) - 1) / 2), clamped for numerical stability
    double trace = R_diff.trace();
    double cos_angle = (trace - 1.0) / 2.0;
    cos_angle = std::max(-1.0, std::min(1.0, cos_angle));
    return std::acos(cos_angle);
}

/**
 * Fanuc M-20iA MDH parameters (alpha deg, a mm, theta deg, d mm):
 *   J1: 0, 0, 0, 525  |  J2: -90, 150, -90, 0  |  J3: 0, 790, 0, 0
 *   J4: -90, 250, 0, 835  |  J5: 90, 0, 0, 0  |  J6: -90, 0, 0, 100
 * Joint senses: 1, 1, -1, -1, -1, -1 | J2-J3 coupling | Base at z=-525mm
 */
Robot create_fanuc_m20ia_mdh() {
    Robot robot("fanuc_m20ia_mdh");

    robot.add_link(Link("base_link"));
    for (int i = 1; i <= 6; ++i) robot.add_link(Link("link" + std::to_string(i)));

    auto make_mdh_joint = [](const std::string& name, int parent, int child,
                             double alpha_deg, double a_mm, double theta_deg, double d_mm) {
        Joint joint(name, JointType::REVOLUTE, parent, child);
        DHParams dh(alpha_deg * DEG2RAD, a_mm * MM2M, d_mm * MM2M,
                    theta_deg * DEG2RAD, DHConvention::MODIFIED);
        joint.set_dh_params(dh);
        joint.set_limits(-2*M_PI, 2*M_PI);
        return joint;
    };

    Joint j1 = make_mdh_joint("J1", 0, 1,    0,   0,   0, 525);
    Joint j2 = make_mdh_joint("J2", 1, 2,  -90, 150, -90,   0);
    Joint j3 = make_mdh_joint("J3", 2, 3,    0, 790,   0,   0);
    Joint j4 = make_mdh_joint("J4", 3, 4,  -90, 250,   0, 835);
    Joint j5 = make_mdh_joint("J5", 4, 5,   90,   0,   0,   0);
    Joint j6 = make_mdh_joint("J6", 5, 6,  -90,   0,   0, 100);

    // Joint senses: 1, 1, -1, -1, -1, -1
    j3.set_axis_direction(-1);
    j4.set_axis_direction(-1);
    j5.set_axis_direction(-1);
    j6.set_axis_direction(-1);

    // J2-J3 coupling: q3_eff = -q3 - q2
    j3.add_coupling(1, -1.0);

    robot.add_joint(std::move(j1));
    robot.add_joint(std::move(j2));
    robot.add_joint(std::move(j3));
    robot.add_joint(std::move(j4));
    robot.add_joint(std::move(j5));
    robot.add_joint(std::move(j6));

    // Base frame at z=-525mm (reference frame is at J1 height)
    robot.set_base(SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, 0, -0.525)));

    return robot;
}

/**
 * KUKA KR 10 R1420 MDH parameters (alpha deg, a mm, theta deg, d mm):
 *   J1: 0, 0, 0, 450  |  J2: -90, 150, 0, 0  |  J3: 0, 610, -90, 0
 *   J4: -90, 20, 0, 660  |  J5: 90, 0, 0, 0  |  J6: -90, 0, 180, 80
 * Joint senses: -1, 1, 1, -1, 1, -1 | No coupling
 */
Robot create_kuka_kr10_r1420_mdh() {
    Robot robot("kuka_kr10_r1420_mdh");

    robot.add_link(Link("base_link"));
    for (int i = 1; i <= 6; ++i) robot.add_link(Link("link" + std::to_string(i)));

    auto make_mdh_joint = [](const std::string& name, int parent, int child,
                             double alpha_deg, double a_mm, double theta_deg, double d_mm) {
        Joint joint(name, JointType::REVOLUTE, parent, child);
        DHParams dh(alpha_deg * DEG2RAD, a_mm * MM2M, d_mm * MM2M,
                    theta_deg * DEG2RAD, DHConvention::MODIFIED);
        joint.set_dh_params(dh);
        joint.set_limits(-2*M_PI, 2*M_PI);
        return joint;
    };

    Joint j1 = make_mdh_joint("J1", 0, 1,    0,   0,   0, 450);
    Joint j2 = make_mdh_joint("J2", 1, 2,  -90, 150,   0,   0);
    Joint j3 = make_mdh_joint("J3", 2, 3,    0, 610, -90,   0);
    Joint j4 = make_mdh_joint("J4", 3, 4,  -90,  20,   0, 660);
    Joint j5 = make_mdh_joint("J5", 4, 5,   90,   0,   0,   0);
    Joint j6 = make_mdh_joint("J6", 5, 6,  -90,   0, 180,  80);

    // Joint senses: -1, 1, 1, -1, 1, -1
    j1.set_axis_direction(-1);
    j4.set_axis_direction(-1);
    j6.set_axis_direction(-1);

    robot.add_joint(std::move(j1));
    robot.add_joint(std::move(j2));
    robot.add_joint(std::move(j3));
    robot.add_joint(std::move(j4));
    robot.add_joint(std::move(j5));
    robot.add_joint(std::move(j6));

    return robot;
}

/**
 * ABB IRB 1200-5/0.9 MDH parameters (alpha deg, a mm, theta deg, d mm):
 *   J1: 0, 0, 0, 399  |  J2: -90, 0, -90, 0  |  J3: 0, 448, 0, 0
 *   J4: -90, 42, 0, 451  |  J5: 90, 0, 0, 0  |  J6: -90, 0, 180, 82
 * Joint senses: all positive | J2-J3 coupling
 */
Robot create_abb_irb1200_mdh() {
    Robot robot("abb_irb1200_mdh");

    // Add links
    robot.add_link(Link("base_link"));
    robot.add_link(Link("link1"));
    robot.add_link(Link("link2"));
    robot.add_link(Link("link3"));
    robot.add_link(Link("link4"));
    robot.add_link(Link("link5"));
    robot.add_link(Link("link6"));

    // Helper to create joint with MDH params
    auto make_mdh_joint = [](const std::string& name, int parent, int child,
                             double alpha_deg, double a_mm, double theta_deg, double d_mm) {
        Joint joint(name, JointType::REVOLUTE, parent, child);
        DHParams dh(
            alpha_deg * DEG2RAD,  // alpha in rad
            a_mm * MM2M,          // a in m
            d_mm * MM2M,          // d in m
            theta_deg * DEG2RAD,  // theta offset in rad
            DHConvention::MODIFIED
        );
        joint.set_dh_params(dh);
        joint.set_limits(-2*M_PI, 2*M_PI);
        return joint;
    };

    // Create joints with MDH parameters
    // Joint senses all 1 (positive Z), no axis direction changes needed
    Joint j1 = make_mdh_joint("J1", 0, 1,    0,   0,   0, 399);
    Joint j2 = make_mdh_joint("J2", 1, 2,  -90,   0, -90,   0);
    Joint j3 = make_mdh_joint("J3", 2, 3,    0, 448,   0,   0);
    Joint j4 = make_mdh_joint("J4", 3, 4,  -90,  42,   0, 451);
    Joint j5 = make_mdh_joint("J5", 4, 5,   90,   0,   0,   0);
    Joint j6 = make_mdh_joint("J6", 5, 6,  -90,   0, 180,  82);

    // ABB IRB 1200 has J2-J3 coupling (parallelogram linkage)
    // q3_actual = q3_controller - q2_controller
    j3.add_coupling(1, -1.0);

    robot.add_joint(std::move(j1));
    robot.add_joint(std::move(j2));
    robot.add_joint(std::move(j3));
    robot.add_joint(std::move(j4));
    robot.add_joint(std::move(j5));
    robot.add_joint(std::move(j6));

    return robot;
}

/**
 * Create ABB IRB 6700-155/2.85 robot from MDH parameters.
 *
 * MDH parameters (alpha deg, a mm, theta deg, d mm):
 *   J1: 0, 0, 0, 780
 *   J2: -90, 320, -90, 0
 *   J3: 0, 1125, 0, 0
 *   J4: -90, 200, 0, 1392
 *   J5: 90, 0, 0, 0
 *   J6: -90, 0, 180, 200
 *
 * Special considerations:
 * - Joint senses: 1, 1, 1, 1, 1, 1 (all positive Z rotation)
 * - No J2-J3 coupling (unlike IRB 1200) - TODO: verify this
 */
Robot create_abb_irb6700_mdh() {
    Robot robot("abb_irb6700_mdh");

    // Add links
    robot.add_link(Link("base_link"));
    robot.add_link(Link("link1"));
    robot.add_link(Link("link2"));
    robot.add_link(Link("link3"));
    robot.add_link(Link("link4"));
    robot.add_link(Link("link5"));
    robot.add_link(Link("link6"));

    // Helper to create joint with MDH params
    auto make_mdh_joint = [](const std::string& name, int parent, int child,
                             double alpha_deg, double a_mm, double theta_deg, double d_mm) {
        Joint joint(name, JointType::REVOLUTE, parent, child);
        DHParams dh(
            alpha_deg * DEG2RAD,  // alpha in rad
            a_mm * MM2M,          // a in m
            d_mm * MM2M,          // d in m
            theta_deg * DEG2RAD,  // theta offset in rad
            DHConvention::MODIFIED
        );
        joint.set_dh_params(dh);
        joint.set_limits(-2*M_PI, 2*M_PI);
        return joint;
    };

    // Create joints with MDH parameters
    // Joint senses all 1 (positive Z), no axis direction changes needed
    // No J2-J3 coupling for IRB 6700
    robot.add_joint(make_mdh_joint("J1", 0, 1,    0,   0,   0,  780));
    robot.add_joint(make_mdh_joint("J2", 1, 2,  -90, 320, -90,    0));
    robot.add_joint(make_mdh_joint("J3", 2, 3,    0,1125,   0,    0));
    robot.add_joint(make_mdh_joint("J4", 3, 4,  -90, 200,   0, 1392));
    robot.add_joint(make_mdh_joint("J5", 4, 5,   90,   0,   0,    0));
    robot.add_joint(make_mdh_joint("J6", 5, 6,  -90,   0, 180,  200));

    return robot;
}

/**
 * Yaskawa GP12 MDH parameters (alpha deg, a mm, theta deg, d mm):
 *   J1: 0, 0, 0, 450  |  J2: -90, 155, -90, 0  |  J3: 0, 614, 0, 0
 *   J4: -90, 200, 0, 640  |  J5: 90, 0, 0, 0  |  J6: -90, 0, 0, 100
 * Joint senses: 1, 1, -1, -1, -1, -1 (J3-J6 negative axis_direction)
 */
Robot create_yaskawa_gp12_mdh() {
    Robot robot("yaskawa_gp12_mdh");

    robot.add_link(Link("base_link"));
    for (int i = 1; i <= 6; ++i) robot.add_link(Link("link" + std::to_string(i)));

    auto make_mdh_joint = [](const std::string& name, int parent, int child,
                             double alpha_deg, double a_mm, double theta_deg, double d_mm) {
        Joint joint(name, JointType::REVOLUTE, parent, child);
        DHParams dh(alpha_deg * DEG2RAD, a_mm * MM2M, d_mm * MM2M,
                    theta_deg * DEG2RAD, DHConvention::MODIFIED);
        joint.set_dh_params(dh);
        joint.set_limits(-2*M_PI, 2*M_PI);
        return joint;
    };

    Joint j1 = make_mdh_joint("J1", 0, 1,    0,   0,   0, 450);
    Joint j2 = make_mdh_joint("J2", 1, 2,  -90, 155, -90,   0);
    Joint j3 = make_mdh_joint("J3", 2, 3,    0, 614,   0,   0);
    Joint j4 = make_mdh_joint("J4", 3, 4,  -90, 200,   0, 640);
    Joint j5 = make_mdh_joint("J5", 4, 5,   90,   0,   0,   0);
    Joint j6 = make_mdh_joint("J6", 5, 6,  -90,   0,   0, 100);

    // Joint senses: 1, 1, -1, -1, -1, -1
    j3.set_axis_direction(-1);
    j4.set_axis_direction(-1);
    j5.set_axis_direction(-1);
    j6.set_axis_direction(-1);

    robot.add_joint(std::move(j1));
    robot.add_joint(std::move(j2));
    robot.add_joint(std::move(j3));
    robot.add_joint(std::move(j4));
    robot.add_joint(std::move(j5));
    robot.add_joint(std::move(j6));

    // Base frame at z=-450mm (reference frame is at J1 height)
    robot.set_base(SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, 0, -0.450)));

    return robot;
}

}  // namespace

TEST_CASE("FK Validation: UR5 vs external reference", "[fk][validation][ur5]") {
    // Create UR5 from MDH parameters
    Robot robot = create_ur5_mdh();
    REQUIRE(robot.dof() == 6);

    // Load reference data
    FKReferenceData ref;
    REQUIRE_NOTHROW(ref = load_reference_data("test_data/fk_reference/ur5_fk_reference.json"));
    REQUIRE(ref.dof == 6);
    REQUIRE(ref.test_cases.size() > 0);

    INFO("Testing " << ref.test_cases.size() << " FK cases for " << ref.robot_name);

    int passed = 0;
    int failed = 0;
    double max_pos_error = 0.0;
    double max_rot_error = 0.0;

    for (const auto& tc : ref.test_cases) {
        // Compute FK with robospace
        SE3 T_computed = robot.fk(tc.joints_rad);

        // Extract reference pose
        Eigen::Vector3d pos_ref = tc.pose_m.block<3, 1>(0, 3);
        Eigen::Matrix3d rot_ref = tc.pose_m.block<3, 3>(0, 0);

        // Compute errors
        Eigen::Vector3d pos_computed = T_computed.translation();
        Eigen::Matrix3d rot_computed = T_computed.rotation();

        double pos_error = (pos_computed - pos_ref).norm();
        double rot_error = rotation_error(rot_computed, rot_ref);

        max_pos_error = std::max(max_pos_error, pos_error);
        max_rot_error = std::max(max_rot_error, rot_error);

        bool pos_ok = pos_error < POSITION_TOL_M;
        bool rot_ok = rot_error < ROTATION_TOL_RAD;

        if (pos_ok && rot_ok) {
            passed++;
        } else {
            failed++;
            if (failed <= 3) {  // Show first 3 failures with detailed info
                WARN("Test case " << tc.id << " (" << tc.type << ") FAILED:"
                     << " pos_err=" << (pos_error * 1000) << "mm"
                     << " rot_err=" << (rot_error / DEG2RAD) << "deg");
            }
        }
    }

    INFO("Results: " << passed << " passed, " << failed << " failed");
    INFO("Max position error: " << (max_pos_error * 1000) << " mm");
    INFO("Max rotation error: " << (max_rot_error / DEG2RAD) << " deg");

    // Require high pass rate
    double pass_rate = static_cast<double>(passed) / ref.test_cases.size();
    REQUIRE(pass_rate >= 0.99);  // At least 99% must pass
}

TEST_CASE("FK Validation: UR5 zero config", "[fk][validation][ur5]") {
    Robot robot = create_ur5_mdh();
    REQUIRE(robot.dof() == 6);

    Eigen::VectorXd q = Eigen::VectorXd::Zero(6);
    SE3 T = robot.fk(q);
    Eigen::Vector3d pos = T.translation();
    Eigen::Matrix3d rot = T.rotation();

    INFO("FK at zero config (UR5 MDH):");
    INFO("  Position (mm): [" << (pos.x()*1000) << ", " << (pos.y()*1000) << ", " << (pos.z()*1000) << "]");
    INFO("  Rotation:");
    INFO("    [" << rot(0,0) << ", " << rot(0,1) << ", " << rot(0,2) << "]");
    INFO("    [" << rot(1,0) << ", " << rot(1,1) << ", " << rot(1,2) << "]");
    INFO("    [" << rot(2,0) << ", " << rot(2,1) << ", " << rot(2,2) << "]");

    // Expected from external reference (after correcting for station offset):
    // x = -817mm, y = -191.8mm, z = -5.5mm
    REQUIRE_THAT(pos.x(), Catch::Matchers::WithinAbs(-0.817, 0.001));
    REQUIRE_THAT(pos.y(), Catch::Matchers::WithinAbs(-0.1918, 0.001));
    REQUIRE_THAT(pos.z(), Catch::Matchers::WithinAbs(-0.00555, 0.001));

    // EE should be ~0.84m from base
    REQUIRE(pos.norm() > 0.5);
    REQUIRE(pos.norm() < 1.5);
}

TEST_CASE("FK Validation: Fanuc M-20iA vs external reference", "[fk][validation][fanuc]") {
    // Create Fanuc M-20iA from MDH parameters
    // Robot model handles axis direction, coupling, and reference frame internally
    Robot robot = create_fanuc_m20ia_mdh();
    REQUIRE(robot.dof() == 6);

    // Load reference data
    FKReferenceData ref;
    REQUIRE_NOTHROW(ref = load_reference_data("test_data/fk_reference/fanuc_m_20ia_fk_reference.json"));
    REQUIRE(ref.dof == 6);
    REQUIRE(ref.test_cases.size() > 0);

    INFO("Testing " << ref.test_cases.size() << " FK cases for " << ref.robot_name);

    int passed = 0;
    int failed = 0;
    double max_pos_error = 0.0;
    double max_rot_error = 0.0;

    for (const auto& tc : ref.test_cases) {
        // Pass controller joints directly - robot model handles transformations
        SE3 T_computed = robot.fk(tc.joints_rad);

        // Extract reference pose
        Eigen::Vector3d pos_ref = tc.pose_m.block<3, 1>(0, 3);
        Eigen::Matrix3d rot_ref = tc.pose_m.block<3, 3>(0, 0);

        // Compute errors
        Eigen::Vector3d pos_computed = T_computed.translation();
        Eigen::Matrix3d rot_computed = T_computed.rotation();

        double pos_error = (pos_computed - pos_ref).norm();
        double rot_error = rotation_error(rot_computed, rot_ref);

        max_pos_error = std::max(max_pos_error, pos_error);
        max_rot_error = std::max(max_rot_error, rot_error);

        bool pos_ok = pos_error < POSITION_TOL_M;
        bool rot_ok = rot_error < ROTATION_TOL_RAD;

        if (pos_ok && rot_ok) {
            passed++;
        } else {
            failed++;
            if (failed <= 5) {  // Show first 5 failures with detailed info
                WARN("Test case " << tc.id << " (" << tc.type << ") FAILED:"
                     << " pos_err=" << (pos_error * 1000) << "mm"
                     << " rot_err=" << (rot_error / DEG2RAD) << "deg");
                WARN("  Controller joints (deg): ["
                     << tc.joints_rad(0)/DEG2RAD << ", " << tc.joints_rad(1)/DEG2RAD << ", "
                     << tc.joints_rad(2)/DEG2RAD << ", " << tc.joints_rad(3)/DEG2RAD << ", "
                     << tc.joints_rad(4)/DEG2RAD << ", " << tc.joints_rad(5)/DEG2RAD << "]");
                WARN("  Expected pos (mm): [" << pos_ref.x()*1000 << ", " << pos_ref.y()*1000 << ", " << pos_ref.z()*1000 << "]");
                WARN("  Computed pos (mm): [" << pos_computed.x()*1000 << ", " << pos_computed.y()*1000 << ", " << pos_computed.z()*1000 << "]");
            }
        }
    }

    INFO("Results: " << passed << " passed, " << failed << " failed");
    INFO("Max position error: " << (max_pos_error * 1000) << " mm");
    INFO("Max rotation error: " << (max_rot_error / DEG2RAD) << " deg");

    // Require high pass rate
    double pass_rate = static_cast<double>(passed) / ref.test_cases.size();
    REQUIRE(pass_rate >= 0.99);  // At least 99% must pass
}

TEST_CASE("FK Validation: Fanuc M-20iA zero config", "[fk][validation][fanuc]") {
    Robot robot = create_fanuc_m20ia_mdh();
    REQUIRE(robot.dof() == 6);

    // Zero config from controller perspective
    Eigen::VectorXd q = Eigen::VectorXd::Zero(6);
    SE3 T = robot.fk(q);
    Eigen::Vector3d pos = T.translation();
    Eigen::Matrix3d rot = T.rotation();

    INFO("FK at zero config (Fanuc M-20iA MDH):");
    INFO("  Controller joints (deg): [0, 0, 0, 0, 0, 0]");
    INFO("  Position (mm): [" << (pos.x()*1000) << ", " << (pos.y()*1000) << ", " << (pos.z()*1000) << "]");
    INFO("  Rotation:");
    INFO("    [" << rot(0,0) << ", " << rot(0,1) << ", " << rot(0,2) << "]");
    INFO("    [" << rot(1,0) << ", " << rot(1,1) << ", " << rot(1,2) << "]");
    INFO("    [" << rot(2,0) << ", " << rot(2,1) << ", " << rot(2,2) << "]");

    // Expected from external reference at zero config:
    // x = 1085mm, y ≈ 0, z = 1040mm
    REQUIRE_THAT(pos.x(), Catch::Matchers::WithinAbs(1.085, 0.01));
    REQUIRE_THAT(pos.y(), Catch::Matchers::WithinAbs(0.0, 0.01));
    REQUIRE_THAT(pos.z(), Catch::Matchers::WithinAbs(1.040, 0.01));
}

TEST_CASE("FK Validation: KUKA KR 10 R1420 vs external reference", "[fk][validation][kuka]") {
    // Create KUKA KR 10 R1420 from MDH parameters
    Robot robot = create_kuka_kr10_r1420_mdh();
    REQUIRE(robot.dof() == 6);

    // Load reference data
    FKReferenceData ref;
    REQUIRE_NOTHROW(ref = load_reference_data("test_data/fk_reference/kuka_kr_10_r1420_fk_reference.json"));
    REQUIRE(ref.dof == 6);
    REQUIRE(ref.test_cases.size() > 0);

    INFO("Testing " << ref.test_cases.size() << " FK cases for " << ref.robot_name);

    int passed = 0;
    int failed = 0;
    double max_pos_error = 0.0;
    double max_rot_error = 0.0;

    for (const auto& tc : ref.test_cases) {
        // Pass controller joints directly - robot model handles transformations
        SE3 T_computed = robot.fk(tc.joints_rad);

        // Extract reference pose
        Eigen::Vector3d pos_ref = tc.pose_m.block<3, 1>(0, 3);
        Eigen::Matrix3d rot_ref = tc.pose_m.block<3, 3>(0, 0);

        // Compute errors
        Eigen::Vector3d pos_computed = T_computed.translation();
        Eigen::Matrix3d rot_computed = T_computed.rotation();

        double pos_error = (pos_computed - pos_ref).norm();
        double rot_error = rotation_error(rot_computed, rot_ref);

        max_pos_error = std::max(max_pos_error, pos_error);
        max_rot_error = std::max(max_rot_error, rot_error);

        bool pos_ok = pos_error < POSITION_TOL_M;
        bool rot_ok = rot_error < ROTATION_TOL_RAD;

        if (pos_ok && rot_ok) {
            passed++;
        } else {
            failed++;
            if (failed <= 5) {  // Show first 5 failures with detailed info
                WARN("Test case " << tc.id << " (" << tc.type << ") FAILED:"
                     << " pos_err=" << (pos_error * 1000) << "mm"
                     << " rot_err=" << (rot_error / DEG2RAD) << "deg");
                WARN("  Controller joints (deg): ["
                     << tc.joints_rad(0)/DEG2RAD << ", " << tc.joints_rad(1)/DEG2RAD << ", "
                     << tc.joints_rad(2)/DEG2RAD << ", " << tc.joints_rad(3)/DEG2RAD << ", "
                     << tc.joints_rad(4)/DEG2RAD << ", " << tc.joints_rad(5)/DEG2RAD << "]");
                WARN("  Expected pos (mm): [" << pos_ref.x()*1000 << ", " << pos_ref.y()*1000 << ", " << pos_ref.z()*1000 << "]");
                WARN("  Computed pos (mm): [" << pos_computed.x()*1000 << ", " << pos_computed.y()*1000 << ", " << pos_computed.z()*1000 << "]");
            }
        }
    }

    INFO("Results: " << passed << " passed, " << failed << " failed");
    INFO("Max position error: " << (max_pos_error * 1000) << " mm");
    INFO("Max rotation error: " << (max_rot_error / DEG2RAD) << " deg");

    // Require high pass rate
    double pass_rate = static_cast<double>(passed) / ref.test_cases.size();
    REQUIRE(pass_rate >= 0.99);  // At least 99% must pass
}

TEST_CASE("FK Validation: KUKA KR 10 R1420 zero config", "[fk][validation][kuka]") {
    Robot robot = create_kuka_kr10_r1420_mdh();
    REQUIRE(robot.dof() == 6);

    // Zero config from controller perspective
    Eigen::VectorXd q = Eigen::VectorXd::Zero(6);
    SE3 T = robot.fk(q);
    Eigen::Vector3d pos = T.translation();
    Eigen::Matrix3d rot = T.rotation();

    INFO("FK at zero config (KUKA KR 10 R1420 MDH):");
    INFO("  Controller joints (deg): [0, 0, 0, 0, 0, 0]");
    INFO("  Position (mm): [" << (pos.x()*1000) << ", " << (pos.y()*1000) << ", " << (pos.z()*1000) << "]");
    INFO("  Rotation:");
    INFO("    [" << rot(0,0) << ", " << rot(0,1) << ", " << rot(0,2) << "]");
    INFO("    [" << rot(1,0) << ", " << rot(1,1) << ", " << rot(1,2) << "]");
    INFO("    [" << rot(2,0) << ", " << rot(2,1) << ", " << rot(2,2) << "]");

    // Expected from external reference at zero config:
    // x = 1500mm, y = 0, z = 470mm
    REQUIRE_THAT(pos.x(), Catch::Matchers::WithinAbs(1.500, 0.01));
    REQUIRE_THAT(pos.y(), Catch::Matchers::WithinAbs(0.0, 0.01));
    REQUIRE_THAT(pos.z(), Catch::Matchers::WithinAbs(0.470, 0.01));
}

TEST_CASE("FK Validation: ABB IRB 1200 vs external reference", "[fk][validation][abb]") {
    // Create ABB IRB 1200-5/0.9 from MDH parameters
    Robot robot = create_abb_irb1200_mdh();
    REQUIRE(robot.dof() == 6);

    // Load reference data
    FKReferenceData ref;
    REQUIRE_NOTHROW(ref = load_reference_data("test_data/fk_reference/abb_irb_1200_fk_reference.json"));
    REQUIRE(ref.dof == 6);
    REQUIRE(ref.test_cases.size() > 0);

    INFO("Testing " << ref.test_cases.size() << " FK cases for " << ref.robot_name);

    int passed = 0;
    int failed = 0;
    double max_pos_error = 0.0;
    double max_rot_error = 0.0;

    for (const auto& tc : ref.test_cases) {
        // Pass controller joints directly - robot model handles transformations
        SE3 T_computed = robot.fk(tc.joints_rad);

        // Extract reference pose
        Eigen::Vector3d pos_ref = tc.pose_m.block<3, 1>(0, 3);
        Eigen::Matrix3d rot_ref = tc.pose_m.block<3, 3>(0, 0);

        // Compute errors
        Eigen::Vector3d pos_computed = T_computed.translation();
        Eigen::Matrix3d rot_computed = T_computed.rotation();

        double pos_error = (pos_computed - pos_ref).norm();
        double rot_error = rotation_error(rot_computed, rot_ref);

        max_pos_error = std::max(max_pos_error, pos_error);
        max_rot_error = std::max(max_rot_error, rot_error);

        bool pos_ok = pos_error < POSITION_TOL_M;
        bool rot_ok = rot_error < ROTATION_TOL_RAD;

        if (pos_ok && rot_ok) {
            passed++;
        } else {
            failed++;
            if (failed <= 5) {  // Show first 5 failures with detailed info
                WARN("Test case " << tc.id << " (" << tc.type << ") FAILED:"
                     << " pos_err=" << (pos_error * 1000) << "mm"
                     << " rot_err=" << (rot_error / DEG2RAD) << "deg");
                WARN("  Controller joints (deg): ["
                     << tc.joints_rad(0)/DEG2RAD << ", " << tc.joints_rad(1)/DEG2RAD << ", "
                     << tc.joints_rad(2)/DEG2RAD << ", " << tc.joints_rad(3)/DEG2RAD << ", "
                     << tc.joints_rad(4)/DEG2RAD << ", " << tc.joints_rad(5)/DEG2RAD << "]");
                WARN("  Expected pos (mm): [" << pos_ref.x()*1000 << ", " << pos_ref.y()*1000 << ", " << pos_ref.z()*1000 << "]");
                WARN("  Computed pos (mm): [" << pos_computed.x()*1000 << ", " << pos_computed.y()*1000 << ", " << pos_computed.z()*1000 << "]");
            }
        }
    }

    INFO("Results: " << passed << " passed, " << failed << " failed");
    INFO("Max position error: " << (max_pos_error * 1000) << " mm");
    INFO("Max rotation error: " << (max_rot_error / DEG2RAD) << " deg");

    // Require high pass rate
    double pass_rate = static_cast<double>(passed) / ref.test_cases.size();
    REQUIRE(pass_rate >= 0.99);  // At least 99% must pass
}

TEST_CASE("FK Validation: ABB IRB 1200 zero config", "[fk][validation][abb]") {
    Robot robot = create_abb_irb1200_mdh();
    REQUIRE(robot.dof() == 6);

    // Zero config from controller perspective
    Eigen::VectorXd q = Eigen::VectorXd::Zero(6);
    SE3 T = robot.fk(q);
    Eigen::Vector3d pos = T.translation();
    Eigen::Matrix3d rot = T.rotation();

    INFO("FK at zero config (ABB IRB 1200 MDH):");
    INFO("  Controller joints (deg): [0, 0, 0, 0, 0, 0]");
    INFO("  Position (mm): [" << (pos.x()*1000) << ", " << (pos.y()*1000) << ", " << (pos.z()*1000) << "]");
    INFO("  Rotation:");
    INFO("    [" << rot(0,0) << ", " << rot(0,1) << ", " << rot(0,2) << "]");
    INFO("    [" << rot(1,0) << ", " << rot(1,1) << ", " << rot(1,2) << "]");
    INFO("    [" << rot(2,0) << ", " << rot(2,1) << ", " << rot(2,2) << "]");

    // Expected from external reference at zero config:
    // x = 533mm, y = 0, z = 889mm
    REQUIRE_THAT(pos.x(), Catch::Matchers::WithinAbs(0.533, 0.01));
    REQUIRE_THAT(pos.y(), Catch::Matchers::WithinAbs(0.0, 0.01));
    REQUIRE_THAT(pos.z(), Catch::Matchers::WithinAbs(0.889, 0.01));
}

TEST_CASE("FK Validation: ABB IRB 6700 vs external reference", "[fk][validation][abb][abb6700]") {
    // Create ABB IRB 6700-155/2.85 from MDH parameters
    Robot robot = create_abb_irb6700_mdh();
    REQUIRE(robot.dof() == 6);

    // Load reference data
    FKReferenceData ref;
    REQUIRE_NOTHROW(ref = load_reference_data("test_data/fk_reference/abb_irb_6700_fk_reference.json"));
    REQUIRE(ref.dof == 6);
    REQUIRE(ref.test_cases.size() > 0);

    INFO("Testing " << ref.test_cases.size() << " FK cases for " << ref.robot_name);

    int passed = 0;
    int failed = 0;
    double max_pos_error = 0.0;
    double max_rot_error = 0.0;

    for (const auto& tc : ref.test_cases) {
        // Pass controller joints directly - robot model handles transformations
        SE3 T_computed = robot.fk(tc.joints_rad);

        // Extract reference pose
        Eigen::Vector3d pos_ref = tc.pose_m.block<3, 1>(0, 3);
        Eigen::Matrix3d rot_ref = tc.pose_m.block<3, 3>(0, 0);

        // Compute errors
        Eigen::Vector3d pos_computed = T_computed.translation();
        Eigen::Matrix3d rot_computed = T_computed.rotation();

        double pos_error = (pos_computed - pos_ref).norm();
        double rot_error = rotation_error(rot_computed, rot_ref);

        max_pos_error = std::max(max_pos_error, pos_error);
        max_rot_error = std::max(max_rot_error, rot_error);

        bool pos_ok = pos_error < POSITION_TOL_M;
        bool rot_ok = rot_error < ROTATION_TOL_RAD;

        if (pos_ok && rot_ok) {
            passed++;
        } else {
            failed++;
            if (failed <= 5) {  // Show first 5 failures with detailed info
                WARN("Test case " << tc.id << " (" << tc.type << ") FAILED:"
                     << " pos_err=" << (pos_error * 1000) << "mm"
                     << " rot_err=" << (rot_error / DEG2RAD) << "deg");
                WARN("  Controller joints (deg): ["
                     << tc.joints_rad(0)/DEG2RAD << ", " << tc.joints_rad(1)/DEG2RAD << ", "
                     << tc.joints_rad(2)/DEG2RAD << ", " << tc.joints_rad(3)/DEG2RAD << ", "
                     << tc.joints_rad(4)/DEG2RAD << ", " << tc.joints_rad(5)/DEG2RAD << "]");
                WARN("  Expected pos (mm): [" << pos_ref.x()*1000 << ", " << pos_ref.y()*1000 << ", " << pos_ref.z()*1000 << "]");
                WARN("  Computed pos (mm): [" << pos_computed.x()*1000 << ", " << pos_computed.y()*1000 << ", " << pos_computed.z()*1000 << "]");
            }
        }
    }

    INFO("Results: " << passed << " passed, " << failed << " failed");
    INFO("Max position error: " << (max_pos_error * 1000) << " mm");
    INFO("Max rotation error: " << (max_rot_error / DEG2RAD) << " deg");

    // Require high pass rate
    double pass_rate = static_cast<double>(passed) / ref.test_cases.size();
    REQUIRE(pass_rate >= 0.99);  // At least 99% must pass
}

TEST_CASE("FK Validation: ABB IRB 6700 zero config", "[fk][validation][abb][abb6700]") {
    Robot robot = create_abb_irb6700_mdh();
    REQUIRE(robot.dof() == 6);

    // Zero config from controller perspective
    Eigen::VectorXd q = Eigen::VectorXd::Zero(6);
    SE3 T = robot.fk(q);
    Eigen::Vector3d pos = T.translation();
    Eigen::Matrix3d rot = T.rotation();

    INFO("FK at zero config (ABB IRB 6700 MDH):");
    INFO("  Controller joints (deg): [0, 0, 0, 0, 0, 0]");
    INFO("  Position (mm): [" << (pos.x()*1000) << ", " << (pos.y()*1000) << ", " << (pos.z()*1000) << "]");
    INFO("  Rotation:");
    INFO("    [" << rot(0,0) << ", " << rot(0,1) << ", " << rot(0,2) << "]");
    INFO("    [" << rot(1,0) << ", " << rot(1,1) << ", " << rot(1,2) << "]");
    INFO("    [" << rot(2,0) << ", " << rot(2,1) << ", " << rot(2,2) << "]");

    // Expected from external reference at zero config:
    // x = 1912.5mm, y = 0, z = 2105mm
    REQUIRE_THAT(pos.x(), Catch::Matchers::WithinAbs(1.9125, 0.01));
    REQUIRE_THAT(pos.y(), Catch::Matchers::WithinAbs(0.0, 0.01));
    REQUIRE_THAT(pos.z(), Catch::Matchers::WithinAbs(2.105, 0.01));
}

TEST_CASE("FK Validation: Yaskawa GP12 vs external reference", "[fk][validation][yaskawa]") {
    Robot robot = create_yaskawa_gp12_mdh();
    REQUIRE(robot.dof() == 6);

    FKReferenceData ref;
    REQUIRE_NOTHROW(ref = load_reference_data("test_data/fk_reference/yaskawa_gp12_fk_reference.json"));
    REQUIRE(ref.dof == 6);

    INFO("Testing " << ref.test_cases.size() << " FK cases for " << ref.robot_name);

    int passed = 0, failed = 0;
    double max_pos_error = 0.0, max_rot_error = 0.0;

    for (const auto& tc : ref.test_cases) {
        SE3 T_computed = robot.fk(tc.joints_rad);
        Eigen::Vector3d pos_ref = tc.pose_m.block<3, 1>(0, 3);
        Eigen::Matrix3d rot_ref = tc.pose_m.block<3, 3>(0, 0);
        Eigen::Vector3d pos_computed = T_computed.translation();
        Eigen::Matrix3d rot_computed = T_computed.rotation();

        double pos_error = (pos_computed - pos_ref).norm();
        double rot_error = rotation_error(rot_computed, rot_ref);
        max_pos_error = std::max(max_pos_error, pos_error);
        max_rot_error = std::max(max_rot_error, rot_error);

        if (pos_error < POSITION_TOL_M && rot_error < ROTATION_TOL_RAD) {
            passed++;
        } else {
            failed++;
            if (failed <= 5) {
                WARN("Test case " << tc.id << " (" << tc.type << ") FAILED:"
                     << " pos_err=" << (pos_error * 1000) << "mm"
                     << " rot_err=" << (rot_error / DEG2RAD) << "deg");
                WARN("  Joints (deg): [" << tc.joints_rad(0)/DEG2RAD << ", " << tc.joints_rad(1)/DEG2RAD
                     << ", " << tc.joints_rad(2)/DEG2RAD << ", " << tc.joints_rad(3)/DEG2RAD
                     << ", " << tc.joints_rad(4)/DEG2RAD << ", " << tc.joints_rad(5)/DEG2RAD << "]");
                WARN("  Expected (mm): [" << pos_ref.x()*1000 << ", " << pos_ref.y()*1000 << ", " << pos_ref.z()*1000 << "]");
                WARN("  Computed (mm): [" << pos_computed.x()*1000 << ", " << pos_computed.y()*1000 << ", " << pos_computed.z()*1000 << "]");
            }
        }
    }

    INFO("Results: " << passed << " passed, " << failed << " failed");
    INFO("Max position error: " << (max_pos_error * 1000) << " mm");
    INFO("Max rotation error: " << (max_rot_error / DEG2RAD) << " deg");

    double pass_rate = static_cast<double>(passed) / ref.test_cases.size();
    REQUIRE(pass_rate >= 0.99);
}

TEST_CASE("FK Validation: Yaskawa GP12 zero config", "[fk][validation][yaskawa]") {
    Robot robot = create_yaskawa_gp12_mdh();
    REQUIRE(robot.dof() == 6);

    Eigen::VectorXd q = Eigen::VectorXd::Zero(6);
    SE3 T = robot.fk(q);
    Eigen::Vector3d pos = T.translation();

    INFO("FK at zero config (Yaskawa GP12): pos (mm) = ["
         << pos.x()*1000 << ", " << pos.y()*1000 << ", " << pos.z()*1000 << "]");

    // Expected: x=895mm, y=0, z=814mm
    REQUIRE_THAT(pos.x(), Catch::Matchers::WithinAbs(0.895, 0.01));
    REQUIRE_THAT(pos.y(), Catch::Matchers::WithinAbs(0.0, 0.01));
    REQUIRE_THAT(pos.z(), Catch::Matchers::WithinAbs(0.814, 0.01));
}
