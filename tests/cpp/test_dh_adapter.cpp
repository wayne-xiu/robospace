#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <robospace/model/dh_adapter.hpp>
#include <robospace/model/dh_params.hpp>
#include <robospace/math/SE3.hpp>
#include <cmath>

using namespace robospace;
using namespace robospace::model;
using namespace robospace::math;

TEST_CASE("DHAdapter: Standard DH Convention", "[dh][adapter][standard]") {
    // UR5 Joint 1 parameters (standard DH)
    DHParams dh(0.0, 0.0, 0.089159, 0.0, DHConvention::STANDARD);

    SECTION("Zero angle") {
        SE3 T = DHAdapter::convert_standard(dh, 0.0, false);

        // At zero angle, should be pure translation along Z
        REQUIRE_THAT(T.translation().x(), Catch::Matchers::WithinAbs(0.0, 1e-6));
        REQUIRE_THAT(T.translation().y(), Catch::Matchers::WithinAbs(0.0, 1e-6));
        REQUIRE_THAT(T.translation().z(), Catch::Matchers::WithinAbs(0.089159, 1e-6));

        // Rotation should be identity
        REQUIRE_THAT(T.rotation()(0, 0), Catch::Matchers::WithinAbs(1.0, 1e-6));
        REQUIRE_THAT(T.rotation()(1, 1), Catch::Matchers::WithinAbs(1.0, 1e-6));
    }

    SECTION("90 degree rotation") {
        SE3 T = DHAdapter::convert_standard(dh, M_PI/2, false);

        // Check rotation matrix
        REQUIRE_THAT(T.rotation()(0, 0), Catch::Matchers::WithinAbs(0.0, 1e-6));
        REQUIRE_THAT(T.rotation()(0, 1), Catch::Matchers::WithinAbs(-1.0, 1e-6));
    }
}

TEST_CASE("DHAdapter: Modified DH Convention", "[dh][adapter][modified]") {
    // ABB IRB120 Joint 1 parameters (modified DH)
    DHParams dh(M_PI/2, 0.0, 0.0, 0.0, DHConvention::MODIFIED);

    SECTION("Zero angle") {
        SE3 T = DHAdapter::convert_modified(dh, 0.0, false);

        // Check rotation includes alpha rotation
        // Modified DH: Rot_x(alpha) first
        REQUIRE_THAT(T.rotation()(1, 1), Catch::Matchers::WithinAbs(0.0, 1e-6));
        REQUIRE_THAT(T.rotation()(2, 2), Catch::Matchers::WithinAbs(0.0, 1e-6));
    }
}

TEST_CASE("DHAdapter: Auto-detection via convert()", "[dh][adapter][auto]") {
    SECTION("Standard convention") {
        DHParams dh_std(0.0, 0.0, 0.1, 0.0, DHConvention::STANDARD);
        SE3 T_auto = DHAdapter::convert(dh_std, 0.0, false);
        SE3 T_std = DHAdapter::convert_standard(dh_std, 0.0, false);

        // Should match standard conversion
        REQUIRE_THAT((T_auto.matrix() - T_std.matrix()).norm(),
                     Catch::Matchers::WithinAbs(0.0, 1e-10));
    }

    SECTION("Modified convention") {
        DHParams dh_mod(M_PI/2, 0.0, 0.0, 0.0, DHConvention::MODIFIED);
        SE3 T_auto = DHAdapter::convert(dh_mod, 0.0, false);
        SE3 T_mod = DHAdapter::convert_modified(dh_mod, 0.0, false);

        // Should match modified conversion
        REQUIRE_THAT((T_auto.matrix() - T_mod.matrix()).norm(),
                     Catch::Matchers::WithinAbs(0.0, 1e-10));
    }
}

TEST_CASE("DHAdapter: Prismatic Joint", "[dh][adapter][prismatic]") {
    DHParams dh(0.0, 0.0, 0.0, 0.0, DHConvention::STANDARD);

    SECTION("Prismatic extension") {
        double extension = 0.5;  // 500mm extension
        SE3 T = DHAdapter::convert(dh, extension, true);

        // Prismatic adds to d (Z translation)
        REQUIRE_THAT(T.translation().z(), Catch::Matchers::WithinAbs(extension, 1e-6));
    }
}

TEST_CASE("DHAdapter: Consistency with DHParams", "[dh][adapter][consistency]") {
    // Verify DHAdapter produces identical results to DHParams::transform()

    SECTION("UR5 Joint parameters") {
        std::vector<DHParams> ur5_dh = {
            {0.0, 0.0, 0.089159, 0.0, DHConvention::STANDARD},
            {M_PI/2, 0.0, 0.0, 0.0, DHConvention::STANDARD},
            {0.0, 0.42500, 0.0, 0.0, DHConvention::STANDARD}
        };

        std::vector<double> test_angles = {0.0, M_PI/4, M_PI/2, -M_PI/3};

        for (const auto& dh : ur5_dh) {
            for (double q : test_angles) {
                SE3 T_adapter = DHAdapter::convert(dh, q, false);
                SE3 T_dhparams = dh.transform(q, false);

                // Should be identical
                REQUIRE_THAT((T_adapter.matrix() - T_dhparams.matrix()).norm(),
                             Catch::Matchers::WithinAbs(0.0, 1e-12));
            }
        }
    }
}
