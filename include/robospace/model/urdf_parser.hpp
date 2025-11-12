#pragma once

#include <robospace/model/robot.hpp>
#include <robospace/model/link.hpp>
#include <robospace/model/joint.hpp>
#include <robospace/math/SE3.hpp>
#include <robospace/math/SO3.hpp>
#include <string>
#include <memory>

// Forward declare TinyXML2 classes to avoid including in header
namespace tinyxml2 {
    class XMLElement;
    class XMLDocument;
}

namespace robospace {
namespace model {

/**
 * @brief Parser for URDF (Unified Robot Description Format) files
 *
 * Converts URDF XML format to Robospace Robot model.
 * Supports:
 * - Link definitions (inertial, visual, collision)
 * - Joint definitions (revolute, prismatic, continuous, fixed)
 * - Joint limits (position, velocity, effort)
 * - Coordinate frame transformations
 *
 * URDF Specification: http://wiki.ros.org/urdf/XML
 */
class URDFParser {
public:
    /**
     * @brief Parse URDF file and create Robot
     * @param urdf_path Path to URDF file
     * @return Robot object constructed from URDF
     * @throws std::runtime_error if file not found or parsing fails
     */
    static Robot parse_file(const std::string& urdf_path);

    /**
     * @brief Parse URDF from string
     * @param urdf_string URDF XML content as string
     * @return Robot object constructed from URDF
     * @throws std::runtime_error if parsing fails
     */
    static Robot parse_string(const std::string& urdf_string);

private:
    // Parse robot element (root)
    static Robot parse_robot(tinyxml2::XMLDocument& doc);

    // Parse link elements
    static Link parse_link(tinyxml2::XMLElement* link_elem);
    static void parse_inertial(tinyxml2::XMLElement* inertial_elem, Link& link);
    static void parse_visual(tinyxml2::XMLElement* visual_elem, Link& link);
    static void parse_collision(tinyxml2::XMLElement* collision_elem, Link& link);

    // Parse joint elements
    static Joint parse_joint(tinyxml2::XMLElement* joint_elem,
                            const std::unordered_map<std::string, int>& link_name_to_id);
    static void parse_joint_limits(tinyxml2::XMLElement* limit_elem, Joint& joint);

    // Parse transforms (origin elements)
    static math::SE3 parse_origin(tinyxml2::XMLElement* origin_elem);

    // Parse vectors
    static Eigen::Vector3d parse_xyz(const std::string& xyz_str);
    static Eigen::Vector3d parse_rpy(const std::string& rpy_str);

    // Utility functions
    static std::string get_attribute(tinyxml2::XMLElement* elem, const char* attr_name);
    static std::string get_attribute_or_default(tinyxml2::XMLElement* elem,
                                                const char* attr_name,
                                                const std::string& default_value);
    static bool has_attribute(tinyxml2::XMLElement* elem, const char* attr_name);
};

} // namespace model
} // namespace robospace
