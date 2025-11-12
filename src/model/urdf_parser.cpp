#include <robospace/model/urdf_parser.hpp>
#include <tinyxml2.h>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <cmath>

namespace robospace {
namespace model {

Robot URDFParser::parse_file(const std::string& urdf_path) {
    // Load file into string
    std::ifstream file(urdf_path);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open URDF file: " + urdf_path);
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    return parse_string(buffer.str());
}

Robot URDFParser::parse_string(const std::string& urdf_string) {
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLError result = doc.Parse(urdf_string.c_str());

    if (result != tinyxml2::XML_SUCCESS) {
        throw std::runtime_error("Failed to parse URDF XML: " + std::string(doc.ErrorName()));
    }

    return parse_robot(doc);
}

Robot URDFParser::parse_robot(tinyxml2::XMLDocument& doc) {
    auto* robot_elem = doc.FirstChildElement("robot");
    if (!robot_elem) {
        throw std::runtime_error("No <robot> element found in URDF");
    }

    std::string robot_name = get_attribute(robot_elem, "name");
    Robot robot(robot_name);

    // First pass: parse and add all links
    std::unordered_map<std::string, int> link_name_to_id;
    int link_id = 0;

    for (auto* link_elem = robot_elem->FirstChildElement("link");
         link_elem;
         link_elem = link_elem->NextSiblingElement("link")) {
        Link link = parse_link(link_elem);
        std::string link_name = link.name();
        robot.add_link(link);
        link_name_to_id[link_name] = link_id++;
    }

    // Second pass: parse and add all joints
    for (auto* joint_elem = robot_elem->FirstChildElement("joint");
         joint_elem;
         joint_elem = joint_elem->NextSiblingElement("joint")) {
        Joint joint = parse_joint(joint_elem, link_name_to_id);
        robot.add_joint(joint);
    }

    return robot;
}

Link URDFParser::parse_link(tinyxml2::XMLElement* link_elem) {
    std::string name = get_attribute(link_elem, "name");
    Link link(name);

    // Parse inertial properties (optional)
    auto* inertial_elem = link_elem->FirstChildElement("inertial");
    if (inertial_elem) {
        parse_inertial(inertial_elem, link);
    }

    // Parse visual geometry (optional)
    auto* visual_elem = link_elem->FirstChildElement("visual");
    if (visual_elem) {
        parse_visual(visual_elem, link);
    }

    // Parse collision geometry (optional)
    auto* collision_elem = link_elem->FirstChildElement("collision");
    if (collision_elem) {
        parse_collision(collision_elem, link);
    }

    return link;
}

void URDFParser::parse_inertial(tinyxml2::XMLElement* inertial_elem, Link& link) {
    // Parse mass
    auto* mass_elem = inertial_elem->FirstChildElement("mass");
    if (mass_elem) {
        double mass = std::stod(get_attribute(mass_elem, "value"));
        link.set_mass(mass);
    }

    // Parse center of mass (origin)
    auto* origin_elem = inertial_elem->FirstChildElement("origin");
    if (origin_elem) {
        math::SE3 com_pose = parse_origin(origin_elem);
        link.set_com(com_pose.translation());
    }

    // Parse inertia tensor
    auto* inertia_elem = inertial_elem->FirstChildElement("inertia");
    if (inertia_elem) {
        double ixx = std::stod(get_attribute(inertia_elem, "ixx"));
        double ixy = std::stod(get_attribute(inertia_elem, "ixy"));
        double ixz = std::stod(get_attribute(inertia_elem, "ixz"));
        double iyy = std::stod(get_attribute(inertia_elem, "iyy"));
        double iyz = std::stod(get_attribute(inertia_elem, "iyz"));
        double izz = std::stod(get_attribute(inertia_elem, "izz"));

        Eigen::Matrix3d inertia;
        inertia << ixx, ixy, ixz,
                   ixy, iyy, iyz,
                   ixz, iyz, izz;
        link.set_inertia(inertia);
    }
}

void URDFParser::parse_visual(tinyxml2::XMLElement* visual_elem, Link& link) {
    // Parse geometry element
    auto* geometry_elem = visual_elem->FirstChildElement("geometry");
    if (geometry_elem) {
        auto* mesh_elem = geometry_elem->FirstChildElement("mesh");
        if (mesh_elem && has_attribute(mesh_elem, "filename")) {
            std::string mesh_path = get_attribute(mesh_elem, "filename");
            link.set_visual_mesh(mesh_path);
        }
    }
}

void URDFParser::parse_collision(tinyxml2::XMLElement* collision_elem, Link& link) {
    // Parse geometry element
    auto* geometry_elem = collision_elem->FirstChildElement("geometry");
    if (geometry_elem) {
        auto* mesh_elem = geometry_elem->FirstChildElement("mesh");
        if (mesh_elem && has_attribute(mesh_elem, "filename")) {
            std::string mesh_path = get_attribute(mesh_elem, "filename");
            link.set_collision_mesh(mesh_path);
        }
    }
}

Joint URDFParser::parse_joint(tinyxml2::XMLElement* joint_elem,
                              const std::unordered_map<std::string, int>& link_name_to_id) {
    std::string name = get_attribute(joint_elem, "name");
    std::string type_str = get_attribute(joint_elem, "type");

    // Parse joint type
    JointType type;
    if (type_str == "revolute") {
        type = JointType::REVOLUTE;
    } else if (type_str == "prismatic") {
        type = JointType::PRISMATIC;
    } else if (type_str == "continuous") {
        type = JointType::CONTINUOUS;
    } else if (type_str == "fixed") {
        type = JointType::FIXED;
    } else {
        throw std::runtime_error("Unsupported joint type: " + type_str);
    }

    // Parse parent and child links
    auto* parent_elem = joint_elem->FirstChildElement("parent");
    auto* child_elem = joint_elem->FirstChildElement("child");
    if (!parent_elem || !child_elem) {
        throw std::runtime_error("Joint " + name + " missing parent or child link");
    }

    std::string parent_name = get_attribute(parent_elem, "link");
    std::string child_name = get_attribute(child_elem, "link");

    auto parent_it = link_name_to_id.find(parent_name);
    auto child_it = link_name_to_id.find(child_name);

    if (parent_it == link_name_to_id.end()) {
        throw std::runtime_error("Parent link not found: " + parent_name);
    }
    if (child_it == link_name_to_id.end()) {
        throw std::runtime_error("Child link not found: " + child_name);
    }

    int parent_id = parent_it->second;
    int child_id = child_it->second;

    Joint joint(name, type, parent_id, child_id);

    // Parse origin (transform from parent to child)
    auto* origin_elem = joint_elem->FirstChildElement("origin");
    if (origin_elem) {
        math::SE3 origin = parse_origin(origin_elem);
        joint.set_origin(origin);
    }

    // Parse axis (for revolute/prismatic joints)
    auto* axis_elem = joint_elem->FirstChildElement("axis");
    if (axis_elem) {
        std::string xyz_str = get_attribute(axis_elem, "xyz");
        Eigen::Vector3d axis = parse_xyz(xyz_str);
        joint.set_axis(axis.normalized());
    }

    // Parse joint limits (for revolute/prismatic joints)
    auto* limit_elem = joint_elem->FirstChildElement("limit");
    if (limit_elem) {
        parse_joint_limits(limit_elem, joint);
    }

    return joint;
}

void URDFParser::parse_joint_limits(tinyxml2::XMLElement* limit_elem, Joint& joint) {
    if (has_attribute(limit_elem, "lower") && has_attribute(limit_elem, "upper")) {
        double lower = std::stod(get_attribute(limit_elem, "lower"));
        double upper = std::stod(get_attribute(limit_elem, "upper"));
        joint.set_limits(lower, upper);
    }

    if (has_attribute(limit_elem, "velocity")) {
        double velocity = std::stod(get_attribute(limit_elem, "velocity"));
        joint.set_velocity_limit(velocity);
    }

    if (has_attribute(limit_elem, "effort")) {
        double effort = std::stod(get_attribute(limit_elem, "effort"));
        joint.set_effort_limit(effort);
    }
}

math::SE3 URDFParser::parse_origin(tinyxml2::XMLElement* origin_elem) {
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
    Eigen::Vector3d rpy = Eigen::Vector3d::Zero();

    if (has_attribute(origin_elem, "xyz")) {
        std::string xyz_str = get_attribute(origin_elem, "xyz");
        translation = parse_xyz(xyz_str);
    }

    if (has_attribute(origin_elem, "rpy")) {
        std::string rpy_str = get_attribute(origin_elem, "rpy");
        rpy = parse_rpy(rpy_str);
    }

    // Convert RPY to rotation matrix (ZYX convention - URDF standard)
    double roll = rpy(0);
    double pitch = rpy(1);
    double yaw = rpy(2);

    // Compute rotation matrix R = Rz(yaw) * Ry(pitch) * Rx(roll)
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

    return math::SE3(R, translation);
}

Eigen::Vector3d URDFParser::parse_xyz(const std::string& xyz_str) {
    std::istringstream iss(xyz_str);
    double x, y, z;
    if (!(iss >> x >> y >> z)) {
        throw std::runtime_error("Failed to parse xyz: " + xyz_str);
    }
    return Eigen::Vector3d(x, y, z);
}

Eigen::Vector3d URDFParser::parse_rpy(const std::string& rpy_str) {
    std::istringstream iss(rpy_str);
    double roll, pitch, yaw;
    if (!(iss >> roll >> pitch >> yaw)) {
        throw std::runtime_error("Failed to parse rpy: " + rpy_str);
    }
    return Eigen::Vector3d(roll, pitch, yaw);
}

std::string URDFParser::get_attribute(tinyxml2::XMLElement* elem, const char* attr_name) {
    const char* attr_value = elem->Attribute(attr_name);
    if (!attr_value) {
        throw std::runtime_error("Missing required attribute: " + std::string(attr_name));
    }
    return std::string(attr_value);
}

std::string URDFParser::get_attribute_or_default(tinyxml2::XMLElement* elem,
                                                 const char* attr_name,
                                                 const std::string& default_value) {
    const char* attr_value = elem->Attribute(attr_name);
    return attr_value ? std::string(attr_value) : default_value;
}

bool URDFParser::has_attribute(tinyxml2::XMLElement* elem, const char* attr_name) {
    return elem->Attribute(attr_name) != nullptr;
}

} // namespace model
} // namespace robospace
