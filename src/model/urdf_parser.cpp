#include <robospace/model/urdf_parser.hpp>
#include <tinyxml2.h>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <cmath>
#include <unordered_set>

namespace robospace {
namespace model {

Robot URDFParser::parse_file(const std::string& urdf_path) {
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

    struct JointTopology {
        tinyxml2::XMLElement* element = nullptr;
        std::string name;
        std::string parent;
        std::string child;
        bool is_fixed = false;
    };

    // First pass: parse links
    std::unordered_map<std::string, Link> links_by_name;
    for (auto* link_elem = robot_elem->FirstChildElement("link");
         link_elem;
         link_elem = link_elem->NextSiblingElement("link")) {
        Link link = parse_link(link_elem);
        std::string link_name = link.name();
        if (links_by_name.find(link_name) != links_by_name.end()) {
            throw std::runtime_error("Duplicate link name in URDF: " + link_name);
        }
        links_by_name.emplace(link_name, link);
    }

    if (links_by_name.empty()) {
        throw std::runtime_error("URDF has no <link> elements");
    }

    // Second pass: parse joint topology for ordering
    std::vector<JointTopology> joints;
    std::unordered_map<std::string, std::vector<int>> joints_by_parent;
    std::unordered_set<std::string> child_links;

    for (auto* joint_elem = robot_elem->FirstChildElement("joint");
         joint_elem;
         joint_elem = joint_elem->NextSiblingElement("joint")) {
        auto* parent_elem = joint_elem->FirstChildElement("parent");
        auto* child_elem = joint_elem->FirstChildElement("child");
        if (!parent_elem || !child_elem) {
            throw std::runtime_error("Joint missing parent or child link");
        }

        std::string joint_name = get_attribute(joint_elem, "name");
        std::string type_str = get_attribute(joint_elem, "type");
        bool is_fixed = false;
        if (type_str == "fixed") {
            is_fixed = true;
        } else if (type_str == "revolute" || type_str == "prismatic" ||
                   type_str == "continuous") {
            is_fixed = false;
        } else {
            throw std::runtime_error("Unsupported joint type: " + type_str);
        }
        std::string parent_name = get_attribute(parent_elem, "link");
        std::string child_name = get_attribute(child_elem, "link");

        if (links_by_name.find(parent_name) == links_by_name.end()) {
            throw std::runtime_error("Parent link not found: " + parent_name);
        }
        if (links_by_name.find(child_name) == links_by_name.end()) {
            throw std::runtime_error("Child link not found: " + child_name);
        }

        if (!child_links.insert(child_name).second) {
            throw std::runtime_error("Link has multiple parents: " + child_name);
        }

        int joint_index = static_cast<int>(joints.size());
        joints.push_back({joint_elem, joint_name, parent_name, child_name, is_fixed});
        joints_by_parent[parent_name].push_back(joint_index);
    }

    if (joints.empty()) {
        if (links_by_name.size() == 1) {
            const auto& link_name = links_by_name.begin()->first;
            robot.add_link(links_by_name.at(link_name));
            return robot;
        }
        throw std::runtime_error("URDF has multiple links but no joints");
    }

    // Find root link (never appears as a child)
    std::string root_link;
    for (const auto& kv : links_by_name) {
        if (child_links.find(kv.first) == child_links.end()) {
            if (!root_link.empty()) {
                throw std::runtime_error("Multiple root links detected in URDF");
            }
            root_link = kv.first;
        }
    }

    if (root_link.empty()) {
        throw std::runtime_error("No root link detected in URDF");
    }

    // Helper: detect whether a subtree contains any active (non-fixed) joints
    std::unordered_map<std::string, bool> active_cache;
    std::unordered_set<std::string> active_stack;
    auto subtree_has_active = [&](const std::string& link_name, const auto& self) -> bool {
        auto cached = active_cache.find(link_name);
        if (cached != active_cache.end()) {
            return cached->second;
        }

        if (!active_stack.insert(link_name).second) {
            throw std::runtime_error("Cycle detected in URDF at link: " + link_name);
        }

        bool has_active = false;
        auto it = joints_by_parent.find(link_name);
        if (it != joints_by_parent.end()) {
            for (int joint_index : it->second) {
                if (!joints[joint_index].is_fixed) {
                    has_active = true;
                    break;
                }
                if (self(joints[joint_index].child, self)) {
                    has_active = true;
                    break;
                }
            }
        }

        active_stack.erase(link_name);
        active_cache[link_name] = has_active;
        return has_active;
    };

    // Helper: maximum depth from this link (number of joints to deepest leaf)
    std::unordered_map<std::string, int> depth_cache;
    std::unordered_set<std::string> depth_stack;
    auto subtree_depth = [&](const std::string& link_name, const auto& self) -> int {
        auto cached = depth_cache.find(link_name);
        if (cached != depth_cache.end()) {
            return cached->second;
        }

        if (!depth_stack.insert(link_name).second) {
            throw std::runtime_error("Cycle detected in URDF at link: " + link_name);
        }

        int max_depth = 0;
        auto it = joints_by_parent.find(link_name);
        if (it != joints_by_parent.end()) {
            for (int joint_index : it->second) {
                int depth = 1 + self(joints[joint_index].child, self);
                if (depth > max_depth) {
                    max_depth = depth;
                }
            }
        }

        depth_stack.erase(link_name);
        depth_cache[link_name] = max_depth;
        return max_depth;
    };

    // Traverse serial chain from root
    std::vector<std::string> ordered_links;
    std::vector<int> ordered_joints;
    std::unordered_set<std::string> visited_links;

    std::string current = root_link;
    ordered_links.push_back(current);
    visited_links.insert(current);

    while (true) {
        auto it = joints_by_parent.find(current);
        if (it == joints_by_parent.end()) {
            break;
        }
        std::vector<int> active_candidates;
        for (int joint_index : it->second) {
            if (!joints[joint_index].is_fixed ||
                subtree_has_active(joints[joint_index].child, subtree_has_active)) {
                active_candidates.push_back(joint_index);
            }
        }

        int joint_index = -1;
        if (active_candidates.size() > 1) {
            throw std::runtime_error("Branching detected at link: " + current);
        }
        if (active_candidates.size() == 1) {
            joint_index = active_candidates.front();
        } else {
            int best_depth = -1;
            for (int candidate : it->second) {
                int depth = 1 + subtree_depth(joints[candidate].child, subtree_depth);
                if (depth > best_depth) {
                    best_depth = depth;
                    joint_index = candidate;
                } else if (depth == best_depth && joint_index >= 0) {
                    if (joints[candidate].name < joints[joint_index].name) {
                        joint_index = candidate;
                    }
                }
            }
        }

        if (joint_index < 0) {
            break;
        }

        ordered_joints.push_back(joint_index);
        const std::string& child = joints[joint_index].child;

        if (!visited_links.insert(child).second) {
            throw std::runtime_error("Cycle detected in URDF at link: " + child);
        }

        ordered_links.push_back(child);
        current = child;
    }

    int active_total = 0;
    int active_in_chain = 0;
    for (const auto& joint : joints) {
        if (!joint.is_fixed) {
            active_total++;
        }
    }
    for (int joint_index : ordered_joints) {
        if (!joints[joint_index].is_fixed) {
            active_in_chain++;
        }
    }

    if (active_in_chain != active_total) {
        throw std::runtime_error("URDF is not a single serial chain (active joints disconnected)");
    }

    // Add links in chain order
    std::unordered_map<std::string, int> link_name_to_id;
    int link_id = 0;
    for (const auto& link_name : ordered_links) {
        robot.add_link(links_by_name.at(link_name));
        link_name_to_id[link_name] = link_id++;
    }

    // Add joints in chain order
    for (int joint_index : ordered_joints) {
        Joint joint = parse_joint(joints[joint_index].element, link_name_to_id);
        robot.add_joint(joint);
    }

    return robot;
}

Link URDFParser::parse_link(tinyxml2::XMLElement* link_elem) {
    std::string name = get_attribute(link_elem, "name");
    Link link(name);

    auto* inertial_elem = link_elem->FirstChildElement("inertial");
    if (inertial_elem) {
        parse_inertial(inertial_elem, link);
    }

    auto* visual_elem = link_elem->FirstChildElement("visual");
    if (visual_elem) {
        parse_visual(visual_elem, link);
    }

    auto* collision_elem = link_elem->FirstChildElement("collision");
    if (collision_elem) {
        parse_collision(collision_elem, link);
    }

    return link;
}

void URDFParser::parse_inertial(tinyxml2::XMLElement* inertial_elem, Link& link) {
    auto* mass_elem = inertial_elem->FirstChildElement("mass");
    if (mass_elem) {
        double mass = std::stod(get_attribute(mass_elem, "value"));
        link.set_mass(mass);
    }

    auto* origin_elem = inertial_elem->FirstChildElement("origin");
    if (origin_elem) {
        math::SE3 com_pose = parse_origin(origin_elem);
        link.set_com(com_pose.translation());
    }

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

    auto* origin_elem = joint_elem->FirstChildElement("origin");
    if (origin_elem) {
        math::SE3 origin = parse_origin(origin_elem);
        joint.set_origin(origin);
    }

    auto* axis_elem = joint_elem->FirstChildElement("axis");
    if (axis_elem) {
        std::string xyz_str = get_attribute(axis_elem, "xyz");
        Eigen::Vector3d axis = parse_xyz(xyz_str);
        joint.set_axis(axis.normalized());
    }

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
