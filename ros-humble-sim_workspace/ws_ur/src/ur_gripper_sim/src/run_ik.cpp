#include <memory>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <array>
#include <cctype>
#include <cmath>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Waypoint parser
// Parses files with the same structure as relevant_contours_cleared_coordinates.json
// [
//    [x, y, z],
//    ...,
//    [x, y, z]
// ]
static bool parse_waypoints(
    const std::string &path,
    std::vector<std::array<double,3>> &out_points,
    std::string &err
) {
    // Clear output vector in case it's not empty
    out_points.clear();
    // Open waypoints file
    std::ifstream ifs(path);
    if (!ifs.is_open()) {
        err = "Failed to open file: " + path;
        return false;
    }

    // Read the file
    std::string s(
        (std::istreambuf_iterator<char>(ifs)),
        std::istreambuf_iterator<char>()
    );

    // i -> index of each character in the string (file)
    // n -> length of the string (file)
    size_t i = 0, n = s.size();

    // Helper to skip whitespaces
    auto skip_ws = [&](void) {
        while (i < n && std::isspace(static_cast<unsigned char>(s[i]))) ++i;
    };

    skip_ws();

    // Except the correct format at the start
    if (i >= n || s[i] != '[') {
        err = "Expected '[' at start";
        return false;
    }
    ++i;

    skip_ws();

    // Parse points until the top level closing, expecting the same structure for each point
    while (i < n) {
        if (s[i] == ',') {
            ++i;
            skip_ws();
            continue;
        }
        if (s[i] == ']') {
            ++i;
            break;
        }

        if (s[i] != '[') {
            err = "Expected '[' for point at pos " + std::to_string(i);
            return false;
        }
        ++i;
        skip_ws();

        // Vector for each point
        std::array<double,3> pt;
        for (int k = 0; k < 3; ++k) {
            size_t start = i;

            // Incorporate the sign
            if (i < n && (s[i] == '+' || s[i] == '-')) {
                ++i;
            }

            // Absolute value of the coordinate
            bool saw_digit = false;
            while (i < n && std::isdigit(static_cast<unsigned char>(s[i]))) { ++i; saw_digit = true; }
            if (i < n && s[i] == '.') {
                ++i;
                while (i < n && std::isdigit(static_cast<unsigned char>(s[i]))) {
                    ++i;
                    saw_digit = true;
                }
            }

            // Handling of exponents and presence of digits
            if (i < n && (s[i] == 'e' || s[i] == 'E')) {
                ++i;
                if (i < n && (s[i] == '+' || s[i] == '-')) {
                    ++i;
                }
                bool exp_digits = false;
                while (i < n && std::isdigit(static_cast<unsigned char>(s[i]))) {
                    ++i;
                    exp_digits = true;
                }
                if (!exp_digits) {
                    err = "Malformed exponent at pos " + std::to_string(start);
                    return false;
                }
            }
            if (!saw_digit) {
                err = "Expected number at pos " + std::to_string(start);
                return false;
            }

            // Convert the strings into double and store
            std::string numstr = s.substr(start, i - start);
            try {
                pt[k] = std::stod(numstr);
            } catch (const std::exception &e) {
                err = std::string("stod failed: ") + e.what();
                return false;
            }

            skip_ws();
            if (k < 2) {
                if (i >= n || s[i] != ',') {
                    err = "Expected ',' between numbers at pos " + std::to_string(i);
                    return false;
                }
                ++i;
                skip_ws();
            }
        }

        if (i >= n || s[i] != ']') {
            err = "Expected ']' after point at pos " + std::to_string(i);
            return false;
        }
        ++i;
        skip_ws();
        out_points.push_back(pt);
    }

    return true;
}

int main(
    int argc,
    char **argv
) {
    // Initialize the ROS2 Node and the logger
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("move_group_interface_cartesian");
    auto logger = rclcpp::get_logger("move_group_interface_cartesian");

    // Spin the node
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner_thread([&executor]() {
        executor.spin();
    });
    rclcpp::sleep_for(std::chrono::seconds(2)); // Make sure the other nodes are ready

    // Create MoveGroup Interface and set it up
    using moveit::planning_interface::MoveGroupInterface;
    MoveGroupInterface move_group(node, "ur_manipulator");
    move_group.setPoseReferenceFrame("world");
    move_group.setPlanningTime(10.0);
    move_group.setMaxVelocityScalingFactor(0.2);
    move_group.setMaxAccelerationScalingFactor(0.2);
    // Log the setup info
    RCLCPP_INFO(logger, "Pose reference frame: %s", move_group.getPoseReferenceFrame().c_str());
    RCLCPP_INFO(logger, "Planning frame: %s", move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(logger, "End effector link: %s", move_group.getEndEffectorLink().c_str());

    // Create the Planning Scene and add the collision objects
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.resize(2);
    // Description of the Robot Base
    collision_objects[0].id = "robot_base";
    collision_objects[0].header.frame_id = "world";
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    collision_objects[0].primitives[0].dimensions = {0.750, 0.175};
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.000;
    collision_objects[0].primitive_poses[0].position.y = 0.000;
    collision_objects[0].primitive_poses[0].position.z = 0.375;
    collision_objects[0].operation = moveit_msgs::msg::CollisionObject::ADD;
    // Description of the Target Base
    collision_objects[1].id = "target_base";
    collision_objects[1].header.frame_id = "world";
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    collision_objects[1].primitives[0].dimensions = {0.25, 0.75, 1.00};
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = 0.50;
    collision_objects[1].primitive_poses[0].position.y = 0.00;
    collision_objects[1].primitive_poses[0].position.z = 0.50;
    collision_objects[1].operation = moveit_msgs::msg::CollisionObject::ADD;
    // Add the objects
    planning_scene_interface.applyCollisionObjects(collision_objects);
    RCLCPP_INFO(logger, "Collision objects added to the planning scene.");

    // Load the trajetory waypoints
    const std::string json_path = "/common/trajectory_planning/relevant_contours_cleared_coordinates.json";
    std::vector<std::array<double,3>> pts;
    std::string perr;
    if (!parse_waypoints(json_path, pts, perr)) {
        RCLCPP_ERROR(logger, "JSON parse error: %s", perr.c_str());
        rclcpp::shutdown();
        spinner_thread.join();
        return 1;
    }
    if (pts.empty()) {
        RCLCPP_ERROR(logger, "No points loaded from JSON.");
        rclcpp::shutdown();
        spinner_thread.join();
        return 1;
    }

    // Set the tool0 Reference Frame target orientation to rpy = [0, -pi, -pi]
    tf2::Quaternion q;
    q.setRPY(0.0, -M_PI, -M_PI);
    geometry_msgs::msg::Quaternion orientation = tf2::toMsg(q);

    // Build the waypoints from the pts vector coming from the file
    // NOTE: the target z position will be offset by the length of the end effector in the closed postion
    // Such info is available on the Robotiq website:
    // https://assets.robotiq.com/website-assets/support_documents/document/online/2F-85_2F-140_TM_InstructionManual_HTML5_20190503.zip/2F-85_2F-140_TM_InstructionManual_HTML5/Content/6.%20Specifications.htm
    const double z_offset = 0.163;
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.reserve(pts.size());
    for (const auto &pp : pts) {
        geometry_msgs::msg::Pose p;
        p.position.x = pp[0];
        p.position.y = pp[1];
        p.position.z = pp[2] + z_offset;
        p.orientation = orientation;
        waypoints.push_back(p);
    }

    // Compute Cartesian path
    moveit_msgs::msg::RobotTrajectory trajectory_msg;
    const double eef_step = 0.001;
    const double jump_threshold = 0.0;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory_msg);
    RCLCPP_INFO(logger, "Computed Cartesian path fraction: %.3f", fraction);
    if (fraction < 0.99) {
        RCLCPP_ERROR(logger, "Cartesian path incomplete (fraction < 0.99). Aborting.");
        rclcpp::shutdown();
        spinner_thread.join();
        return 1;
    }

    // Execute trajectory
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory_msg;

    // Log the success of the operation
    bool success = (move_group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(logger, "Execution: %s", success ? "SUCCESS" : "FAILED");

    // Exit
    rclcpp::shutdown();
    spinner_thread.join();
    if (success) {
        return 0;
    } else {
        return 1;
    }
}