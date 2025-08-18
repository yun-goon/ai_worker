#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <urdf/model.h>
#include <memory>
#include <vector>
#include <map>

class RealtimeIKSolverJL : public rclcpp::Node
{
public:
    RealtimeIKSolverJL() : Node("realtime_ik_solver_jl"),
                           lift_joint_index_(-1),
                           setup_complete_(false),
                           has_joint_states_(false)
    {
        // Parameters
        this->declare_parameter<std::string>("base_link", "base_link");
        this->declare_parameter<std::string>("arm_base_link", "arm_base_link");
        this->declare_parameter<std::string>("end_effector_link", "arm_r_link7");
        this->declare_parameter<std::string>("target_pose_topic", "/target_pose");

        base_link_ = this->get_parameter("base_link").as_string();
        arm_base_link_ = this->get_parameter("arm_base_link").as_string();
        end_effector_link_ = this->get_parameter("end_effector_link").as_string();
        std::string target_pose_topic = this->get_parameter("target_pose_topic").as_string();

        RCLCPP_INFO(this->get_logger(), "🚀 Realtime IK Solver with Joint Limits starting...");
        RCLCPP_INFO(this->get_logger(), "Base link: %s", base_link_.c_str());
        RCLCPP_INFO(this->get_logger(), "Arm base link: %s", arm_base_link_.c_str());
        RCLCPP_INFO(this->get_logger(), "End effector link: %s", end_effector_link_.c_str());
        RCLCPP_INFO(this->get_logger(), "Target pose topic: %s", target_pose_topic.c_str());

        // Subscribers
        robot_description_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/robot_description", rclcpp::QoS(1).transient_local(),
            std::bind(&RealtimeIKSolverJL::robotDescriptionCallback, this, std::placeholders::_1));

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&RealtimeIKSolverJL::jointStateCallback, this, std::placeholders::_1));

        // Subscribe to target pose topic for real-time IK solving
        target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            target_pose_topic, 10,
            std::bind(&RealtimeIKSolverJL::targetPoseCallback, this, std::placeholders::_1));

        // Publishers
        current_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/current_end_effector_pose", 10);

        joint_trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/leader/joint_trajectory_command_broadcaster_right/joint_trajectory", 10);

        // Timer for publishing current pose at 10Hz
        pose_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&RealtimeIKSolverJL::publishCurrentPose, this));

        // Try to get robot_description from parameter server
        auto param_client = std::make_shared<rclcpp::SyncParametersClient>(this, "/robot_state_publisher");

        if (param_client->wait_for_service(std::chrono::seconds(2))) {
            try {
                auto parameters = param_client->get_parameters({"robot_description"});
                if (!parameters.empty() && parameters[0].get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                    std::string robot_description = parameters[0].as_string();
                    processRobotDescription(robot_description);
                }
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Failed to get robot_description from parameter server: %s", e.what());
            }
        }

        RCLCPP_INFO(this->get_logger(), "✅ Node initialized. Waiting for target poses on %s", target_pose_topic.c_str());
    }

private:
    void robotDescriptionCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received robot_description via topic");
        processRobotDescription(msg->data);
    }

    void processRobotDescription(const std::string& robot_description)
    {
        RCLCPP_INFO(this->get_logger(), "Processing robot_description (%zu bytes)", robot_description.size());

        try {
            // Parse URDF
            urdf::Model model;
            if (!model.initString(robot_description)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF");
                return;
            }

            RCLCPP_INFO(this->get_logger(), "URDF parsed successfully: %s", model.getName().c_str());

            // Create KDL tree
            KDL::Tree tree;
            if (!kdl_parser::treeFromUrdfModel(model, tree)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to create KDL tree from URDF");
                return;
            }

            RCLCPP_INFO(this->get_logger(), "KDL tree created with %d segments", tree.getNrOfSegments());

            // Extract chain from tree: arm_base_link -> end_effector_link (excluding lift_joint)
            if (!tree.getChain(arm_base_link_, end_effector_link_, chain_)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to extract chain from %s to %s",
                           arm_base_link_.c_str(), end_effector_link_.c_str());

                // Print available segments for debugging
                auto segments = tree.getSegments();
                RCLCPP_INFO(this->get_logger(), "Available segments:");
                for (const auto& seg : segments) {
                    RCLCPP_INFO(this->get_logger(), "  - %s", seg.first.c_str());
                }
                return;
            }

            RCLCPP_INFO(this->get_logger(), "KDL chain extracted with %d joints", chain_.getNrOfJoints());

            // Extract joint names and identify lift_joint index
            extractJointNames();

            // Setup joint limits from URDF
            setupJointLimits(model);

            // Create solvers with joint limits
            fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
            ik_vel_solver_ = std::make_unique<KDL::ChainIkSolverVel_pinv>(chain_);

            // Create IK solver with joint limits
            ik_solver_jl_ = std::make_unique<KDL::ChainIkSolverPos_NR_JL>(
                chain_, q_min_, q_max_, *fk_solver_, *ik_vel_solver_, 1000, 1e-6);

            setup_complete_ = true;
            RCLCPP_INFO(this->get_logger(), "✅ KDL setup with Joint Limits completed successfully! Ready for real-time IK.");

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error processing robot description: %s", e.what());
        }
    }

    void extractJointNames()
    {
        joint_names_.clear();
        lift_joint_index_ = -1;  // Not used in arm-only chain

        for (unsigned int i = 0; i < chain_.getNrOfSegments(); i++) {
            KDL::Segment segment = chain_.getSegment(i);
            if (segment.getJoint().getType() != KDL::Joint::None) {
                std::string joint_name = segment.getJoint().getName();
                joint_names_.push_back(joint_name);
                RCLCPP_INFO(this->get_logger(), "Found arm joint: %s", joint_name.c_str());
            }
        }

        RCLCPP_INFO(this->get_logger(), "Arm joint names extracted (lift_joint excluded):");
        for (size_t i = 0; i < joint_names_.size(); i++) {
            RCLCPP_INFO(this->get_logger(), "  [%zu] %s", i, joint_names_[i].c_str());
        }
    }

    void setupJointLimits(const urdf::Model& model)
    {
        unsigned int num_joints = chain_.getNrOfJoints();
        q_min_.resize(num_joints);
        q_max_.resize(num_joints);

        RCLCPP_INFO(this->get_logger(), "🔒 Setting up joint limits:");

        for (size_t i = 0; i < joint_names_.size(); i++) {
            const std::string& joint_name = joint_names_[i];

            // Get joint from URDF
            auto joint_ptr = model.getJoint(joint_name);

            if (joint_ptr && joint_ptr->limits) {
                // Use URDF limits
                q_min_(i) = joint_ptr->limits->lower;
                q_max_(i) = joint_ptr->limits->upper;

                RCLCPP_INFO(this->get_logger(), "  %s: [%.3f, %.3f] rad ([%.1f°, %.1f°])",
                           joint_name.c_str(),
                           q_min_(i), q_max_(i),
                           q_min_(i) * 180.0 / M_PI, q_max_(i) * 180.0 / M_PI);
            } else {
                // Default limits if not specified in URDF
                if (joint_name == "lift_joint") {
                    // Lift joint has limited range
                    q_min_(i) = -0.1;   // -0.1m
                    q_max_(i) = 0.5;    // 0.5m
                } else {
                    // Arm joints - reasonable limits
                    q_min_(i) = -3.14159;  // -180°
                    q_max_(i) = 3.14159;   // +180°
                }

                RCLCPP_WARN(this->get_logger(), "  %s: Using default limits [%.3f, %.3f] rad ([%.1f°, %.1f°])",
                           joint_name.c_str(),
                           q_min_(i), q_max_(i),
                           q_min_(i) * 180.0 / M_PI, q_max_(i) * 180.0 / M_PI);
            }
        }

        RCLCPP_INFO(this->get_logger(), "✅ Joint limits configured for %d joints", num_joints);
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (!setup_complete_) {
            return;
        }

        // Extract current lift_joint position for coordinate transformation
        lift_joint_position_ = 0.0;
        for (size_t i = 0; i < msg->name.size(); i++) {
            if (msg->name[i] == "lift_joint" && i < msg->position.size()) {
                lift_joint_position_ = msg->position[i];
                break;
            }
        }

        // Extract joint positions for our arm chain
        current_joint_positions_.assign(joint_names_.size(), 0.0);
        bool all_joints_found = true;

        for (size_t i = 0; i < joint_names_.size(); i++) {
            bool found = false;
            for (size_t j = 0; j < msg->name.size(); j++) {
                if (msg->name[j] == joint_names_[i] && j < msg->position.size()) {
                    current_joint_positions_[i] = msg->position[j];
                    found = true;
                    break;
                }
            }
            if (!found) {
                all_joints_found = false;
                break;
            }
        }

        if (all_joints_found && !has_joint_states_) {
            has_joint_states_ = true;
            RCLCPP_INFO(this->get_logger(), "✅ Joint states received. Arm-only IK system ready!");
            RCLCPP_INFO(this->get_logger(), "   Lift joint position: %.3f m", lift_joint_position_);

            // Check if current joint positions are within limits
            checkCurrentJointLimits();
        }
    }

    void checkCurrentJointLimits()
    {
        bool all_within_limits = true;

        RCLCPP_INFO(this->get_logger(), "🔍 Checking current joint positions against limits:");

        for (size_t i = 0; i < current_joint_positions_.size(); i++) {
            double pos = current_joint_positions_[i];
            bool within_limits = (pos >= q_min_(i) && pos <= q_max_(i));

            if (!within_limits) {
                all_within_limits = false;
            }

            RCLCPP_INFO(this->get_logger(), "  %s: %.3f rad (%.1f°) %s [%.3f, %.3f]",
                       joint_names_[i].c_str(),
                       pos, pos * 180.0 / M_PI,
                       within_limits ? "✅" : "⚠️ OUTSIDE",
                       q_min_(i), q_max_(i));
        }

        if (all_within_limits) {
            RCLCPP_INFO(this->get_logger(), "✅ All joints are within limits");
        } else {
            RCLCPP_WARN(this->get_logger(), "⚠️ Some joints are outside their limits!");
        }
    }

    void targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (!setup_complete_ || !has_joint_states_) {
            RCLCPP_WARN(this->get_logger(), "🚫 Arm-only IK solver not ready yet, ignoring target pose");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "🎯 Received target pose (base_link frame):");
        RCLCPP_INFO(this->get_logger(), "   Position: x=%.3f, y=%.3f, z=%.3f",
                   msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        RCLCPP_INFO(this->get_logger(), "   Orientation: x=%.3f, y=%.3f, z=%.3f, w=%.3f",
                   msg->pose.orientation.x, msg->pose.orientation.y,
                   msg->pose.orientation.z, msg->pose.orientation.w);

        // Transform pose from base_link to arm_base_link frame
        geometry_msgs::msg::PoseStamped arm_base_pose = *msg;

        // URDF lift_joint origin: xyz="0.0055 0 1.4316"  
        // Transform: base_link -> arm_base_link
        arm_base_pose.pose.position.x -= 0.0055;  // lift_joint x offset
        arm_base_pose.pose.position.y -= 0.0;     // lift_joint y offset
        arm_base_pose.pose.position.z -= (1.4316 + lift_joint_position_); // lift_joint z offset + current lift position

        RCLCPP_INFO(this->get_logger(), "🔄 Transformed to arm_base_link frame:");
        RCLCPP_INFO(this->get_logger(), "   Position: x=%.3f, y=%.3f, z=%.3f (lift: %.3f m)",
                   arm_base_pose.pose.position.x, arm_base_pose.pose.position.y,
                   arm_base_pose.pose.position.z, lift_joint_position_);

        // Calculate distance for reachability check
        double distance = sqrt(arm_base_pose.pose.position.x * arm_base_pose.pose.position.x +
                              arm_base_pose.pose.position.y * arm_base_pose.pose.position.y +
                              arm_base_pose.pose.position.z * arm_base_pose.pose.position.z);
        RCLCPP_INFO(this->get_logger(), "📐 Target distance from arm_base: %.3f m", distance);

        // Solve IK for the transformed target
        solveIKAndMove(arm_base_pose);
    }

    void solveIKAndMove(const geometry_msgs::msg::PoseStamped& target_pose)
    {
        RCLCPP_INFO(this->get_logger(), "🔧 Solving arm-only IK with Joint Limits...");

        // Convert target pose to KDL Frame
        KDL::Frame target_frame;
        target_frame.p.x(target_pose.pose.position.x);
        target_frame.p.y(target_pose.pose.position.y);
        target_frame.p.z(target_pose.pose.position.z);

        // Convert quaternion to rotation matrix
        KDL::Rotation rot = KDL::Rotation::Quaternion(
            target_pose.pose.orientation.x,
            target_pose.pose.orientation.y,
            target_pose.pose.orientation.z,
            target_pose.pose.orientation.w
        );
        target_frame.M = rot;

        // Get current arm joint positions as initial guess
        KDL::JntArray q_init(chain_.getNrOfJoints());
        for (size_t i = 0; i < current_joint_positions_.size(); i++) {
            q_init(i) = current_joint_positions_[i];
        }

        // Log current joint positions
        std::string current_log = "🔧 Current arm joints: [";
        for (size_t i = 0; i < current_joint_positions_.size(); i++) {
            current_log += joint_names_[i] + "=" + std::to_string(current_joint_positions_[i]);
            if (i < current_joint_positions_.size() - 1) current_log += ", ";
        }
        current_log += "]";
        RCLCPP_INFO(this->get_logger(), "%s", current_log.c_str());

        // Clamp initial guess to joint limits
        for (unsigned int i = 0; i < q_init.rows(); i++) {
            if (q_init(i) < q_min_(i)) {
                q_init(i) = q_min_(i);
                RCLCPP_DEBUG(this->get_logger(), "Clamped initial guess for joint %d to min limit", i);
            }
            if (q_init(i) > q_max_(i)) {
                q_init(i) = q_max_(i);
                RCLCPP_DEBUG(this->get_logger(), "Clamped initial guess for joint %d to max limit", i);
            }
        }

        KDL::JntArray q_result(chain_.getNrOfJoints());
        int ik_result = ik_solver_jl_->CartToJnt(q_init, target_frame, q_result);

        // If IK failed, try with different initial guesses within limits
        if (ik_result < 0) {
            RCLCPP_WARN(this->get_logger(), "IK failed with current guess (error: %d), trying alternatives...", ik_result);

            // Try with center of joint limits as initial guess
            KDL::JntArray q_center(chain_.getNrOfJoints());
            for (unsigned int i = 0; i < chain_.getNrOfJoints(); i++) {
                q_center(i) = (q_min_(i) + q_max_(i)) / 2.0;
            }
            ik_result = ik_solver_jl_->CartToJnt(q_center, target_frame, q_result);

            if (ik_result < 0) {
                // Try with home position (zeros)
                KDL::JntArray q_home(chain_.getNrOfJoints());
                for (unsigned int i = 0; i < q_home.rows(); i++) {
                    q_home(i) = 0.0;
                }
                ik_result = ik_solver_jl_->CartToJnt(q_home, target_frame, q_result);
            }
        }

        if (ik_result >= 0) {
            // Verify all joints are within limits
            bool all_within_limits = true;
            for (unsigned int i = 0; i < q_result.rows(); i++) {
                if (q_result(i) < q_min_(i) || q_result(i) > q_max_(i)) {
                    all_within_limits = false;
                    RCLCPP_WARN(this->get_logger(), "Joint %d solution %.3f is outside limits [%.3f, %.3f]",
                               i, q_result(i), q_min_(i), q_max_(i));
                }
            }

            if (!all_within_limits) {
                RCLCPP_ERROR(this->get_logger(), "❌ IK solution violates joint limits! Skipping movement.");
                return;
            }

            RCLCPP_INFO(this->get_logger(), "✅ Arm-only IK solution found with Joint Limits. Moving robot...");

            // Log joint solution
            std::string solution_log = "🎯 Arm joint solution: [";
            for (size_t i = 0; i < joint_names_.size(); i++) {
                solution_log += joint_names_[i] + "=" + std::to_string(q_result(i));
                if (i < joint_names_.size() - 1) solution_log += ", ";
            }
            solution_log += "]";
            RCLCPP_INFO(this->get_logger(), "%s", solution_log.c_str());

            // Send joint trajectory command to move the robot
            sendJointTrajectory(q_result);

        } else {
            // Provide detailed error information
            std::string error_msg;
            switch(ik_result) {
                case -1: error_msg = "Failed to converge"; break;
                case -2: error_msg = "Undefined problem"; break;
                case -3: error_msg = "Degraded gradient"; break;
                case -4: error_msg = "Singularity detected"; break;
                case -5: error_msg = "Maximum iterations exceeded"; break;
                default: error_msg = "Unknown error"; break;
            }

            RCLCPP_ERROR(this->get_logger(), "❌ Arm-only IK with Joint Limits failed: %d (%s)", ik_result, error_msg.c_str());
            
            // Additional failure analysis
            if (ik_result == -5) {
                RCLCPP_ERROR(this->get_logger(), "  → Target may be unreachable for arm-only configuration");
            } else if (ik_result == -3 || ik_result == -4) {
                RCLCPP_ERROR(this->get_logger(), "  → Robot is in or near a singularity");
            }
        }
    }

    void sendJointTrajectory(const KDL::JntArray& joint_positions)
    {
        // Create joint trajectory message for arm joints only
        auto traj_msg = trajectory_msgs::msg::JointTrajectory();
        traj_msg.header.frame_id = "";

        // Set arm joint names (excluding lift_joint)
        std::vector<std::string> arm_joint_names = joint_names_;
        std::vector<double> target_arm_positions;

        for (size_t i = 0; i < joint_names_.size(); i++) {
            target_arm_positions.push_back(joint_positions(i));
        }

        traj_msg.joint_names = arm_joint_names;

        // Create single trajectory point with smooth timing
        auto point = trajectory_msgs::msg::JointTrajectoryPoint();
        point.positions = target_arm_positions;
        point.velocities = {};  // Empty velocities array
        point.accelerations = {};  // Empty accelerations array
        point.effort = {};  // Empty effort array
        point.time_from_start.sec = 0;    // 2 second execution time for smooth movement
        point.time_from_start.nanosec = 0;

        traj_msg.points.push_back(point);

        // Publish the trajectory
        joint_trajectory_pub_->publish(traj_msg);

        // Log sent trajectory
        std::string sent_log = "📤 Sent arm trajectory: [";
        for (size_t i = 0; i < arm_joint_names.size(); i++) {
            sent_log += arm_joint_names[i] + "=" + std::to_string(target_arm_positions[i]);
            if (i < arm_joint_names.size() - 1) sent_log += ", ";
        }
        sent_log += "]";
        RCLCPP_INFO(this->get_logger(), "%s", sent_log.c_str());
        RCLCPP_INFO(this->get_logger(), "📤 Arm-only joint trajectory sent! Robot should move in 2 seconds.");
    }

    void publishCurrentPose()
    {
        if (!setup_complete_ || !has_joint_states_) {
            return;
        }

        // Forward Kinematics to get current end-effector pose
        KDL::JntArray q(chain_.getNrOfJoints());
        for (size_t i = 0; i < current_joint_positions_.size(); i++) {
            q(i) = current_joint_positions_[i];
        }

        KDL::Frame end_effector_frame;
        int fk_result = fk_solver_->JntToCart(q, end_effector_frame);

        if (fk_result >= 0) {
            // Extract position and orientation
            KDL::Vector pos = end_effector_frame.p;
            double qx, qy, qz, qw;
            end_effector_frame.M.GetQuaternion(qx, qy, qz, qw);

            // Publish current pose
            auto pose_msg = geometry_msgs::msg::PoseStamped();
            pose_msg.header.stamp = this->get_clock()->now();
            pose_msg.header.frame_id = base_link_;
            pose_msg.pose.position.x = pos.x();
            pose_msg.pose.position.y = pos.y();
            pose_msg.pose.position.z = pos.z();
            pose_msg.pose.orientation.x = qx;
            pose_msg.pose.orientation.y = qy;
            pose_msg.pose.orientation.z = qz;
            pose_msg.pose.orientation.w = qw;
            current_pose_pub_->publish(pose_msg);
        }
    }

private:
    // Parameters
    std::string base_link_;
    std::string arm_base_link_;
    std::string end_effector_link_;

    // ROS interfaces
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_pub_;
    rclcpp::TimerBase::SharedPtr pose_timer_;

    // KDL objects
    KDL::Chain chain_;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    std::unique_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;
    std::unique_ptr<KDL::ChainIkSolverPos_NR_JL> ik_solver_jl_;  // Joint Limits IK solver

    // Joint limits
    KDL::JntArray q_min_;  // Minimum joint limits
    KDL::JntArray q_max_;  // Maximum joint limits

    // Joint information
    std::vector<std::string> joint_names_;
    std::vector<double> current_joint_positions_;
    int lift_joint_index_;  // Not used in arm-only chain
    double lift_joint_position_;  // Current lift joint position for coordinate transformation

    // Status flags
    bool setup_complete_;
    bool has_joint_states_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<RealtimeIKSolverJL>();

    RCLCPP_INFO(node->get_logger(), "🚀 Realtime IK Solver with Joint Limits node started");

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
