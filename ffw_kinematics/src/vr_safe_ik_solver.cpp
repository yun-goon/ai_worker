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
#include <cmath>
#include <limits>

class VRSafeIKSolver : public rclcpp::Node
{
public:
    VRSafeIKSolver() : Node("vr_safe_ik_solver"),
                       lift_joint_index_(-1),
                       setup_complete_(false),
                       has_joint_states_(false),
                       max_joint_change_per_update_(0.3),  // 매우 보수적: 17도 제한
                       min_update_interval_ms_(50),        // 최소 50ms 간격
                       position_smoothing_factor_(0.8),    // 위치 smoothing
                       last_update_time_(0)
    {
        // Parameters
        this->declare_parameter<std::string>("base_link", "base_link");
        this->declare_parameter<std::string>("arm_base_link", "arm_base_link");
        this->declare_parameter<std::string>("end_effector_link", "arm_r_link7");
        this->declare_parameter<std::string>("target_pose_topic", "/target_pose");
        this->declare_parameter<double>("max_joint_change_per_update", 0.3);
        this->declare_parameter<int>("min_update_interval_ms", 50);
        this->declare_parameter<double>("position_smoothing_factor", 0.8);

        base_link_ = this->get_parameter("base_link").as_string();
        arm_base_link_ = this->get_parameter("arm_base_link").as_string();
        end_effector_link_ = this->get_parameter("end_effector_link").as_string();
        std::string target_pose_topic = this->get_parameter("target_pose_topic").as_string();
        max_joint_change_per_update_ = this->get_parameter("max_joint_change_per_update").as_double();
        min_update_interval_ms_ = this->get_parameter("min_update_interval_ms").as_int();
        position_smoothing_factor_ = this->get_parameter("position_smoothing_factor").as_double();

        RCLCPP_INFO(this->get_logger(), "🥽 VR Safe IK Solver starting...");
        RCLCPP_INFO(this->get_logger(), "Base link: %s", base_link_.c_str());
        RCLCPP_INFO(this->get_logger(), "Arm base link: %s", arm_base_link_.c_str());
        RCLCPP_INFO(this->get_logger(), "End effector link: %s", end_effector_link_.c_str());
        RCLCPP_INFO(this->get_logger(), "🔧 Max joint change per update: %.3f rad (%.1f°)", 
                   max_joint_change_per_update_, max_joint_change_per_update_ * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(), "⏱️  Min update interval: %d ms", min_update_interval_ms_);
        RCLCPP_INFO(this->get_logger(), "🎯 Position smoothing factor: %.2f", position_smoothing_factor_);

        // Subscribers
        robot_description_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/robot_description", rclcpp::QoS(1).transient_local(),
            std::bind(&VRSafeIKSolver::robotDescriptionCallback, this, std::placeholders::_1));

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&VRSafeIKSolver::jointStateCallback, this, std::placeholders::_1));

        target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            target_pose_topic, 10,
            std::bind(&VRSafeIKSolver::targetPoseCallback, this, std::placeholders::_1));

        // Publishers
        joint_trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/leader/joint_trajectory_command_broadcaster_right/joint_trajectory", 10);

        // 초기화
        last_target_pose_.pose.position.x = 0.0;
        last_target_pose_.pose.position.y = 0.0;
        last_target_pose_.pose.position.z = 0.0;
        last_target_pose_.pose.orientation.w = 1.0;
        
        smoothed_target_pose_ = last_target_pose_;

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
    }

private:
    void robotDescriptionCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        processRobotDescription(msg->data);
    }

    void processRobotDescription(const std::string& robot_description)
    {
        urdf::Model model;
        if (!model.initString(robot_description)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF");
            return;
        }
        KDL::Tree tree;
        if (!kdl_parser::treeFromUrdfModel(model, tree)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create KDL tree from URDF");
            return;
        }
        // Chain: arm_base_link_ ~ end_effector_link_
        if (!tree.getChain(arm_base_link_, end_effector_link_, chain_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to extract chain from %s to %s", arm_base_link_.c_str(), end_effector_link_.c_str());
            return;
        }
        // Extract joint names
        joint_names_.clear();
        for (unsigned int i = 0; i < chain_.getNrOfSegments(); i++) {
            KDL::Segment segment = chain_.getSegment(i);
            if (segment.getJoint().getType() != KDL::Joint::None) {
                joint_names_.push_back(segment.getJoint().getName());
            }
        }
        // Setup joint limits
        unsigned int num_joints = chain_.getNrOfJoints();
        q_min_.resize(num_joints);
        q_max_.resize(num_joints);
        for (size_t i = 0; i < joint_names_.size(); i++) {
            auto joint_ptr = model.getJoint(joint_names_[i]);
            if (joint_ptr && joint_ptr->limits) {
                q_min_(i) = joint_ptr->limits->lower;
                q_max_(i) = joint_ptr->limits->upper;
            } else {
                q_min_(i) = -3.14159;
                q_max_(i) = 3.14159;
            }
        }
        fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
        ik_vel_solver_ = std::make_unique<KDL::ChainIkSolverVel_pinv>(chain_);
        ik_solver_jl_ = std::make_unique<KDL::ChainIkSolverPos_NR_JL>(chain_, q_min_, q_max_, *fk_solver_, *ik_vel_solver_, 1000, 1e-6);
        setup_complete_ = true;
        RCLCPP_INFO(this->get_logger(), "✅ VR Safe IK setup complete!");
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // Find lift_joint position
        lift_joint_position_ = 0.0;
        for (size_t i = 0; i < msg->name.size(); i++) {
            if (msg->name[i] == "lift_joint") {
                lift_joint_position_ = msg->position[i];
            }
        }
        // Extract arm joint positions
        current_joint_positions_.assign(joint_names_.size(), 0.0);
        for (size_t i = 0; i < joint_names_.size(); i++) {
            for (size_t j = 0; j < msg->name.size(); j++) {
                if (msg->name[j] == joint_names_[i] && j < msg->position.size()) {
                    current_joint_positions_[i] = msg->position[j];
                    break;
                }
            }
        }
        has_joint_states_ = true;
    }

    void targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (!setup_complete_ || !has_joint_states_) {
            RCLCPP_WARN(this->get_logger(), "⚠️  Setup not complete or no joint states received");
            return;
        }

        // Rate limiting check
        auto current_time = this->get_clock()->now().nanoseconds() / 1000000; // ms
        if (current_time - last_update_time_ < min_update_interval_ms_) {
            return; // Skip this update
        }
        last_update_time_ = current_time;

        // Position smoothing
        geometry_msgs::msg::PoseStamped smoothed_pose = applySmoothingFilter(*msg);

        // Transform pose from base_link to arm_base_link frame
        geometry_msgs::msg::PoseStamped arm_base_pose = smoothed_pose;
        
        // base_link에서 arm_base_link로 변환 (lift_joint의 static offset + current lift position)
        arm_base_pose.pose.position.x -= (-0.0199);  // lift_joint x offset
        arm_base_pose.pose.position.y -= 0.0;        // lift_joint y offset  
        arm_base_pose.pose.position.z -= (1.4316 + lift_joint_position_); // lift_joint z offset + current lift position
        
        RCLCPP_DEBUG(this->get_logger(), "🥽 VR target: [%.3f, %.3f, %.3f] -> [%.3f, %.3f, %.3f]", 
                   msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
                   arm_base_pose.pose.position.x, arm_base_pose.pose.position.y, arm_base_pose.pose.position.z);
        
        solveVRSafeIK(arm_base_pose);
    }

    geometry_msgs::msg::PoseStamped applySmoothingFilter(const geometry_msgs::msg::PoseStamped& new_pose)
    {
        geometry_msgs::msg::PoseStamped result = new_pose;
        
        // Position smoothing using exponential moving average
        result.pose.position.x = position_smoothing_factor_ * smoothed_target_pose_.pose.position.x + 
                                (1.0 - position_smoothing_factor_) * new_pose.pose.position.x;
        result.pose.position.y = position_smoothing_factor_ * smoothed_target_pose_.pose.position.y + 
                                (1.0 - position_smoothing_factor_) * new_pose.pose.position.y;
        result.pose.position.z = position_smoothing_factor_ * smoothed_target_pose_.pose.position.z + 
                                (1.0 - position_smoothing_factor_) * new_pose.pose.position.z;
        
        // Keep orientation as is for now (could add quaternion slerp later)
        result.pose.orientation = new_pose.pose.orientation;
        
        smoothed_target_pose_ = result;
        return result;
    }

    void solveVRSafeIK(const geometry_msgs::msg::PoseStamped& target_pose)
    {
        KDL::Frame target_frame;
        target_frame.p.x(target_pose.pose.position.x);
        target_frame.p.y(target_pose.pose.position.y);
        target_frame.p.z(target_pose.pose.position.z);
        KDL::Rotation rot = KDL::Rotation::Quaternion(
            target_pose.pose.orientation.x,
            target_pose.pose.orientation.y,
            target_pose.pose.orientation.z,
            target_pose.pose.orientation.w
        );
        target_frame.M = rot;
        
        // Use multiple initial guesses to find the best solution
        std::vector<KDL::JntArray> initial_guesses = generateSmartInitialGuesses();
        
        KDL::JntArray best_solution;
        double best_movement_cost = std::numeric_limits<double>::max();
        bool found_solution = false;
        
        for (const auto& q_init : initial_guesses) {
            KDL::JntArray q_result(chain_.getNrOfJoints());
            int ik_result = ik_solver_jl_->CartToJnt(q_init, target_frame, q_result);
            
            if (ik_result >= 0) {
                // Calculate movement cost (how much joints need to move)
                double movement_cost = calculateMovementCost(q_result);
                
                // Check if this solution is safe
                if (isSafeSolution(q_result) && movement_cost < best_movement_cost) {
                    best_solution = q_result;
                    best_movement_cost = movement_cost;
                    found_solution = true;
                }
            }
        }
        
        if (found_solution) {
            RCLCPP_DEBUG(this->get_logger(), "✅ VR Safe IK found solution (cost: %.3f)", best_movement_cost);
            sendSafeJointTrajectory(best_solution);
        } else {
            RCLCPP_DEBUG(this->get_logger(), "⚠️  VR Safe IK: No safe solution found, ignoring command");
            // VR에서는 위험한 동작보다는 무시하는 것이 낫습니다
        }
    }

    std::vector<KDL::JntArray> generateSmartInitialGuesses()
    {
        std::vector<KDL::JntArray> guesses;
        
        // 1. Current position (most likely to be close)
        KDL::JntArray current_guess(chain_.getNrOfJoints());
        for (size_t i = 0; i < current_joint_positions_.size(); i++) {
            current_guess(i) = current_joint_positions_[i];
        }
        guesses.push_back(current_guess);
        
        // 2. Small perturbations around current position
        for (int perturbation = 0; perturbation < 4; perturbation++) {
            KDL::JntArray perturbed_guess = current_guess;
            double angle = perturbation * M_PI / 2.0; // 0, 90, 180, 270 degrees
            
            // Slightly perturb some joints
            for (unsigned int i = 0; i < perturbed_guess.rows(); i++) {
                double small_change = 0.1 * sin(angle + i); // Small perturbation
                perturbed_guess(i) = current_guess(i) + small_change;
                
                // Clamp to limits
                perturbed_guess(i) = std::max(q_min_(i), std::min(q_max_(i), perturbed_guess(i)));
            }
            guesses.push_back(perturbed_guess);
        }
        
        // 3. Center position (as backup)
        KDL::JntArray center_guess(chain_.getNrOfJoints());
        for (unsigned int i = 0; i < center_guess.rows(); i++) {
            center_guess(i) = (q_min_(i) + q_max_(i)) / 2.0;
        }
        guesses.push_back(center_guess);
        
        return guesses;
    }

    double calculateMovementCost(const KDL::JntArray& target_joints)
    {
        double total_cost = 0.0;
        
        for (unsigned int i = 0; i < target_joints.rows(); i++) {
            double change = std::abs(target_joints(i) - current_joint_positions_[i]);
            
            // Higher cost for larger changes, with exponential penalty
            total_cost += change * change;
            
            // Extra penalty for very large changes
            if (change > max_joint_change_per_update_) {
                total_cost += 1000.0; // Big penalty
            }
        }
        
        return total_cost;
    }

    bool isSafeSolution(const KDL::JntArray& target_joints)
    {
        // Check joint limits
        for (unsigned int i = 0; i < target_joints.rows(); i++) {
            if (target_joints(i) < q_min_(i) || target_joints(i) > q_max_(i)) {
                return false;
            }
        }
        
        // Check maximum change per update
        for (unsigned int i = 0; i < target_joints.rows(); i++) {
            double change = std::abs(target_joints(i) - current_joint_positions_[i]);
            if (change > max_joint_change_per_update_) {
                RCLCPP_DEBUG(this->get_logger(), "Joint %d change too large: %.3f > %.3f", 
                           i, change, max_joint_change_per_update_);
                return false;
            }
        }
        
        return true;
    }

    void sendSafeJointTrajectory(const KDL::JntArray& joint_positions)
    {
        auto traj_msg = trajectory_msgs::msg::JointTrajectory();
        std::vector<std::string> arm_joint_names = joint_names_;
        std::vector<double> target_arm_positions;
        for (size_t i = 0; i < joint_names_.size(); i++) {
            target_arm_positions.push_back(joint_positions(i));
        }
        
        traj_msg.joint_names = arm_joint_names;
        
        auto point = trajectory_msgs::msg::JointTrajectoryPoint();
        point.positions = target_arm_positions;
        point.time_from_start.sec = 0;
        point.time_from_start.nanosec = 100000000; // 0.1초 (빠른 응답)
        traj_msg.points.push_back(point);
        
        joint_trajectory_pub_->publish(traj_msg);
    }

private:
    // Basic members
    std::string base_link_;
    std::string arm_base_link_;
    std::string end_effector_link_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_pub_;
    KDL::Chain chain_;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    std::unique_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;
    std::unique_ptr<KDL::ChainIkSolverPos_NR_JL> ik_solver_jl_;
    KDL::JntArray q_min_;
    KDL::JntArray q_max_;
    std::vector<std::string> joint_names_;
    std::vector<double> current_joint_positions_;
    int lift_joint_index_;
    double lift_joint_position_;
    bool setup_complete_;
    bool has_joint_states_;
    
    // VR-specific members
    double max_joint_change_per_update_;  // Maximum joint change per update
    int min_update_interval_ms_;          // Minimum time between updates
    double position_smoothing_factor_;    // Position smoothing factor
    int64_t last_update_time_;           // Last update timestamp
    geometry_msgs::msg::PoseStamped last_target_pose_;     // Last target pose
    geometry_msgs::msg::PoseStamped smoothed_target_pose_; // Smoothed target pose
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VRSafeIKSolver>();
    RCLCPP_INFO(node->get_logger(), "🥽 VR Safe IK Solver node started");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
