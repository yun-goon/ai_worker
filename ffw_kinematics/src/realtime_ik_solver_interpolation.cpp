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

class RealtimeIKSolverInterpolation : public rclcpp::Node
{
public:
    RealtimeIKSolverInterpolation() : Node("realtime_ik_solver_interpolation"),
                                     lift_joint_index_(-1),
                                     setup_complete_(false),
                                     has_joint_states_(false),
                                     max_joint_change_per_step_(0.5), // 0.5 rad (~28.6도) per step
                                     max_distance_threshold_(0.15),   // 15cm 이상이면 interpolation
                                     interpolation_steps_(5)          // 5단계로 나누어 이동
    {
        // Parameters
        this->declare_parameter<std::string>("base_link", "base_link");
        this->declare_parameter<std::string>("arm_base_link", "arm_base_link");
        this->declare_parameter<std::string>("end_effector_link", "arm_r_link7");
        this->declare_parameter<std::string>("target_pose_topic", "/target_pose");
        this->declare_parameter<double>("max_joint_change_per_step", 0.5);
        this->declare_parameter<double>("max_distance_threshold", 0.15);
        this->declare_parameter<int>("interpolation_steps", 5);

        base_link_ = this->get_parameter("base_link").as_string();
        arm_base_link_ = this->get_parameter("arm_base_link").as_string();
        end_effector_link_ = this->get_parameter("end_effector_link").as_string();
        std::string target_pose_topic = this->get_parameter("target_pose_topic").as_string();
        max_joint_change_per_step_ = this->get_parameter("max_joint_change_per_step").as_double();
        max_distance_threshold_ = this->get_parameter("max_distance_threshold").as_double();
        interpolation_steps_ = this->get_parameter("interpolation_steps").as_int();

        RCLCPP_INFO(this->get_logger(), "🚀 Interpolation IK Solver starting...");
        RCLCPP_INFO(this->get_logger(), "Base link: %s", base_link_.c_str());
        RCLCPP_INFO(this->get_logger(), "Arm base link: %s", arm_base_link_.c_str());
        RCLCPP_INFO(this->get_logger(), "End effector link: %s", end_effector_link_.c_str());
        RCLCPP_INFO(this->get_logger(), "📐 Max distance threshold: %.3f m", max_distance_threshold_);
        RCLCPP_INFO(this->get_logger(), "🔧 Max joint change per step: %.3f rad (%.1f°)", 
                   max_joint_change_per_step_, max_joint_change_per_step_ * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(), "📊 Interpolation steps: %d", interpolation_steps_);

        // Subscribers
        robot_description_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/robot_description", rclcpp::QoS(1).transient_local(),
            std::bind(&RealtimeIKSolverInterpolation::robotDescriptionCallback, this, std::placeholders::_1));

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&RealtimeIKSolverInterpolation::jointStateCallback, this, std::placeholders::_1));

        target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            target_pose_topic, 10,
            std::bind(&RealtimeIKSolverInterpolation::targetPoseCallback, this, std::placeholders::_1));

        // Publishers
        joint_trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/leader/joint_trajectory_command_broadcaster_right/joint_trajectory", 10);

        // Timer for interpolation steps
        interpolation_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(300), // 300ms per step
            std::bind(&RealtimeIKSolverInterpolation::interpolationTimerCallback, this));
        interpolation_timer_->cancel(); // Start as cancelled

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
        RCLCPP_INFO(this->get_logger(), "✅ Interpolation IK setup complete!");
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

        // Stop any ongoing interpolation
        if (!interpolation_timer_->is_canceled()) {
            interpolation_timer_->cancel();
            RCLCPP_INFO(this->get_logger(), "🛑 Stopping previous interpolation");
        }

        // target_pose를 arm_base_link 기준으로 변환
        geometry_msgs::msg::PoseStamped arm_base_pose = *msg;
        
        // base_link에서 arm_base_link로 변환 (lift_joint의 static offset + current lift position)
        arm_base_pose.pose.position.x -= (-0.0199);  // lift_joint x offset
        arm_base_pose.pose.position.y -= 0.0;        // lift_joint y offset  
        arm_base_pose.pose.position.z -= (1.4316 + lift_joint_position_); // lift_joint z offset + current lift position
        
        RCLCPP_INFO(this->get_logger(), "🎯 Original pose: [%.3f, %.3f, %.3f]", 
                   msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        RCLCPP_INFO(this->get_logger(), "🔄 Transformed pose: [%.3f, %.3f, %.3f], lift: %.3f", 
                   arm_base_pose.pose.position.x, arm_base_pose.pose.position.y, arm_base_pose.pose.position.z, lift_joint_position_);
        
        solveIKAndMove(arm_base_pose);
    }

    void solveIKAndMove(const geometry_msgs::msg::PoseStamped& target_pose)
    {
        // 현재 end-effector 위치 계산
        KDL::Frame current_ee_frame;
        KDL::JntArray current_joints(chain_.getNrOfJoints());
        for (size_t i = 0; i < current_joint_positions_.size(); i++) {
            current_joints(i) = current_joint_positions_[i];
        }
        
        if (fk_solver_->JntToCart(current_joints, current_ee_frame) < 0) {
            RCLCPP_ERROR(this->get_logger(), "❌ Forward kinematics failed");
            return;
        }
        
        // 목표 위치
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
        
        // 거리 계산
        KDL::Vector diff = target_frame.p - current_ee_frame.p;
        double distance = diff.Norm();
        
        RCLCPP_INFO(this->get_logger(), "📐 Current EE: [%.3f, %.3f, %.3f]", 
                   current_ee_frame.p.x(), current_ee_frame.p.y(), current_ee_frame.p.z());
        RCLCPP_INFO(this->get_logger(), "📐 Movement distance: %.3f m", distance);
        
        // 거리가 크면 interpolation 사용
        if (distance > max_distance_threshold_) {
            RCLCPP_INFO(this->get_logger(), "🔄 Large movement detected! Using interpolation (%.3f > %.3f)", 
                       distance, max_distance_threshold_);
            startInterpolation(current_ee_frame, target_frame);
        } else {
            RCLCPP_INFO(this->get_logger(), "➡️  Small movement, direct IK");
            solveIKDirect(target_frame);
        }
    }

    void startInterpolation(const KDL::Frame& start_frame, const KDL::Frame& target_frame)
    {
        // Interpolation 데이터 설정
        interpolation_start_frame_ = start_frame;
        interpolation_target_frame_ = target_frame;
        interpolation_current_step_ = 0;
        
        RCLCPP_INFO(this->get_logger(), "🚀 Starting interpolation: %d steps", interpolation_steps_);
        
        // Timer 시작
        interpolation_timer_->reset();
    }

    void interpolationTimerCallback()
    {
        interpolation_current_step_++;
        
        if (interpolation_current_step_ > interpolation_steps_) {
            interpolation_timer_->cancel();
            RCLCPP_INFO(this->get_logger(), "✅ Interpolation completed!");
            return;
        }
        
        double t = (double)interpolation_current_step_ / interpolation_steps_;
        
        // 위치 interpolation (linear)
        KDL::Frame intermediate_frame;
        intermediate_frame.p = interpolation_start_frame_.p + 
                              t * (interpolation_target_frame_.p - interpolation_start_frame_.p);
        
        // 회전 interpolation (간단한 linear interpolation)
        double start_z, start_y, start_x, target_z, target_y, target_x;
        interpolation_start_frame_.M.GetEulerZYX(start_z, start_y, start_x);
        interpolation_target_frame_.M.GetEulerZYX(target_z, target_y, target_x);
        
        // 각도 차이 정규화 (-π ~ π)
        auto normalizeAngle = [](double angle) {
            while (angle > M_PI) angle -= 2.0 * M_PI;
            while (angle < -M_PI) angle += 2.0 * M_PI;
            return angle;
        };
        
        double diff_x = normalizeAngle(target_x - start_x);
        double diff_y = normalizeAngle(target_y - start_y);
        double diff_z = normalizeAngle(target_z - start_z);
        
        double interp_x = start_x + t * diff_x;
        double interp_y = start_y + t * diff_y;
        double interp_z = start_z + t * diff_z;
        
        intermediate_frame.M = KDL::Rotation::EulerZYX(interp_z, interp_y, interp_x);
        
        RCLCPP_INFO(this->get_logger(), "🔄 Interpolation step %d/%d (t=%.2f): [%.3f, %.3f, %.3f]", 
                   interpolation_current_step_, interpolation_steps_, t,
                   intermediate_frame.p.x(), intermediate_frame.p.y(), intermediate_frame.p.z());
        
        if (!solveIKDirect(intermediate_frame)) {
            RCLCPP_ERROR(this->get_logger(), "❌ Interpolation failed at step %d", interpolation_current_step_);
            interpolation_timer_->cancel();
        }
    }

    bool solveIKDirect(const KDL::Frame& target_frame)
    {
        KDL::JntArray q_init(chain_.getNrOfJoints());
        for (size_t i = 0; i < current_joint_positions_.size(); i++) {
            q_init(i) = current_joint_positions_[i];
        }
        
        KDL::JntArray q_result(chain_.getNrOfJoints());
        int ik_result = ik_solver_jl_->CartToJnt(q_init, target_frame, q_result);
        
        if (ik_result >= 0) {
            // Joint 변화량 체크
            bool safe_movement = checkJointSafety(q_init, q_result);
            
            if (safe_movement) {
                sendJointTrajectory(q_result);
                return true;
            } else {
                RCLCPP_ERROR(this->get_logger(), "❌ Movement rejected: unsafe joint changes");
                return false;
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "❌ IK failed: %d", ik_result);
            
            // 실패 원인 분석
            if (ik_result == -5) {
                RCLCPP_ERROR(this->get_logger(), "  → Maximum iterations exceeded");
            } else if (ik_result == -3) {
                RCLCPP_ERROR(this->get_logger(), "  → Singularity detected");
            }
            
            return false;
        }
    }

    bool checkJointSafety(const KDL::JntArray& current_q, const KDL::JntArray& target_q)
    {
        bool safe = true;
        double max_change = 0.0;
        
        for (unsigned int i = 0; i < current_q.rows(); i++) {
            double change = std::abs(target_q(i) - current_q(i));
            max_change = std::max(max_change, change);
            
            if (change > max_joint_change_per_step_) {
                RCLCPP_WARN(this->get_logger(), "⚠️ Joint %d change too large: %.3f rad (%.1f°) > %.3f rad", 
                           i, change, change * 180.0 / M_PI, max_joint_change_per_step_);
                safe = false;
            }
            
            // Joint limits 체크
            if (target_q(i) < q_min_(i) || target_q(i) > q_max_(i)) {
                RCLCPP_WARN(this->get_logger(), "⚠️ Joint %d out of limits: %.3f [%.3f, %.3f]", 
                           i, target_q(i), q_min_(i), q_max_(i));
                safe = false;
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "🔧 Max joint change: %.3f rad (%.1f°)", 
                   max_change, max_change * 180.0 / M_PI);
        
        return safe;
    }

    void sendJointTrajectory(const KDL::JntArray& joint_positions)
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
        point.time_from_start.nanosec = 500000000; // 0.5초
        traj_msg.points.push_back(point);
        
        joint_trajectory_pub_->publish(traj_msg);
        
        RCLCPP_INFO(this->get_logger(), "📤 Joint trajectory sent!");
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
    
    // Interpolation members
    double max_joint_change_per_step_;
    double max_distance_threshold_;
    int interpolation_steps_;
    rclcpp::TimerBase::SharedPtr interpolation_timer_;
    KDL::Frame interpolation_start_frame_;
    KDL::Frame interpolation_target_frame_;
    int interpolation_current_step_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RealtimeIKSolverInterpolation>();
    RCLCPP_INFO(node->get_logger(), "🚀 Interpolation IK Solver node started");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
