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

class RealtimeIKSolverArmOnly : public rclcpp::Node
{
public:
    RealtimeIKSolverArmOnly() : Node("realtime_ik_solver_arm_only"),
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

        RCLCPP_INFO(this->get_logger(), "🚀 Arm-only IK Solver starting...");
        RCLCPP_INFO(this->get_logger(), "Base link: %s", base_link_.c_str());
        RCLCPP_INFO(this->get_logger(), "Arm base link: %s", arm_base_link_.c_str());
        RCLCPP_INFO(this->get_logger(), "End effector link: %s", end_effector_link_.c_str());

        // Subscribers
        robot_description_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/robot_description", rclcpp::QoS(1).transient_local(),
            std::bind(&RealtimeIKSolverArmOnly::robotDescriptionCallback, this, std::placeholders::_1));

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&RealtimeIKSolverArmOnly::jointStateCallback, this, std::placeholders::_1));

        target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            target_pose_topic, 10,
            std::bind(&RealtimeIKSolverArmOnly::targetPoseCallback, this, std::placeholders::_1));

        // Publishers
        joint_trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/leader/joint_trajectory_command_broadcaster_right/joint_trajectory", 10);

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
        // Find lift_joint index in full robot (for joint_states)
        lift_joint_index_ = -1;
        auto all_joints = model.joints_; // name->ptr map
        int idx = 0;
        for (const auto& kv : all_joints) {
            if (kv.first == "lift_joint") {
                lift_joint_index_ = idx;
                break;
            }
            idx++;
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
        RCLCPP_INFO(this->get_logger(), "✅ Arm-only KDL setup complete!");
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
        if (!setup_complete_ || !has_joint_states_) return;
        
        // target_pose를 arm_base_link 기준으로 변환
        // URDF에서 lift_joint origin: xyz="-0.0199 0 1.4316"
        geometry_msgs::msg::PoseStamped arm_base_pose = *msg;
        
        // base_link에서 arm_base_link로 변환 (lift_joint의 static offset + current lift position)
        arm_base_pose.pose.position.x -= (-0.0199);  // lift_joint x offset
        arm_base_pose.pose.position.y -= 0.0;        // lift_joint y offset  
        arm_base_pose.pose.position.z -= (1.4316 + lift_joint_position_); // lift_joint z offset + current lift position
        
        RCLCPP_INFO(this->get_logger(), "🎯 Original pose: [%.3f, %.3f, %.3f]", 
                   msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        RCLCPP_INFO(this->get_logger(), "🔄 Transformed pose: [%.3f, %.3f, %.3f], lift: %.3f", 
                   arm_base_pose.pose.position.x, arm_base_pose.pose.position.y, arm_base_pose.pose.position.z, lift_joint_position_);
        
        // 목표 위치의 거리 계산 (대략적인 도달성 확인)
        double distance = sqrt(arm_base_pose.pose.position.x * arm_base_pose.pose.position.x + 
                              arm_base_pose.pose.position.y * arm_base_pose.pose.position.y + 
                              arm_base_pose.pose.position.z * arm_base_pose.pose.position.z);
        RCLCPP_INFO(this->get_logger(), "📐 Target distance from arm_base: %.3f m", distance);
        
        solveIKAndMove(arm_base_pose);
    }

    void solveIKAndMove(const geometry_msgs::msg::PoseStamped& target_pose)
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
        
        KDL::JntArray q_init(chain_.getNrOfJoints());
        for (size_t i = 0; i < current_joint_positions_.size(); i++) {
            q_init(i) = current_joint_positions_[i];
        }
        
        // 현재 조인트 위치 로그
        std::string joint_log = "🔧 Current joints: [";
        for (size_t i = 0; i < current_joint_positions_.size(); i++) {
            joint_log += std::to_string(current_joint_positions_[i]);
            if (i < current_joint_positions_.size() - 1) joint_log += ", ";
        }
        joint_log += "]";
        RCLCPP_INFO(this->get_logger(), "%s", joint_log.c_str());
        
        KDL::JntArray q_result(chain_.getNrOfJoints());
        int ik_result = ik_solver_jl_->CartToJnt(q_init, target_frame, q_result);
        
        if (ik_result >= 0) {
            // 결과 조인트 위치 로그
            std::string result_log = "✅ IK solution: [";
            for (unsigned int i = 0; i < q_result.rows(); i++) {
                result_log += std::to_string(q_result(i));
                if (i < q_result.rows() - 1) result_log += ", ";
            }
            result_log += "]";
            RCLCPP_INFO(this->get_logger(), "%s", result_log.c_str());
            
            sendJointTrajectory(q_result);
        } else {
            RCLCPP_ERROR(this->get_logger(), "❌ Arm-only IK failed: %d", ik_result);
            
            // 실패 원인 분석
            if (ik_result == -5) {
                RCLCPP_ERROR(this->get_logger(), "  → Maximum iterations exceeded (1000). Target may be unreachable or too far.");
            } else if (ik_result == -3) {
                RCLCPP_ERROR(this->get_logger(), "  → Singularity detected. Robot is in a difficult pose.");
            }
            
            // 더 나은 초기값으로 재시도
            RCLCPP_WARN(this->get_logger(), "🔄 Trying with home position as initial guess...");
            KDL::JntArray q_home(chain_.getNrOfJoints());
            for (unsigned int i = 0; i < q_home.rows(); i++) {
                q_home(i) = 0.0;  // 홈 포지션
            }
            
            int retry_result = ik_solver_jl_->CartToJnt(q_home, target_frame, q_result);
            if (retry_result >= 0) {
                RCLCPP_INFO(this->get_logger(), "✅ IK succeeded with home position initial guess!");
                sendJointTrajectory(q_result);
            } else {
                RCLCPP_ERROR(this->get_logger(), "❌ IK failed even with home position: %d", retry_result);
            }
        }
    }

    void sendJointTrajectory(const KDL::JntArray& joint_positions)
    {
        auto traj_msg = trajectory_msgs::msg::JointTrajectory();
        std::vector<std::string> arm_joint_names = joint_names_;
        std::vector<double> target_arm_positions;
        for (size_t i = 0; i < joint_names_.size(); i++) {
            target_arm_positions.push_back(joint_positions(i));
        }
        
        // lift_joint는 arm_r_controller가 제어하지 않으므로 제외
        // arm 조인트만 포함
        traj_msg.joint_names = arm_joint_names;
        
        auto point = trajectory_msgs::msg::JointTrajectoryPoint();
        point.positions = target_arm_positions;
        point.time_from_start.sec = 0;
        point.time_from_start.nanosec = 0;
        traj_msg.points.push_back(point);
        
        joint_trajectory_pub_->publish(traj_msg);
        
        // 전송된 조인트 정보 로그
        std::string sent_joints = "📤 Sent joints: [";
        for (size_t i = 0; i < arm_joint_names.size(); i++) {
            sent_joints += arm_joint_names[i] + "=" + std::to_string(target_arm_positions[i]);
            if (i < arm_joint_names.size() - 1) sent_joints += ", ";
        }
        sent_joints += "]";
        RCLCPP_INFO(this->get_logger(), "%s", sent_joints.c_str());
        RCLCPP_INFO(this->get_logger(), "📤 Arm-only joint trajectory sent!");
    }

private:
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
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RealtimeIKSolverArmOnly>();
    RCLCPP_INFO(node->get_logger(), "🚀 Arm-only IK Solver node started");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
