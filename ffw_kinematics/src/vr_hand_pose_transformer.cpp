#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class VRHandPoseTransformer : public rclcpp::Node
{
public:
    VRHandPoseTransformer() : Node("vr_hand_pose_transformer")
    {
        // Subscribe to VR hand poses
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/vr_hand/right_poses", 10,
            std::bind(&VRHandPoseTransformer::hand_pose_callback, this, std::placeholders::_1));

        // Publisher for target pose to IK solver
        target_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/target_pose", 10);

        // Static transform from zedm_camera_center to arm_base_link
        // Calculated from URDF: arm_base_link -> head_link1 -> head_link2 -> zedm_camera_link -> zedm_camera_center
        // Forward: [0.1132952, -0.00651797, 0.1449406]
        // Inverse (zedm_camera_center -> arm_base_link): [-0.1132952, 0.00651797, -0.1449406]
        transform_zedm_to_arm_ = Eigen::Vector3d(-0.1132952, 0.00651797, -0.1449406);

        RCLCPP_INFO(this->get_logger(), "VR Hand Pose Transformer node started");
    }

private:
    void hand_pose_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        if (msg->poses.empty())
        {
            return;
        }

        // Assume the first pose in the array is the wrist pose
        // (or find the wrist pose based on your VR system's convention)
        const auto& wrist_pose_vr = msg->poses[0];  // Adjust index if needed


        // Extract position from VR pose (Meta Quest/Unity 좌표계)
        Eigen::Vector3d vr_position(
            wrist_pose_vr.position.x,
            wrist_pose_vr.position.y,
            wrist_pose_vr.position.z
        );


        // Extract orientation from VR pose
        Eigen::Quaterniond vr_quaternion(
            wrist_pose_vr.orientation.w,  // w comes first in Eigen
            wrist_pose_vr.orientation.x,
            wrist_pose_vr.orientation.y,
            wrist_pose_vr.orientation.z
        );


    // === VR(Meta Quest/Unity) → ROS 변환 (position) ===
    // ROS.x = -VR.z, ROS.y = -VR.x, ROS.z = VR.y
    Eigen::Vector3d ros_position;
    ros_position.x() = -vr_position.z();
    ros_position.y() = -vr_position.x();
    ros_position.z() = vr_position.y();

    // Transform position from zedm_camera_center frame to arm_base_link frame
    Eigen::Vector3d base_position = ros_position + transform_zedm_to_base_;


    // === VR(Meta Quest/Unity) → ROS 변환 (orientation) ===
    // 변환 행렬: (VR → ROS)
    //   [ 0,  0, -1 ]
    //   [ -1, 0,  0 ]
    //   [ 0,  1,  0 ]
    Eigen::Matrix3d vr_to_ros;
    vr_to_ros <<  0, 0, -1,
             -1, 0, 0,
              0, 1, 0;

    Eigen::Matrix3d vr_rot = vr_quaternion.toRotationMatrix();
    Eigen::Matrix3d ros_rot = vr_to_ros * vr_rot;
    Eigen::Quaterniond arm_quaternion(ros_rot);


    // Create target pose message

    auto target_pose = geometry_msgs::msg::PoseStamped();
    target_pose.header.stamp = this->get_clock()->now();
    target_pose.header.frame_id = "base_link";

    target_pose.pose.position.x = base_position.x();
    target_pose.pose.position.y = base_position.y();
    target_pose.pose.position.z = base_position.z();

    target_pose.pose.orientation.x = arm_quaternion.x();
    target_pose.pose.orientation.y = arm_quaternion.y();
    target_pose.pose.orientation.z = arm_quaternion.z();
    target_pose.pose.orientation.w = arm_quaternion.w();

        // Publish target pose
        target_pose_pub_->publish(target_pose);

        RCLCPP_DEBUG(this->get_logger(),
            "Transformed VR pose: pos=[%.3f, %.3f, %.3f]",
            base_position.x(), base_position.y(), base_position.z());
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
    Eigen::Vector3d transform_zedm_to_base_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<VRHandPoseTransformer>();

    try
    {
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(node->get_logger(), "Exception caught: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}
