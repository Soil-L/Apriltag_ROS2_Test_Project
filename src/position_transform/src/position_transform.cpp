#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  // 必须包含这个头文件
#include <tf2/LinearMath/Transform.h>

class ObjectPositionCalculator : public rclcpp::Node
{
public:
    ObjectPositionCalculator()
    : Node("object_position_calculator"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
    {
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ObjectPositionCalculator::calculatePosition, this));
    }

private:
    void calculatePosition()
    {
        try {
            // 获取从 world 到 camera_color_optical_frame 的变换
            auto world_to_camera = tf_buffer_.lookupTransform(
                "world", "camera_color_optical_frame",
                tf2::TimePointZero);
            
            // 获取从 camera_color_optical_frame 到 object 的变换
            auto camera_to_object = tf_buffer_.lookupTransform(
                "camera_color_optical_frame", "object",
                tf2::TimePointZero);
            
            // 将消息转换为 tf2::Transform
            tf2::Transform tf_world_to_camera, tf_camera_to_object;
            tf2::fromMsg(world_to_camera.transform, tf_world_to_camera);
            tf2::fromMsg(camera_to_object.transform, tf_camera_to_object);
            
            // 计算 world → object 的变换
            tf2::Transform tf_world_to_object = tf_world_to_camera * tf_camera_to_object;
            
            // 计算 object → world 的变换（即 world → object 的逆矩阵）
            tf2::Transform tf_object_to_world = tf_world_to_object.inverse();
            
            // 提取变换矩阵（4×4 齐次矩阵）
            tf2::Matrix3x3 rotation = tf_object_to_world.getBasis();
            tf2::Vector3 translation = tf_object_to_world.getOrigin();
            
            // 打印变换矩阵（可以用于后续计算）
            RCLCPP_INFO(this->get_logger(), "Object → World Transform Matrix:");
            RCLCPP_INFO(this->get_logger(), "  [%.3f, %.3f, %.3f, %.3f]",
                       rotation[0][0], rotation[0][1], rotation[0][2], translation.x());
            RCLCPP_INFO(this->get_logger(), "  [%.3f, %.3f, %.3f, %.3f]",
                       rotation[1][0], rotation[1][1], rotation[1][2], translation.y());
            RCLCPP_INFO(this->get_logger(), "  [%.3f, %.3f, %.3f, %.3f]",
                       rotation[2][0], rotation[2][1], rotation[2][2], translation.z());
            RCLCPP_INFO(this->get_logger(), "  [%.3f, %.3f, %.3f, %.3f]",
                       0.0, 0.0, 0.0, 1.0);
            
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF error: %s", ex.what());
        }
    }

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectPositionCalculator>());
    rclcpp::shutdown();
    return 0;
}