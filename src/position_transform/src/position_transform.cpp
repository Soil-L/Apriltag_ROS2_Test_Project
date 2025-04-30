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
            // 获取从world到camera_color_optical_frame的变换
            auto world_to_camera = tf_buffer_.lookupTransform(
                "world", "camera_color_optical_frame",
                tf2::TimePointZero);
            
            // 获取从camera_color_optical_frame到object的变换
            auto camera_to_object = tf_buffer_.lookupTransform(
                "camera_color_optical_frame", "object",
                tf2::TimePointZero);
            
            // 将消息转换为tf2::Transform
            tf2::Transform tf_world_to_camera, tf_camera_to_object;
            tf2::fromMsg(world_to_camera.transform, tf_world_to_camera);
            tf2::fromMsg(camera_to_object.transform, tf_camera_to_object);
            
            // 计算组合变换
            tf2::Transform tf_world_to_object = tf_world_to_camera * tf_camera_to_object;
            
            // 提取位置和旋转
            auto position = tf_world_to_object.getOrigin();
            auto rotation = tf_world_to_object.getRotation();
            
            RCLCPP_INFO(this->get_logger(), "Object in world frame:");
            RCLCPP_INFO(this->get_logger(), "  Position: [%.3f, %.3f, %.3f]",
                       position.x(), position.y(), position.z());
            RCLCPP_INFO(this->get_logger(), "  Orientation: [%.3f, %.3f, %.3f, %.3f]",
                       rotation.x(), rotation.y(), rotation.z(), rotation.w());
            
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