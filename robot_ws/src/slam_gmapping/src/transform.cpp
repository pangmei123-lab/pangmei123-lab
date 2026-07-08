#include "slam_gmapping/transform.h"
#include "tf2_ros/create_timer_ros.h"
#include <cmath>

using std::placeholders::_1;

Transform::Transform() : Node("transform_node"), transform_thread_(nullptr) {
    // 声明参数并提供默认值
    this->declare_parameter<std::string>("parents_frame", "odom");
    this->declare_parameter<std::string>("child_frame", "lidar_laser_sensor");
    this->declare_parameter<double>("x", 0.0);
    this->declare_parameter<double>("y", 0.0);
    this->declare_parameter<double>("z", 0.0);
    this->declare_parameter<double>("roll", 0.0);
    this->declare_parameter<double>("pitch", 0.0);
    this->declare_parameter<double>("yaw", 0.0);

    // 获取参数值
    this->get_parameter_or<std::string>("parents_frame", parents_frame, "odom");
    this->get_parameter_or<std::string>("child_frame", child_frame, "lidar_laser_sensor");
    this->get_parameter_or<double>("x", x, 0.0);
    this->get_parameter_or<double>("y", y, 0.0);
    this->get_parameter_or<double>("z", z, 0.0);
    this->get_parameter_or<double>("roll", roll, 0.0);
    this->get_parameter_or<double>("pitch", pitch, 0.0);
    this->get_parameter_or<double>("yaw", yaw, 0.0);

    // 将角度从度转换为弧度
    yaw = yaw * M_PI / 180.0;
    pitch = pitch * M_PI / 180.0;
    roll = roll * M_PI / 180.0;

    // 计算四元数
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    rotation_w = cy * cp * cr + sy * sp * sr;
    rotation_x = cy * cp * sr - sy * sp * cr;
    rotation_y = sy * cp * sr + cy * sp * cr;
    rotation_z = sy * cp * cr - cy * sp * sr;

    // 初始化 TF 缓冲区和广播器
    buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(),
        get_node_timers_interface());
    buffer_->setCreateTimerInterface(timer_interface);

    node_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});
    tfB_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

    // 启动发布线程
    transform_thread_ = std::make_shared<std::thread>(std::bind(&Transform::publishLoop, this));
}

void Transform::publishLoop() {
    rclcpp::Rate rate(1.0 / 0.05); // 20 Hz
    while (rclcpp::ok()) {
        publishTransform();
        rate.sleep();
    }
}

void Transform::publishTransform() {
    rclcpp::Time tf_expiration = get_clock()->now() + rclcpp::Duration(0, 50000000); // 50 ms
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = parents_frame;
    transform.header.stamp = tf_expiration;
    transform.child_frame_id = child_frame;
    transform.transform.translation.x = x;
    transform.transform.translation.y = y;
    transform.transform.translation.z = z;
    transform.transform.rotation.w = rotation_w;
    transform.transform.rotation.x = rotation_x;
    transform.transform.rotation.y = rotation_y;
    transform.transform.rotation.z = rotation_z;

    try {
        tfB_->sendTransform(transform);
    } catch (tf2::LookupException &te) {
        RCLCPP_INFO(this->get_logger(), te.what());
    }
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto transform = std::make_shared<Transform>();
    rclcpp::spin(transform);
    return 0;
}