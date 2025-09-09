#include <chrono>
#include <cmath>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;

class ImuPublisher : public rclcpp::Node {
public:
  ImuPublisher() : Node("imu_publisher"), t_(0.0) {
    pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/sensors/imu", 50);
    timer_ = this->create_wall_timer(20ms, std::bind(&ImuPublisher::tick, this)); // 50 Hz
  }

private:
  void tick() {
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "imu_link";

    // синтетика: маленькие синус/косинус для угл.скоростей и лин.ускорений
    double a = 0.01 * std::sin(t_);
    double b = 0.01 * std::cos(t_);
    msg.angular_velocity.x = a;
    msg.angular_velocity.y = b;
    msg.angular_velocity.z = -a;

    msg.linear_acceleration.x = 0.1 * std::sin(0.5 * t_);
    msg.linear_acceleration.y = 0.1 * std::cos(0.5 * t_);
    msg.linear_acceleration.z = 9.81; // «гравитация» для вида

    pub_->publish(msg);
    t_ += 0.02; // ~50 Гц шаг времени
  }

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  double t_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuPublisher>());
  rclcpp::shutdown();
  return 0;
}
