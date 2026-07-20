#ifndef EROB_CANOPEN_DRIVER_HPP
#define EROB_CANOPEN_DRIVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include <thread>
#include <vector>
#include <linux/can.h>

class ERobCanopenDriver : public rclcpp::Node
{
public:
  explicit ERobCanopenDriver();
  ~ERobCanopenDriver();

private:
  int can_fd_;
  std::vector<int> online_nodes_;  // 自动扫描到的ID

  // ==========================
  // 新增：自动扫描 CANopen 节点
  // ==========================
  void scan_nodes();

  bool sdo_write(int node, uint16_t index, uint8_t sub, uint32_t value);
  bool sdo_read(int node, uint16_t index, uint8_t sub);
  void send_sync();
  void send_pdo(int node, uint32_t value);
  void init_all_with_brake_release();
  void can_receive_thread();

  void callback_pos(const std_msgs::msg::UInt32MultiArray::SharedPtr msg);
  void callback_vel(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void callback_tor(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr sub_pos_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_vel_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_tor_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_state_;

  std::thread recv_thread_;
  bool running_;
};

#endif
