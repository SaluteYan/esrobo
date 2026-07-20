#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include "WebFTSensorAdapterCobot.h"
#include <memory>
#include <string>
#include <QCoreApplication>
#include <QThread>

// 全局 Qt 应用对象（Qt 要求一个进程只能有一个 QCoreApplication）
std::unique_ptr<QCoreApplication> qt_app;

// 自定义观察者类，用于接收传感器数据并发布
class ROS2FTDataObserver : public ForceSensorStreamObserver {
public:
    ROS2FTDataObserver(rclcpp::Node::SharedPtr node) : node_(node) {
        // 创建六维力数据发布者 (WrenchStamped 包含力和力矩，带时间戳和帧ID)
        ft_pub_ = node_->create_publisher<geometry_msgs::msg::WrenchStamped>(
            "ft_sensor/data", 10);
        // 创建连接状态发布者
        status_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
            "ft_sensor/connected", 10);
    }

    // 重写数据更新回调函数
    void onForceSensorDataStreamUpdate(std::vector<double> &_ft) override {
        if (_ft.size() != 6) {
            RCLCPP_WARN(node_->get_logger(), "Invalid FT data size: %ld", _ft.size());
            return;
        }

        // 构造 WrenchStamped 消息
        geometry_msgs::msg::WrenchStamped msg;
        msg.header.stamp = node_->get_clock()->now();
        msg.header.frame_id = "ft_sensor_link"; // 根据实际硬件修改帧ID

        // 力数据 (x,y,z)
        msg.wrench.force.x = _ft[0];
        msg.wrench.force.y = _ft[1];
        msg.wrench.force.z = _ft[2];

        // 力矩数据 (x,y,z)
        msg.wrench.torque.x = _ft[3];
        msg.wrench.torque.y = _ft[4];
        msg.wrench.torque.z = _ft[5];

        // 发布数据
        ft_pub_->publish(msg);
    }

    // 发布连接状态
    void publishConnectionStatus(bool connected) {
        std_msgs::msg::Bool status_msg;
        status_msg.data = connected;
        status_pub_->publish(status_msg);
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr ft_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_pub_;
};

int main(int argc, char *argv[]) {
	
    // 初始化 ROS2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("ft_sensor_node");

    // 声明参数 (支持通过命令行/配置文件修改)
    node->declare_parameter<std::string>("sensor_type", "serial"); // tcp/serial
    node->declare_parameter<std::string>("ip", "127.0.0.1");    // TCP IP
    node->declare_parameter<std::string>("port", "/dev/ttyUSB1");      // TCP端口/串口设备名
    node->declare_parameter<bool>("long_data_format", true);   // 长数据格式标志

    // 获取参数
    std::string sensor_type = node->get_parameter("sensor_type").as_string();
    std::string ip = node->get_parameter("ip").as_string();
    std::string port = node->get_parameter("port").as_string();
    bool long_data = node->get_parameter("long_data_format").as_bool();
    
    qt_app = std::make_unique<QCoreApplication>(argc, argv);

    // 创建传感器适配器和ROS2观察者
    auto ft_adapter = std::make_shared<WebFTSensorAdapterCobot>(long_data);
    auto ft_observer = std::make_shared<ROS2FTDataObserver>(node);

    // 绑定观察者
    ft_adapter->attach(ft_observer);

    // 配置传感器
    if (sensor_type == "tcp") {
        // TCP模式
        ft_adapter->setIp(ip);
        ft_adapter->setPort(port);
        RCLCPP_INFO(node->get_logger(), "Configured TCP sensor: %s:%s", ip.c_str(), port.c_str());
    } else if (sensor_type == "serial") {
        // 串口模式
        ft_adapter->setPort(port);
        RCLCPP_INFO(node->get_logger(), "Configured Serial sensor: %s", port.c_str());
    } else {
        RCLCPP_ERROR(node->get_logger(), "Invalid sensor type: %s (must be tcp/serial)", sensor_type.c_str());
        rclcpp::shutdown();
        return -1;
    }

    // 启动传感器
    if (ft_adapter->start()) {
        RCLCPP_INFO(node->get_logger(), "FT sensor started successfully");
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to start FT sensor");
        rclcpp::shutdown();
        return -1;
    }
    
    std::thread *ros_thread = new std::thread([=](){
    
    	rclcpp::Rate rate(10); // 10Hz发布状态
    	while (rclcpp::ok()) {
		    // 发布连接状态
		    ft_observer->publishConnectionStatus(ft_adapter->getStatus());
		    // 处理ROS2消息回调
		    rclcpp::spin_some(node);
		    rate.sleep();
    	}
    	qt_app->quit();
    });

    qt_app->exec();
    rclcpp::shutdown();
    RCLCPP_INFO(node->get_logger(), "FT sensor stopped");
    return 0;
}
