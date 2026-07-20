#include "rclcpp/rclcpp.hpp"
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

class URDirectControlNode : public rclcpp::Node
{
public:
    URDirectControlNode() : Node("ur_direct_control")
    {
        robot_ip_ = "192.168.1.100"; // 改成你的UR IP
        dashboard_port_ = 29999;

        RCLCPP_INFO(this->get_logger(), "✅ UR 直连控制节点启动");
        timer_ = this->create_wall_timer(1000ms, std::bind(&URDirectControlNode::auto_control, this));
    }

private:
    std::string robot_ip_;
    int dashboard_port_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string send_dashboard_cmd(const std::string& cmd)
    {
        int sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0) return "SOCK_ERR";

        struct sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(dashboard_port_);
        inet_pton(AF_INET, robot_ip_.c_str(), &addr.sin_addr);

        if (connect(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            close(sock);
            return "CONNECT_ERR";
        }

        send(sock, cmd.c_str(), cmd.length(), 0);
        char buf[1024];
        int n = recv(sock, buf, sizeof(buf)-1, 0);
        close(sock);

        if (n > 0) {
            buf[n] = 0;
            return std::string(buf);
        }
        return "NO_RESP";
    }

    void auto_control()
    {
        std::string resp = send_dashboard_cmd("robotmode\n");
        RCLCPP_INFO(this->get_logger(), "Robot mode: %s", resp.c_str());

        if (resp.find("POWER_OFF") != std::string::npos) {
            RCLCPP_WARN(this->get_logger(), "→ 上电");
            send_dashboard_cmd("power on\n");
            std::this_thread::sleep_for(2s);
            return;
        }

        if (resp.find("IDLE") != std::string::npos) {
            RCLCPP_WARN(this->get_logger(), "→ 松刹车");
            send_dashboard_cmd("brake release\n");
            std::this_thread::sleep_for(1s);
            return;
        }

        std::string safety = send_dashboard_cmd("safetymode\n");
        if (safety.find("PROTECTIVE_STOP") != std::string::npos) {
            RCLCPP_WARN(this->get_logger(), "→ 复位保护停止");
            send_dashboard_cmd("unlock protective stop\n");
            std::this_thread::sleep_for(1s);
            return;
        }

        if (resp.find("RUNNING") != std::string::npos) {
            RCLCPP_INFO(this->get_logger(), "→ 进入远程控制");
            send_dashboard_cmd("remote control\n");
        }

        RCLCPP_INFO(this->get_logger(), "✅ 机械臂已就绪");
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<URDirectControlNode>());
    rclcpp::shutdown();
    return 0;
}
