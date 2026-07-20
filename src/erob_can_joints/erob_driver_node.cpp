#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <thread>
#include <algorithm>
#include <map>
#include <vector>

using namespace std::chrono_literals;

// ==============================
// 官方私有协议指令表
// ==============================
#define CMD_SCAN             0x008A

#define CMD_READ_POS         0x0002
#define CMD_READ_VEL         0x0005
#define CMD_READ_CURRENT      0x0008
#define CMD_READ_TEMP         0x0026
#define CMD_READ_ERROR        0x001F

#define CMD_MOTOR_ENABLE      0x0100
#define CMD_BRAKE_RELEASE     0x0188
#define CMD_BRAKE_LOCK        0x0189
#define CMD_CLEAR_ERROR       0x004E
#define CMD_MOTOR_START       0x0083
#define CMD_MOTOR_STOP        0x0084

#define CMD_CONTROL_MODE      0x008D
#define CMD_PROFILE_ACC       0x0088
#define CMD_PROFILE_DEC       0x0089
#define CMD_PROFILE_VEL       0x008A
#define CMD_TARGET_POS        0x0086
#define CMD_TARGET_VEL        0x01FE
#define CMD_TARGET_TORQUE     0x01FC


class ERoboDriver : public rclcpp::Node
{
public:
    ERoboDriver() : Node("erobo_driver")
    {
        // CAN 初始化
        can_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        fcntl(can_fd_, F_SETFL, O_NONBLOCK);

        struct ifreq ifr;
        strcpy(ifr.ifr_name, "can_waist");
        ioctl(can_fd_, SIOCGIFINDEX, &ifr);

        struct sockaddr_can addr{};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        bind(can_fd_, (struct sockaddr*)&addr, sizeof(addr));

        // 扫描关节
        scan_joints();
        if (joint_ids_.empty()) {
            RCLCPP_FATAL(get_logger(), "未找到关节，驱动退出");
            return;
        }

        // 初始化数据
        for (int id : joint_ids_) {
            pos_[id] = 0.0f;
            vel_[id] = 0.0f;
            cur_[id] = 0.0f;
            temp_[id] = 0.0f;
            error_[id] = 0;
        }

        // ROS 接口
        pub_state_ = create_publisher<std_msgs::msg::Float32MultiArray>("/erobo/joint_states", 10);
        pub_error_ = create_publisher<std_msgs::msg::UInt8MultiArray>("/erobo/joint_errors", 10);

        sub_enable_ = create_subscription<std_msgs::msg::Bool>("/erobo/enable", 10,
            std::bind(&ERoboDriver::cb_enable, this, std::placeholders::_1));
        sub_brake_ = create_subscription<std_msgs::msg::Bool>("/erobo/brake", 10,
            std::bind(&ERoboDriver::cb_brake, this, std::placeholders::_1));
        sub_clear_ = create_subscription<std_msgs::msg::Empty>("/erobo/clear_error", 10,
            std::bind(&ERoboDriver::cb_clear, this, std::placeholders::_1));
        sub_stop_ = create_subscription<std_msgs::msg::Empty>("/erobo/stop", 10,
            std::bind(&ERoboDriver::cb_stop_motor, this, std::placeholders::_1));

        sub_pos_ = create_subscription<std_msgs::msg::Float32MultiArray>("/erobo/pos_target", 10,
            std::bind(&ERoboDriver::cb_pos, this, std::placeholders::_1));
        sub_vel_ = create_subscription<std_msgs::msg::Float32MultiArray>("/erobo/vel_target", 10,
            std::bind(&ERoboDriver::cb_vel, this, std::placeholders::_1));
        sub_tor_ = create_subscription<std_msgs::msg::Float32MultiArray>("/erobo/tor_target", 10,
            std::bind(&ERoboDriver::cb_tor, this, std::placeholders::_1));

        // 数据采集线程
        std::thread(&ERoboDriver::feedback_thread, this).detach();

        RCLCPP_INFO(get_logger(), "驱动已启动");
    }

    // ==============================
    // 扫描函数
    // ==============================
    void scan_joints() {
        RCLCPP_INFO(this->get_logger(), "扫描关节中...");
        joint_ids_.clear();

        for (int id = 1; id <= 127; id++) {
            can_frame f{};
            f.can_id = 0x640 + id;
            f.can_dlc = 2;
            f.data[0] = 0x00;
            f.data[1] = 0x8A;
            write(can_fd_, &f, sizeof(f));
            usleep(8000);
        }

        auto start = std::chrono::steady_clock::now();
        while (true) {
            auto now = std::chrono::steady_clock::now();
            std::chrono::duration<double> diff = now - start;
            if (diff.count() > 2.0) break;

            can_frame frame{};
            if (read(can_fd_, &frame, sizeof(frame)) <= 0)
                continue;

            if (frame.can_id >= 0x581 && frame.can_id <= 0x5FF && frame.can_dlc == 5)
            {
                int node = frame.can_id - 0x580;
                node = node - 0x40;
                if (std::find(joint_ids_.begin(), joint_ids_.end(), node) == joint_ids_.end())
                {
                    RCLCPP_INFO(this->get_logger(), "找到关节 ID: %d", node);
                    joint_ids_.push_back(node);
                }
            }

            if (joint_ids_.size() == 3) break;
        }

        if (joint_ids_.empty())
            RCLCPP_ERROR(this->get_logger(), "未找到关节");
        else
            RCLCPP_INFO(this->get_logger(), "扫描完成：%zu 个关节", joint_ids_.size());
    }

private:
    int can_fd_;
    std::vector<int> joint_ids_;
    std::map<int, float> pos_, vel_, cur_, temp_;
    std::map<int, uint8_t> error_;  // 新增：错误码

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_state_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub_error_;  // 新增：错误发布

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_enable_, sub_brake_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_clear_, sub_stop_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_pos_, sub_vel_, sub_tor_;

    bool send_frame(uint32_t can_id, uint8_t* data, uint8_t len) {
        struct can_frame frame;
        frame.can_id = can_id;
        frame.can_dlc = len;
        memcpy(frame.data, data, len);
        int ret = write(can_fd_, &frame, sizeof(frame));
        return ret == sizeof(frame);
    }

    void send_cmd(int id, uint16_t cmd, bool ext = false) {
        can_frame f{};
        f.can_id = 0x640 + id;
        f.can_dlc = 2;
        f.data[0] = (cmd >> 8) & 0xFF;
        f.data[1] = cmd & 0xFF;
        if(ext) {
            f.can_dlc = 4;
            f.data[2] = 0x00;
            f.data[3] = 0x01;
        }
        write(can_fd_, &f, sizeof(f));
        usleep(10000);
    }

    void send_write(int id, uint16_t cmd, uint32_t val) {
        can_frame f{};
        f.can_id = 0x640 + id;
        f.can_dlc = 6;
        f.data[0] = (cmd >> 8) & 0xFF;
        f.data[1] = cmd & 0xFF;
        f.data[2] = (val >> 24) & 0xFF;
        f.data[3] = (val >> 16) & 0xFF;
        f.data[4] = (val >> 8) & 0xFF;
        f.data[5] = val & 0xFF;
        write(can_fd_, &f, sizeof(f));
        usleep(12000);
    }

    void recv_only() {
        can_frame rx;
        while (read(can_fd_, &rx, sizeof(rx)) > 0);
    }

    float parse_pos(int id) {
        can_frame rx{};
        if (read(can_fd_, &rx, sizeof(rx)) > 0 && rx.can_dlc ==5 && rx.data[4]==0x3E) {
            int32_t raw = (rx.data[0]<<24) | (rx.data[1]<<16) | (rx.data[2]<<8) | rx.data[3];
            if (raw & 0x80000000) raw -= 0x100000000;
            return raw / 524287.0f * 360.0f;
        }
        return pos_[id];
    }

    float parse_vel(int id) {
        can_frame rx{};
        if (read(can_fd_, &rx, sizeof(rx)) > 0 && rx.can_dlc ==5 && rx.data[4]==0x3E) {
            int32_t raw = (rx.data[0]<<24) | (rx.data[1]<<16) | (rx.data[2]<<8) | rx.data[3];
            if (raw & 0x80000000) raw -= 0x100000000;
            return raw / 100.0f;
        }
        return vel_[id];
    }

    float parse_cur(int id) {
        can_frame rx{};
        if (read(can_fd_, &rx, sizeof(rx)) > 0 && rx.can_dlc ==5 && rx.data[4]==0x3E) {
            int32_t raw = (rx.data[0]<<24) | (rx.data[1]<<16) | (rx.data[2]<<8) | rx.data[3];
            if (raw & 0x80000000) raw -= 0x100000000;
            return raw / 1000.0f;
        }
        return cur_[id];
    }

    float parse_temp(int id) {
        can_frame rx{};
        if (read(can_fd_, &rx, sizeof(rx)) > 0 && rx.can_dlc ==5 && rx.data[4]==0x3E) {
            int32_t raw = (rx.data[0]<<24) | (rx.data[1]<<16) | (rx.data[2]<<8) | rx.data[3];
            return raw / 10.0f;
        }
        return temp_[id];
    }

    // ==============================
    // 新增：读取关节错误状态
    // ==============================
    uint8_t parse_error(int id) {
        can_frame rx{};
        if (read(can_fd_, &rx, sizeof(rx)) > 0 && rx.can_dlc == 5 && rx.data[4] == 0x3E) {
            return rx.data[3];
        }
        return error_[id];
    }

    // ==============================
    // 数据采集线程（已加入错误读取）
    // ==============================
    void feedback_thread() {
        while (rclcpp::ok()) {
            for (int id : joint_ids_) {
                send_cmd(id, CMD_READ_POS, false);
                pos_[id] = parse_pos(id);

                send_cmd(id, CMD_READ_VEL, true);
                vel_[id] = parse_vel(id);

                send_cmd(id, CMD_READ_CURRENT, false);
                cur_[id] = parse_cur(id);

                send_cmd(id, CMD_READ_TEMP, false);
                temp_[id] = parse_temp(id);

                // 新增：读取错误
                send_cmd(id, CMD_READ_ERROR, false);
                error_[id] = parse_error(id);

                recv_only();
            }

            // 发布状态
            std_msgs::msg::Float32MultiArray msg;
            for (int id : joint_ids_) {
                msg.data.push_back(id);
                msg.data.push_back(pos_[id]);
                msg.data.push_back(vel_[id]);
                msg.data.push_back(cur_[id]);
                msg.data.push_back(temp_[id]);
            }
            pub_state_->publish(msg);

            // 新增：发布错误码
            std_msgs::msg::UInt8MultiArray err_msg;
            for (int id : joint_ids_) {
                err_msg.data.push_back(id);
                err_msg.data.push_back(error_[id]);
            }
            pub_error_->publish(err_msg);

            usleep(100000);
        }
    }

    void cb_enable(const std_msgs::msg::Bool::SharedPtr msg) {
        for (int id : joint_ids_)
            send_write(id, CMD_MOTOR_ENABLE, msg->data ? 1 : 0);
        RCLCPP_INFO(get_logger(), msg->data ? "电机已使能" : "电机已失能");
    }

    void cb_brake(const std_msgs::msg::Bool::SharedPtr msg) {
        for (int id : joint_ids_) {
            if (msg->data) {
                uint8_t cmd[2] = {0x01, 0x4F};
                send_frame(0x65F - 31 + id, cmd, 2);
            } else {
                uint8_t cmd[6] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
                send_frame(0x65F - 31 + id, cmd, 6);
            }
        }
        RCLCPP_INFO(get_logger(), msg->data ? "刹车已释放" : "刹车已锁定");
    }

    void cb_clear(const std_msgs::msg::Empty::SharedPtr) {
        for (int id : joint_ids_)
            send_cmd(id, CMD_CLEAR_ERROR);
        RCLCPP_INFO(get_logger(), "故障已清除");
    }

    void cb_pos(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        int id = msg->data[0];
        int32_t  p = (int32_t )(msg->data[1]/360.0f*524287.0f);
        float a = msg->data[2];
        float v = msg->data[3];
        send_write(id, CMD_CONTROL_MODE, 1);
        send_write(id, CMD_PROFILE_ACC, a*100);
        send_write(id, CMD_PROFILE_DEC, a*100);
        send_write(id, CMD_PROFILE_VEL, v*100);
        send_write(id, CMD_TARGET_POS, (uint32_t)(p));
        send_cmd(id, CMD_MOTOR_START);
        RCLCPP_INFO(get_logger(), "位置控制指令已发送: %i.", (uint32_t)p);
    }

    void cb_vel(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        int id = msg->data[0];
        float v = msg->data[1];
        send_write(id, CMD_CONTROL_MODE, 0);
        send_write(id, CMD_TARGET_VEL, v*100);
    }

    void cb_tor(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        int id = msg->data[0];
        float t = msg->data[1];
        send_write(id, CMD_CONTROL_MODE, 1);
        send_write(id, CMD_TARGET_TORQUE, t*1000);
    }
    
    void cb_stop_motor(const std_msgs::msg::Empty::SharedPtr msg) {
    	for (int id : joint_ids_)
            send_cmd(id, CMD_MOTOR_STOP);
        RCLCPP_INFO(get_logger(), "电机停止");
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ERoboDriver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
