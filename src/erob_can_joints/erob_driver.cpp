#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <cstring>
#include <unistd.h>
#include <vector>
#include <chrono>
#include <algorithm>
#include <thread>
#include <map>

using namespace std::chrono_literals;

struct CmdItem {
    std::vector<uint8_t> data;
};

// ====================== eRob 私有连接序列（来自你抓包）======================
const std::vector<CmdItem> EROB_CONNECT_SEQ = {
    {{0x00, 0x8A}},
    {{0x00, 0x01, 0x00, 0x03}},
    {{0x00, 0x8A}},
    {{0x01, 0x02}},
    {{0x00, 0x43}},
    {{0x00, 0xFF}},
    {{0x00, 0x2A, 0x00, 0x02}},
    {{0x00, 0x01, 0x00, 0x03}},
    {{0x00, 0x01, 0x00, 0x03}},
    {{0x02, 0x08, 0x00, 0x02}},
    {{0x00, 0x01, 0x00, 0x04}},
    {{0x02, 0x08, 0x00, 0x03}},
    {{0x00, 0x25}},
    {{0x02, 0x08, 0x00, 0x04}},
    {{0x01, 0xFB}},
    {{0x02, 0x08, 0x00, 0x05}},
    {{0x01, 0xFC}},
    {{0x02, 0x02}},
    {{0x02, 0x24}}
};

// ====================== 驱动主类 ======================
class ERobArmDriver : public rclcpp::Node {
public:
    ERobArmDriver() : Node("erob_arm_driver") {
        // 1. 初始化 CAN
        can_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_fd_ < 0) {
            RCLCPP_FATAL(get_logger(), "CAN 初始化失败");
            return;
        }

        struct ifreq ifr;
        strcpy(ifr.ifr_name, "can0");
        ioctl(can_fd_, SIOCGIFINDEX, &ifr);

        struct sockaddr_can addr;
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        bind(can_fd_, (struct sockaddr*)&addr, sizeof(addr));

        RCLCPP_INFO(get_logger(), "✅ CAN 接口准备就绪");

        // 2. 扫描关节
        scan_joints();
        if (joint_ids_.empty()) {
            RCLCPP_FATAL(get_logger(), "❌ 未找到关节");
            return;
        }

        // 3. 连接所有关节
        //for (int id : joint_ids_) {
        //    connect_erob_joint(id);
        //}
        RCLCPP_INFO(get_logger(), "✅ NMT 启动");

        // 4. NMT 启动
        nmt_operational();
        usleep(200000);

	RCLCPP_INFO(get_logger(), "✅ 驱动器使能");
        // 5. 驱动器使能
        for (int id : joint_ids_) {
            enable_drive(id);
        }

        // 6. 启动反馈线程
        std::thread(&ERobArmDriver::rx_pdo_thread, this).detach();

        // 7. ROS 话题
        pub_state_ = create_publisher<std_msgs::msg::Float32MultiArray>("/erob_joint_states", 10);
        sub_target_all_ = create_subscription<std_msgs::msg::Float32MultiArray>(
            "/erob_target_pos_all", 10,
            std::bind(&ERobArmDriver::cb_target_all, this, std::placeholders::_1));
        sub_single_ = create_subscription<std_msgs::msg::Float32MultiArray>(
            "/erob_target_single", 10,
            std::bind(&ERobArmDriver::cb_single_control, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "🎉 驱动启动成功！关节数量：%d", (int)joint_ids_.size());
    }

private:
    int can_fd_;
    std::vector<int> joint_ids_;
    std::map<int, std::vector<float>> joint_data_;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_state_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_target_all_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_single_;

    // ====================== 扫描关节 ======================
    void scan_joints() {
        RCLCPP_INFO(this->get_logger(), "🔍 【eTuner同款】扫描关节中...");
		joint_ids_.clear();

		// 发送 私有扫描帧：ID=0x640+id, 数据=[0x00, 0x8A]
		for (int id = 1; id <= 127; id++)
		{
		    can_frame f{};
		    f.can_id = 0x640 + id;      // eTuner 扫描 ID
		    f.can_dlc = 2;              // 固定 2 字节
		    f.data[0] = 0x00;           // 固定
		    f.data[1] = 0x8A;           // 固定
		    write(can_fd_, &f, sizeof(f));
		    usleep(8000);
		}

		// 监听回复 0x581 ~ 0x5FF
		// 正确：计时 2 秒，不卡死
		auto start = std::chrono::steady_clock::now();
		while (true) {
		    auto now = std::chrono::steady_clock::now();
		    std::chrono::duration<double> diff = now - start;
		    if (diff.count() > 2.0) break; 
		    
		    RCLCPP_INFO(this->get_logger(), "🔍 【eTuner同款】扫描关节中...");
		    can_frame frame{};
		    if (read(can_fd_, &frame, sizeof(frame)) <= 0)
		        continue;

		RCLCPP_INFO(this->get_logger(), "🔍 【eTuner同款】扫描关节中1...");
		    // 关节正确回复规则：
		    // ID = 0x580 + id
		    // 长度 = 5
		    if (frame.can_id >= 0x581 && frame.can_id <= 0x5FF && frame.can_dlc == 5)
		    {
		        int node = frame.can_id - 0x580;
		        node = node - 0x40;
		        if (std::find(joint_ids_.begin(), joint_ids_.end(), node) == joint_ids_.end())
		        {
		            RCLCPP_INFO(this->get_logger(), "✅ 找到关节 ID: %d", node);
		            joint_ids_.push_back(node);
		        }
		    }
		    
		    if(joint_ids_.size() == 3) break;
		}

		if (joint_ids_.empty())
		    RCLCPP_ERROR(this->get_logger(), "❌ 未找到关节");
		else
		    RCLCPP_INFO(this->get_logger(), "✅ 扫描完成：%zu 个关节", joint_ids_.size());
    }

    // ====================== 私有连接 ======================
    bool connect_erob_joint(int real_id) {
        int vid = real_id + 0x40;
        uint32_t tx = 0x640 + vid;
        uint32_t rx = 0x580 + vid;

        for (const auto& cmd : EROB_CONNECT_SEQ) {
            can_frame f{};
            f.can_id = tx;
            f.can_dlc = cmd.data.size();
            memcpy(f.data, cmd.data.data(), f.can_dlc);
            write(can_fd_, &f, sizeof(f));

            bool ok = false;
            auto t0 = std::chrono::steady_clock::now();
            while (std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - t0).count() < 40) {
                can_frame frame;
                if (read(can_fd_, &frame, sizeof(frame)) > 0 &&
                    frame.can_id == rx && frame.can_dlc == 5) {
                    ok = true;
                    break;
                }
            }
            if (!ok) return false;
            usleep(3000);
        }
        RCLCPP_INFO(get_logger(), "✅ 关节 %d 连接成功", real_id);
        return true;
    }

    // ====================== NMT ======================
    void nmt_operational() {
        can_frame f{};
        f.can_id = 0x000;
        f.can_dlc = 2;
        f.data[0] = 0x01;
        f.data[1] = 0x00;
        write(can_fd_, &f, sizeof(f));
    }

    // ====================== 驱动器使能 ======================
    void enable_drive(int id) {
        send_sdo(id, 0x6040, 0x00, 0x06);
        usleep(50000);
        send_sdo(id, 0x6040, 0x00, 0x07);
        usleep(50000);
        send_sdo(id, 0x6040, 0x00, 0x0F);
        usleep(50000);
        send_sdo(id, 0x6060, 0x00, 0x08);
        usleep(50000);
    }

    void send_sdo(int id, uint16_t idx, uint8_t sub, uint32_t v) {
        can_frame f{};
        f.can_id = 0x600 + id;
        f.can_dlc = 8;
        f.data[0] = 0x23;
        f.data[1] = idx & 0xFF;
        f.data[2] = (idx >> 8) & 0xFF;
        f.data[3] = sub;
        f.data[4] = v & 0xFF;
        f.data[5] = (v >> 8) & 0xFF;
        f.data[6] = (v >> 16) & 0xFF;
        f.data[7] = (v >> 24) & 0xFF;
        write(can_fd_, &f, sizeof(f));
    }

    // ====================== 接收反馈 ======================
    void rx_pdo_thread() {
        //can_frame f;
        //RCLCPP_INFO(get_logger(), "✅ 接收反馈");
        while (rclcpp::ok())
        {
            // 【关键】你只需要发这一帧！
            can_frame f{};
            f.can_id = 0x65F;
            f.can_dlc = 2;
            f.data[0] = 0x00;
            f.data[1] = 0x22;
            write(can_fd_, &f, sizeof(f));

            // 接收 50ms
            auto t_start = std::chrono::steady_clock::now();
            while (std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - t_start).count() < 50)
            {
                can_frame rx{};
                if (read(can_fd_, &rx, sizeof(rx)) <= 0)
                {
                    usleep(5000);
                    continue;
                }

                // =========================================
                // 自动解析关节返回的所有帧
                // =========================================
                if (rx.can_id == 0x61F && rx.can_dlc == 8)
                {
                    uint8_t cmd = rx.data[0];
                    uint16_t index = rx.data[1] | (rx.data[2] << 8);

                    // 速度 0x606C
                    if (index == 0x606C)
                    {
                        int32_t val = rx.data[4] | (rx.data[5] << 8) | (rx.data[6] << 16) | (rx.data[7] << 24);
                        vel_ = val / 100.0f;
                    }
                    // 电流 0x6078
                    else if (index == 0x6078)
                    {
                        int32_t val = rx.data[4] | (rx.data[5] << 8);
                        cur_ = val / 100.0f;
                    }
                    // 位置 0x6064
                    else if (index == 0x6064)
                    {
                        int32_t val = rx.data[4] | (rx.data[5] << 8) | (rx.data[6] << 16) | (rx.data[7] << 24);
                        pos_ = val / 10000.0f;
                    }
                }
            }

            // 发布
            std_msgs::msg::Float32MultiArray msg;
            msg.data = {pos_, vel_, cur_};
            pub_->publish(msg);
            usleep(50000);
        }
    }

    void publish_state() {
        std_msgs::msg::Float32MultiArray m;
        for (int id : joint_ids_) {
            auto& d = joint_data_[id];
            m.data.push_back(d[0]);
            m.data.push_back(d[1]);
            m.data.push_back(d[2]);
            m.data.push_back(d[3]);
        }
        pub_state_->publish(m);
    }

    // ====================== 同步控制所有关节 ======================
    void cb_target_all(const std_msgs::msg::Float32MultiArray::SharedPtr m) {
        if (m->data.size() != joint_ids_.size()) return;
        for (size_t i=0; i<joint_ids_.size(); i++) {
            set_pos(joint_ids_[i], m->data[i]);
        }
    }

    // ====================== 单独控制关节 ======================
    void cb_single_control(const std_msgs::msg::Float32MultiArray::SharedPtr m) {
        if (m->data.size() < 2) return;
        int id = (int)m->data[0];
        float pos = m->data[1];
        set_pos(id, pos);
    }

    void set_pos(int id, float pos) {
        int32_t v = (int32_t)(pos * 10000);
        can_frame f{};
        f.can_id = 0x200 + id;
        f.can_dlc = 4;
        f.data[0] = v & 0xFF;
        f.data[1] = (v >> 8) & 0xFF;
        f.data[2] = (v >> 16) & 0xFF;
        f.data[3] = (v >> 24) & 0xFF;
        write(can_fd_, &f, sizeof(f));
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ERobArmDriver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
