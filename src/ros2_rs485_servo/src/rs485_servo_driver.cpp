#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstdint>
#include <vector>
#include <cstring>

class ServoDriver : public rclcpp::Node
{
private:
  int serial_fd_ = -1;
  std::string serial_port_;
  int baudrate_;

  // 校验和：~(ID+LEN+CMD+DATA) & 0xFF
  uint8_t calculate_checksum(const std::vector<uint8_t>& data)
  {
    uint16_t sum = 0;
    for (auto b : data) sum += b;
    return static_cast<uint8_t>(~sum & 0xFF);
  }

  // 发送舵机数据包
  void send_packet(const std::vector<uint8_t>& packet)
  {
    if (serial_fd_ < 0) return;
    write(serial_fd_, packet.data(), packet.size());
    tcdrain(serial_fd_);
    std::cout << "rs485 servo data: ";
    for(int ii=0;ii<packet.size();ii++) std::cout << std::to_string(packet[ii]) << " ";
    std::cout << std::endl;
  }
  
  void action() {
  	std::vector<uint8_t> buf = {0xFF, 0xFF, 0xFE, 0x02, 0x05, 0xFA};
  	send_packet(buf);
  }

  // 控制舵机：ID, 位置(0~4095), 速度, 电流
  void set_servo(uint8_t id, uint16_t position, uint16_t speed, bool now = true)
  {
    std::vector<uint8_t> buf;

    // 帧头
    buf.push_back(0xFF);
    buf.push_back(0xFF);
    buf.push_back(id);

    // 长度：指令1 + 地址1 + 数据8 = 10 → 0x0A
    buf.push_back(0x09);
    buf.push_back(0x04);  // WRITE_DATA 指令

    // 寄存器起始地址：目标位置 0x2A
    buf.push_back(0x2A);

    // 目标位置(2字节 小端)
    buf.push_back(position & 0xFF);
    buf.push_back((position >> 8) & 0xFF);

    // 运行时间(2字节)
    buf.push_back(0x00);
    buf.push_back(0x00);

    // 速度(2字节)
    buf.push_back(speed & 0xFF);
    buf.push_back((speed >> 8) & 0xFF);

    // 电流(2字节)
    // buf.push_back(current & 0xFF);
    // buf.push_back((current >> 8) & 0xFF);

    // 校验和
    std::vector<uint8_t> ck_buf(buf.begin()+2, buf.end());
    uint8_t checksum = calculate_checksum(ck_buf);
    buf.push_back(checksum);

    send_packet(buf);

    RCLCPP_INFO(this->get_logger(),
      "ID:%d | POS:%d | SPEED:%d",
      id, position, speed);
      
    if(now) action();
  }
  
  bool checkLimits(uint8_t id, uint16_t position, uint16_t speed) {
  
  	if(speed > 15) return false;
  	if(id == 2) {
  		if(position < 2000 || position > 5000) return false;
  		else return true;
  	} else if(id == 1) {
  		if(position < 1000 || position > 2700) return false;
  		else return true;
  	}
  }

  // 订阅话题：/servo/ctrl
  //当发送的数据为4个时，如果最后一个是0，则代表只发送位置值，但不执行运动指令，如果最后一个不0，则收到位置后立即执行运动；
  //当发送的数据为3个时，则代表只发送位置值，但不执行运动指令，
  //如果发送的数据为1个时，只发送执行指令
  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr sub_;

  void callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg)
  {
    if (msg->data.size() == 3) {
    
    	uint8_t id       = (uint8_t)msg->data[0];
		uint16_t pos     = (msg->data[1]) ;
		uint16_t speed   = (msg->data[2]);
		//uint16_t current = (msg->data[6] << 8) | msg->data[5];
		
		if(checkLimits(id, pos, speed)) set_servo(id, pos, speed, false);
    }
    if (msg->data.size() == 4) {
    
    	uint8_t id       = (uint8_t)msg->data[0];
		uint16_t pos     = (msg->data[1]) ;
		uint16_t speed   = (msg->data[2]);
		//uint16_t current = (msg->data[6] << 8) | msg->data[5];

		if(msg->data[3] != 0) if(checkLimits(id, pos, speed)) set_servo(id, pos, speed, true);
		else if(checkLimits(id, pos, speed)) set_servo(id, pos, speed, false);
    }
    else if (msg->data.size() == 1 && msg->data[0] != 0) {
    
    	action();
    }

    
  }

  // 打开串口
  bool open_serial()
  {
    serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY);
    if (serial_fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open %s", serial_port_.c_str());
      return false;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(serial_fd_, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "tcgetattr failed");
      return false;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag = 0;
    tty.c_iflag = 0;
    tty.c_oflag = 0;

    tty.c_cc[VTIME] = 1;
    tty.c_cc[VMIN] = 0;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "tcsetattr failed");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Serial %s connected", serial_port_.c_str());
    return true;
  }

public:
  ServoDriver() : Node("servo_driver_node")
  {
    this->declare_parameter<std::string>("port", "/dev/ttyACM0");
    this->get_parameter("port", serial_port_);

    if (!open_serial()) {
      RCLCPP_FATAL(this->get_logger(), "Cannot open serial");
      return;
    }
    
    set_servo(1, 1500, 10, true);
    set_servo(2, 3450, 10, true);

    sub_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
      "/servo/ctrl", 10,
      std::bind(&ServoDriver::callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Servo driver ready!");
  }

  ~ServoDriver() override {
    if (serial_fd_ >= 0) close(serial_fd_);
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ServoDriver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
