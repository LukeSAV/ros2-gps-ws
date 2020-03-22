#include <chrono>
#include <fcntl.h>
#include <iostream>
#include <memory>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/logging.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rcl_interfaces/srv/list_parameters.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;

#define MAX_BUFFER_SIZE 1500

struct SerialConnection
{
    struct termios term_if;
    int stream;
};
struct SerialParams 
{
    int baud_rate;
    int stop_bits;
};

class SerialIF : public rclcpp::Node
{
public:
    explicit SerialIF (
        const std::string &node_name, 
        const rclcpp::NodeOptions &options
    )
    : Node(node_name, options)
    {
        if((serial_connection_.stream = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NONBLOCK)) <= 0)
        {
            std::cout << "Could not open serial port" << std::endl;
            return;
        }
        else 
        {
            std::cout << "Opened serial port" << std::endl;
        }
        struct termios new_term_if;
        tcgetattr(serial_connection_.stream, &(serial_connection_.term_if));
        memset(&new_term_if, 0, sizeof(struct termios));
        new_term_if.c_cflag = B57600 | CS8 | CLOCAL | CREAD; 
        new_term_if.c_cc[VMIN] = 1;
        tcflush(serial_connection_.stream, TCIOFLUSH);
        tcsetattr(serial_connection_.stream, TCSANOW, &new_term_if);
        tcflush(serial_connection_.stream, TCIOFLUSH);
        fcntl(serial_connection_.stream, F_SETFL, O_NONBLOCK);

        read_timer_ = this->create_wall_timer(
            500ms, std::bind(&SerialIF::read_callback, this));

        subscriber_ = this->create_subscription<std_msgs::msg::ByteMultiArray>(
            "/ntrip_client/rtcm3_0", 10, std::bind(&SerialIF::corrections_callback, this, _1));
    }
private:
    void read_callback()
    {
        char buffer[MAX_BUFFER_SIZE];
        int j = read(serial_connection_.stream, buffer, MAX_BUFFER_SIZE);
        if(j < 0) {
            return;
        }
        else 
        {
            std::cout << buffer << std::endl;
        }
    }
    void corrections_callback(const std_msgs::msg::ByteMultiArray::SharedPtr msg) const
    {
        char buffer[MAX_BUFFER_SIZE];
        std::copy(msg->data.begin(), msg->data.end(), buffer);
        int j = write(serial_connection_.stream, buffer, msg->data.size());
        if(j < 0)
        {
            return;
        }
        else 
        {
            std::cout << "Sent corrections" << std::endl;
        }
    }
    rclcpp::TimerBase::SharedPtr read_timer_;
    SerialParams serial_params_;
    SerialConnection serial_connection_;
    rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr subscriber_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options = rclcpp::NodeOptions();
  node_options.allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true);
  std::string node_name = "serial_if";
  rclcpp::spin(std::make_shared<SerialIF>(node_name, node_options));
  rclcpp::shutdown();
  return 0;
}