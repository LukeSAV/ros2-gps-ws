#include <arpa/inet.h>
#include <chrono>
#include <iostream>
#include <memory>
#include <stdio.h>
#include <string>
#include <sys/socket.h>
#include <unistd.h>

#include "rclcpp/logging.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rcl_interfaces/srv/list_parameters.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"

#include "ntrip_client/encode.h"


static const int MAXDATASIZE = 1024;

using namespace std::chrono_literals;

class NTRIPClient : public rclcpp::Node
{
public:
  explicit NTRIPClient(
    const std::string &node_name, 
    const rclcpp::NodeOptions &options
  )
  : Node(node_name, options) 
  {
    publisher_ = this->create_publisher<std_msgs::msg::ByteMultiArray>("/ntrip_client/rtcm3_0", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&NTRIPClient::timer_callback, this));

    this->get_parameter("ip_addr", this->ip_addr_);
    RCLCPP_INFO(this->get_logger(), "IP Address: %s", this->ip_addr_.c_str());
    this->get_parameter("port", this->port_);
    RCLCPP_INFO(this->get_logger(), "Port: %d", this->port_);
    this->get_parameter("reply", this->reply_);
    RCLCPP_INFO(this->get_logger(), "Reply: %d", this->reply_);
    this->get_parameter("user", this->user_);
    RCLCPP_INFO(this->get_logger(), "User: %s", this->user_.c_str());
    this->get_parameter("password", this->password_);

    this->sock_ = 0;
    struct sockaddr_in addr;
    if ((this->sock_ = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Could not create socket");
      return;
    }
    addr.sin_family = AF_INET;
    addr.sin_port = htons(this->port_);
    if(::inet_pton(AF_INET, this->ip_addr_.c_str(), &addr.sin_addr) <= 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Invalid address");
      return;
    }
    if(::connect(this->sock_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Connection failed");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Successful connection to caster");
    nmeahead_ = "$GPGGA,194000,4244.0933,N,08318.3117,W,1,08,0.9,545.4,M,46.9,M,,*5F"; // Kroger on Baldwin Road
    long out_i;
    int numbytes;
    /* Send NTRIP 2.0 message to caster */ 
    out_i=snprintf(buf_, MAXDATASIZE-40, // Leave some space for login 
      "GET %s%s%s%s/%s HTTP/1.1\r\n"
      "Host: %s\r\n%s"
      "User-Agent: %s/%s\r\n"
      "%s%s%s"
      "Connection: close%s"
      , "", "",
      "", "",
      "NEAREST_RTCM3-GG", this->ip_addr_.c_str(),
      "Ntrip-Version: Ntrip/2.0\r\n",
      "NTRIP Client ROS2", "$Revision: 1.01 $",
      "Ntrip-GGA: ", nmeahead_.c_str(),
      "\r\n",
      "\r\nAuthorization: Basic "
    );
    out_i += encode(buf_ + out_i, MAXDATASIZE - out_i - 4, this->user_.c_str(), this->password_.c_str()); // -4 is to save room for carriage returns and line feeds
    buf_[out_i++] = '\r';
    buf_[out_i++] = '\n';
    buf_[out_i++] = '\r';
    buf_[out_i++] = '\n';

    RCLCPP_INFO(this->get_logger(), "Sending to caster");
    RCLCPP_INFO(this->get_logger(), buf_);

    if(::send(sock_, buf_, (size_t)out_i, 0) != out_i)
    {
      RCLCPP_ERROR(this->get_logger(), "Issue writing on socket");
      return;
    }
    numbytes = ::recv(sock_, buf_, MAXDATASIZE - 1, 0);
    if(numbytes > 17 && strstr(buf_, "ICY 200 OK"))
    {
      RCLCPP_INFO(this->get_logger(), "Proper stream connected. Caster expects data.");
    }
    if(::send(sock_, nmeahead_.c_str(), nmeahead_.size(), 0) != static_cast<int>(nmeahead_.size()))
    {
      RCLCPP_ERROR(this->get_logger(), "Issue writing on socket");
    }
    time_last_msg_sent_to_caster_ = this->now();
    std::cout << std::flush;
  }

private:
  void timer_callback()
  {
    int numbytes = ::recv(sock_, buf_, MAXDATASIZE - 1, 0);
    auto message = std_msgs::msg::ByteMultiArray();
    message.data.insert(message.data.begin(), buf_, buf_ + numbytes);
    message.layout.data_offset = numbytes;
    RCLCPP_INFO(this->get_logger(), "Publishing RTCM");
    publisher_->publish(message);
    rclcpp::Time cur_time = this->now();
    if((cur_time - time_last_msg_sent_to_caster_) > rclcpp::Duration(3, 0))
    {
      if(::send(sock_, nmeahead_.c_str(), nmeahead_.size(), 0) != static_cast<int>(nmeahead_.size()))
      {
        RCLCPP_ERROR(this->get_logger(), "Unable to write to socket");
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "Sent update to caster");
        time_last_msg_sent_to_caster_ = this->now();
      }
    }
    std::cout << std::flush;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr publisher_;
  int sock_;                // Bound socket
  rclcpp::Time time_last_msg_sent_to_caster_;
  char buf_[MAXDATASIZE];

  /* Parameters specified in .yaml file */
  std::string ip_addr_;     // IP address of NTRIP caster
  int port_;                // Port number to bind
  std::string mount_point;  // NTRIP mount point
  bool reply_;              // Used to determine whether to respond to caster for automatic base station connection
  std::string user_;        // Username if required
  std::string password_;    // Password if required
  std::string nmeahead_;


};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options = rclcpp::NodeOptions();
  options.allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true);
  std::string node_name = "ntrip_client";
  rclcpp::spin(std::make_shared<NTRIPClient>(node_name, options));
  rclcpp::shutdown();
  return 0;
}
