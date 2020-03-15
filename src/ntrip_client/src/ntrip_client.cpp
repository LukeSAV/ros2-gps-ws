// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <iostream>

#include <arpa/inet.h>
#include <chrono>
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


using namespace std::chrono_literals;

#define MAXDATASIZE 1000

static const char encodingTable [64] = {
  'A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P',
  'Q','R','S','T','U','V','W','X','Y','Z','a','b','c','d','e','f',
  'g','h','i','j','k','l','m','n','o','p','q','r','s','t','u','v',
  'w','x','y','z','0','1','2','3','4','5','6','7','8','9','+','/'
};

/* does not buffer overrun, but breaks directly after an error */
/* returns the number of required bytes */
static int encode(char *buf, int size, const char *user, const char *pwd)
{
  unsigned char inbuf[3];
  char *out = buf;
  int i, sep = 0, fill = 0, bytes = 0;

  while(*user || *pwd)
  {
    i = 0;
    while(i < 3 && *user) inbuf[i++] = *(user++);
    if(i < 3 && !sep)    {inbuf[i++] = ':'; ++sep; }
    while(i < 3 && *pwd)  inbuf[i++] = *(pwd++);
    while(i < 3)         {inbuf[i++] = 0; ++fill; }
    if(out-buf < size-1)
      *(out++) = encodingTable[(inbuf [0] & 0xFC) >> 2];
    if(out-buf < size-1)
      *(out++) = encodingTable[((inbuf [0] & 0x03) << 4)
               | ((inbuf [1] & 0xF0) >> 4)];
    if(out-buf < size-1)
    {
      if(fill == 2)
        *(out++) = '=';
      else
        *(out++) = encodingTable[((inbuf [1] & 0x0F) << 2)
                 | ((inbuf [2] & 0xC0) >> 6)];
    }
    if(out-buf < size-1)
    {
      if(fill >= 1)
        *(out++) = '=';
      else
        *(out++) = encodingTable[inbuf [2] & 0x3F];
    }
    bytes += 4;
  }
  if(out-buf < size)
    *out = 0;
  return bytes;
}


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
      std::cout << "Could not create socket" << std::endl;
      return;
    }
    addr.sin_family = AF_INET;
    addr.sin_port = htons(this->port_);
    if(::inet_pton(AF_INET, this->ip_addr_.c_str(), &addr.sin_addr) <= 0)
    {
      std::cout << "Invalid address" << std::endl;
      return;
    }
    if(::connect(this->sock_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
      std::cout << "Connection failed" << std::endl;
      return;
    }
    std::cout << "Successful connection to caster" << std::endl;
    nmeahead_ = "$GPGGA,194000,4244.0933,N,08318.3117,W,1,08,0.9,545.4,M,46.9,M,,*5F"; // Kroger on Baldwin Road
    char buf[MAXDATASIZE];
    long out_i;
    int numbytes;
    /* Send NTRIP 2.0 message to caster */ 
    out_i=snprintf(buf, MAXDATASIZE-40, // Leave some space for login 
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
    out_i += encode(buf + out_i, MAXDATASIZE - out_i - 4, this->user_.c_str(), this->password_.c_str()); // -4 is to save room for carriage returns and line feeds
    buf[out_i++] = '\r';
    buf[out_i++] = '\n';
    buf[out_i++] = '\r';
    buf[out_i++] = '\n';

    std::cout << "Sending to caster" << std::endl;
    std::cout << buf << std::endl;

    if(::send(sock_, buf, (size_t)out_i, 0) != out_i)
    {
      std::cout << "Issue writing on socket" << std::endl;
      return;
    }
    numbytes = ::recv(sock_, buf, MAXDATASIZE - 1, 0);
    std::cout << numbytes << std::endl;
    std::cout << buf << std::endl;
    if(numbytes > 17 && strstr(buf, "ICY 200 OK"))
    {
      std::cout << "Proper stream connected. Caster expects data." << std::endl;
    }
    if(::send(sock_, nmeahead_.c_str(), nmeahead_.size(), 0) != static_cast<int>(nmeahead_.size()))
    {
      std::cout << "Issue writing on socket" << std::endl;
    }
    time_last_msg_sent_to_caster_ = this->now();
  }

private:
  void timer_callback()
  {
    char buffer[1024] = {0};
    std::cout << "Receiving data" << std::endl;
    int numbytes = ::recv(sock_, buffer, MAXDATASIZE - 1, 0);
    std::cout << numbytes << std::endl;
    std::cout << buffer << std::endl;
    auto message = std_msgs::msg::ByteMultiArray();
    message.data.insert(message.data.begin(), buffer, buffer + numbytes);
    RCLCPP_INFO(this->get_logger(), "Publishing RTCM");
    publisher_->publish(message);
    rclcpp::Time cur_time = this->now();
    if((cur_time - time_last_msg_sent_to_caster_) > rclcpp::Duration(3, 0))
    {
      if(::send(sock_, nmeahead_.c_str(), nmeahead_.size(), 0) != static_cast<int>(nmeahead_.size()))
      {
        std::cout << "Issue writing on socket" << std::endl;
      }
      std::cout << "Sent update to caster" << std::endl;
      time_last_msg_sent_to_caster_ = this->now();
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr publisher_;
  int sock_;                // Bound socket
  rclcpp::Time time_last_msg_sent_to_caster_;

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
