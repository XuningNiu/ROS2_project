#include <chrono>
#include <cstdio>
#include <memory>
#include <utility>
#include <thread>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/epoll.h>
#include <unordered_map>
#include <signal.h> // Include for signal handling

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "fwb_message_interfaces/msg/device.hpp"  // 引入 device.h 生成的消息类型

constexpr const char* topic_name = "fwb_stats";

namespace fwb_test
{
// Create a FwbPub class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class FwbPub : public rclcpp::Node
{
public:
  explicit FwbPub() : Node("fwb_pub")
  {
    // 立即输出到终端
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    thread_ = std::thread([this]() {
        threadTask();
    });
    // Install signal handler for SIGINT (CTRL+C)
    signal(SIGINT, handleSignal);
  }
  ~FwbPub()
  {
      // 在节点销毁时，清理线程
      if (thread_.joinable()) {
          thread_.join();
      }
  }

private:
  std::thread thread_;
  std::unordered_map<std::string, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> topic_pub_mapping_;

  auto get_pub(const std::string& topic_name)
  {
    // 如果在mapping里面没有找到topic，就创建一个新的publisher，并且加入mapping
    auto it = topic_pub_mapping_.find(topic_name);
    if (it == topic_pub_mapping_.end()) {
      // 设置QoS配置
      auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
      // 创建发布者
      auto pub = this->create_publisher<std_msgs::msg::String>(topic_name, qos);
      topic_pub_mapping_[topic_name] = pub;
      return pub;
    }
    return it->second;
  }

  void process_and_publish(const std::string &data_str)
  {
    size_t topic_pos = data_str.find("topic:");
    size_t data_pos = data_str.find("data:");
    
    if (topic_pos != std::string::npos && data_pos != std::string::npos) {
      topic_pos += 6; // "topic:" 的长度
      data_pos += 5;  // "data:" 的长度
      
      // 提取 topic 和 data
      size_t topic_end_pos = data_str.find(",", topic_pos);
      size_t data_end_pos = data_str.length();
      
      if (topic_end_pos != std::string::npos) {
        std::string topic = data_str.substr(topic_pos, topic_end_pos - topic_pos);
        std::string data = data_str.substr(data_pos, data_end_pos - data_pos);
        
        // // 创建消息并发布
        // auto msg = std::make_unique<std_msgs::msg::String>();
        // msg->data = std::move(data);              
        auto message = fwb_message_interfaces::msg::Device();
        message.name = "ExampleDevice";
        message.id = 123;
        message.email = "abc.com";
        message.age = 999;
        
        RCLCPP_INFO(this->get_logger(), "-->Publishing to topic [%s]\n", topic.c_str());
        // 获取或创建相应的 publisher，并发布消息
        auto pub_test = this->create_publisher<fwb_message_interfaces::msg::Device>(topic_name, 15);
        pub_test->publish(message);
        //get_pub(topic)->publish(message);
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to parse topic from buffer: '%s'", data_str.c_str());
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Invalid buffer format: '%s'", data_str.c_str());
    }
  }
  void publish_message(const char *buffer)
  {
    // 按 \n\n 分割 buffer
    std::string buffer_str(buffer);
    size_t pos = 0;
    std::string delimiter = "\n\n";
    while ((pos = buffer_str.find(delimiter)) != std::string::npos) {
      std::string token = buffer_str.substr(0, pos);
      process_and_publish(token);
      buffer_str.erase(0, pos + delimiter.length());
    }
    // 处理最后一个或唯一一个数据块
    if (!buffer_str.empty()) {
      process_and_publish(buffer_str);
    }
  }
  void server_tcp(){
    constexpr int PORT = 8080;
    constexpr int MAX_EVENTS = 10;
    constexpr int BUFFER_SIZE = 1024*5;

    int server_fd, new_socket;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);
    
    // Create epoll instance
    int epoll_fd = epoll_create1(0);
    if (epoll_fd == -1) {
      perror("epoll_create1");
      return;
    }

    // Create socket file descriptor
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
      perror("socket failed");
      return;
    }

    // Set socket to non-blocking
    if (fcntl(server_fd, F_SETFL, O_NONBLOCK) < 0) {
      perror("fcntl");
      return;
    }

    // Forcefully attaching socket to the port 8080
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
      perror("setsockopt");
      return;
    }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    // Bind the socket to localhost and port 8080
    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
      perror("bind failed");
      return;
    }

    // Listen for incoming connections
    if (listen(server_fd, 3) < 0) {
      perror("listen");
      return;
    }

    // Add server_fd to epoll
    struct epoll_event event;
    event.events = EPOLLIN | EPOLLET; // Edge-triggered mode
    event.data.fd = server_fd;
    if (epoll_ctl(epoll_fd, EPOLL_CTL_ADD, server_fd, &event) == -1) {
      perror("epoll_ctl: server_fd");
      return;
    }

    std::vector<struct epoll_event> events(MAX_EVENTS);

    while (rclcpp::ok()) {
      // Wait for events
      int num_events = epoll_wait(epoll_fd, events.data(), MAX_EVENTS, -1);
      if (num_events == -1) {
        perror("epoll_wait");
        return;
      }

      // Process each event
      for (int i = 0; i < num_events; ++i) {
        if (events[i].data.fd == server_fd) {
          // Accept new connection
          while ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen)) > 0) {
            // Set new_socket to non-blocking
            if (fcntl(new_socket, F_SETFL, O_NONBLOCK) < 0) {
              perror("fcntl");
              return;
            }

            // Add new_socket to epoll
            event.events = EPOLLIN | EPOLLET;
            event.data.fd = new_socket;
            if (epoll_ctl(epoll_fd, EPOLL_CTL_ADD, new_socket, &event) == -1) {
              perror("epoll_ctl: new_socket");
              return;
            }

            std::cout << "New client connected" << std::endl;
          }
          if (new_socket == -1 && errno != EAGAIN && errno != EWOULDBLOCK) {
            perror("accept");
          }
        } else {
          // Handle data from existing clients
          int client_socket = events[i].data.fd;
          char buffer[BUFFER_SIZE] = {0};//要在每次read前清零，否则有问题
          int valread = read(client_socket, buffer, BUFFER_SIZE);
          if (valread == 0) {
            // Client closed connection
            std::cout << "Client disconnected" << std::endl;
            close(client_socket);
            epoll_ctl(epoll_fd, EPOLL_CTL_DEL, client_socket, nullptr);
          } else if (valread > 0) {
            // Process received data
            //printf("Received:\n %s\n", buffer);
            publish_message(buffer);
            // Echo back to client
            //send(client_socket, buffer, valread, 0);            
          } else {
            // Error in read
            perror("read");
          }
        }
      }
    }
  }
  void threadTask()
  {      
      // 新线程的具体任务
      while (rclcpp::ok()) {
          server_tcp();
      }
  }
  static void handleSignal(int signal)
  {
      RCLCPP_INFO(rclcpp::get_logger("FwbPub"), "Received signal %d. Shutting down...", signal);
      rclcpp::shutdown();
      exit(0);
  }
};

}  // namespace fwb_test

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<fwb_test::FwbPub>());
  rclcpp::shutdown();
  return 0;
}
