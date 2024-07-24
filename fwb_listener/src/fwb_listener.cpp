#include <cstdio>
#include <vector>
#include <mutex>  // 添加互斥锁头文件
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

constexpr const char* base_topic_name = "fwb_stats";

namespace fwb_test
{
std::mutex mutex_;  // 互斥锁对象
// Create a FwbListener class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class FwbListener : public rclcpp::Node
{
public:
  explicit FwbListener(const std::string& topic_name)  : Node("fwb_listener_" + topic_name)
  {
    // Create a callback function for when messages are received.
    auto callback =
      [this, topic_name](std_msgs::msg::String::ConstSharedPtr msg) -> void
      {
        // 使用互斥锁保证输出顺序
        std::lock_guard<std::mutex> lock(mutex_);
        RCLCPP_INFO(this->get_logger(), "-->Listener heard on topic '%s': [%s] \n\n", topic_name.c_str(), msg->data.c_str());
      };
    
    // Create a subscription to the topic
    sub_ = create_subscription<std_msgs::msg::String>(topic_name, 10, callback);
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

}  // namespace fwb_test

int main(int argc, char * argv[])
{
  if (argc < 2) {
    printf("Usage: %s <topic1> <topic2> ... <topicN>\n", argv[0]);
    return 1;
  }

  printf("Running fwb_listener. Listening to %d topics:\n", argc - 1);

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;

  std::vector<std::shared_ptr<fwb_test::FwbListener>> listeners;
  for (int i = 1; i < argc; ++i) {
    std::string topic_name = argv[i];
    listeners.push_back(std::make_shared<fwb_test::FwbListener>(topic_name));
    exec.add_node(listeners.back());
  }

  exec.spin();

  rclcpp::shutdown();
  return 0;
}
