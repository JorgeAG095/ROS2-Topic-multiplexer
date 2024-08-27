#include "pixkit_multiplexer/multiplexer.hpp"

using namespace multiplexer;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::string config_path = YAML_CONFIG_PATH;
    auto node = std::make_shared<TopicMultiplexer>(config_path);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}