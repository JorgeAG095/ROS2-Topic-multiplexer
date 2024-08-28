#include "multiplexer_node/multiplexer.hpp"

using namespace multiplexer;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::string multiplexer_config_path = MULTIPLEXER_CONFIG_PATH;
    std::string locks_config_path = LOCK_CONFIG_PATH;
    auto node = std::make_shared<TopicMultiplexer>(multiplexer_config_path, locks_config_path);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}