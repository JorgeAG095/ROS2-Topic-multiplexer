#include "multiplexer_node/multiplexer.hpp"
namespace multiplexer
{
    void TopicMultiplexer::TwistTopicCallback(const geometry_msgs::msg::Twist::SharedPtr msg, const std::string& topic, const std::string& mux_name)
    {
        for (auto& mux : multiplexer_list)
        {
            if (mux.name == mux_name)
            {
                auto it = std::find_if(mux.latest_messages.begin(), mux.latest_messages.end(),
                    [&topic](const std::pair<std::string, std::variant<std::monostate, geometry_msgs::msg::Twist, std_msgs::msg::Float64>>& element){
                        return element.first == topic;
                    });

                if (it != mux.latest_messages.end())
                {
                    it->second = geometry_msgs::msg::Twist(*msg);
                }
            }
        }
    }
    void TopicMultiplexer::FloatTopicCallback(const std_msgs::msg::Float64::SharedPtr msg, const std::string& topic, const std::string& mux_name)
    {
        for (auto& mux : multiplexer_list)
        {
            if (mux.name == mux_name)
            {
                auto it = std::find_if(mux.latest_messages.begin(), mux.latest_messages.end(),
                    [&topic](const std::pair<std::string, std::variant<std::monostate, geometry_msgs::msg::Twist, std_msgs::msg::Float64>>& element){
                        return element.first == topic;
                    });

                if (it != mux.latest_messages.end())
                {
                    it->second = std_msgs::msg::Float64(*msg);
                }
            }
        }
    }
    void TopicMultiplexer::LockTopicCallback(const std_msgs::msg::Bool::SharedPtr msg, const std::string& topic)
    {
        auto it = std::find_if(lock_list.latest_messages.begin(), lock_list.latest_messages.end(),
                [&topic](const std::pair<std::string, std_msgs::msg::Bool>& element){
                    return element.first == topic;
                });
        if (it != lock_list.latest_messages.end())
        {
            it->second = std_msgs::msg::Bool(*msg);
        }
    }
}