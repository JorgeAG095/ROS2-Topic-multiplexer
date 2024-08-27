#include "pixkit_multiplexer/multiplexer.hpp"
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
                    // XXX - DEBUG
                    // TODO : test  if (std::holds_alternative<geometry_msgs::msg::Twist>(var)) {}
                    /*if (const auto* twist_msg = std::get_if<geometry_msgs::msg::Twist>(&(it->second))) {
                        // If the variant holds a geometry_msgs::msg::Twist
                        std::cout << "Twist message:" << std::endl;
                        std::cout << "  Linear X: " << twist_msg->linear.x << std::endl;
                        std::cout << "  Linear Y: " << twist_msg->linear.y << std::endl;
                        std::cout << "  Linear Z: " << twist_msg->linear.z << std::endl;
                        std::cout << "  Angular X: " << twist_msg->angular.x << std::endl;
                        std::cout << "  Angular Y: " << twist_msg->angular.y << std::endl;
                        std::cout << "  Angular Z: " << twist_msg->angular.z << std::endl;
                    }*/
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
                    // XXX - DEBUG
                    /*if (const auto* float_msg = std::get_if<std_msgs::msg::Float64>(&(it->second))) {
                        // If the variant holds a std_msgs::msg::Float64
                        std::cout << "Float message:" << std::endl;
                        std::cout << "  Data: " << float_msg->data << std::endl;
                    }*/
                }
            }
        }
    }
}