#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"

namespace multiplexer
{
    class TopicMultiplexer : public rclcpp::Node
    {
    public:
        // TODO : Use locks for topics
        struct multiplexer
        {
            int frequency;
            double timeout;
            std::string name;
            std::string type;
            std::string output_topic;
            std::vector<std::pair<std::string, int>> input_topics;
            std::vector<std::pair<std::string,std::variant<std::monostate, geometry_msgs::msg::Twist, std_msgs::msg::Float64>>> latest_messages;
            std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriber_list;
            rclcpp::PublisherBase::SharedPtr output_publisher;
            rclcpp::TimerBase::SharedPtr timer_;
        };
        TopicMultiplexer(const std::string &config_path) : Node("topic_multiplexer")
        {
            load_config(config_path, multiplexer_list);
            for (multiplexer& mux : multiplexer_list)
            {
                // Create subscribers
                for (auto it = mux.input_topics.begin(); it != mux.input_topics.end(); it++)
                {
                    std::string topic = it->first;
                    if (mux.type == "Twist")
                    {
                        mux.latest_messages.emplace_back(topic, geometry_msgs::msg::Twist());
                        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _subscriber = 
                        this->create_subscription<geometry_msgs::msg::Twist>(
                            topic, 10,
                            std::function<void(const geometry_msgs::msg::Twist::SharedPtr)>(
                                std::bind(&TopicMultiplexer::TwistTopicCallback, this, std::placeholders::_1, topic, mux.name)
                            )        
                        );
                        mux.subscriber_list.emplace_back(_subscriber);
                    }
                    else if (mux.type == "Float64")
                    {
                        mux.latest_messages.emplace_back(topic, std_msgs::msg::Float64());
                        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr _subscriber = 
                        this->create_subscription<std_msgs::msg::Float64>(
                            topic, 10, 
                            std::function<void(const std_msgs::msg::Float64::SharedPtr)>(
                                std::bind(&TopicMultiplexer::FloatTopicCallback, this, std::placeholders::_1, topic, mux.name)
                            )    
                        );
                        mux.subscriber_list.emplace_back(_subscriber);
                    }
                }
                // Create Publishers
                std::string topic = mux.output_topic;
                if (mux.type == "Twist")
                {
                    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _publisher =
                    this->create_publisher<geometry_msgs::msg::Twist>(topic, 10);
                    mux.output_publisher = _publisher;
                }
                else if (mux.type == "Float64")
                {
                    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _publisher =
                    this->create_publisher<std_msgs::msg::Float64>(topic, 10);
                    mux.output_publisher = _publisher;
                }
                // Create Timers
                mux.timer_ = this->create_wall_timer(
                    std::chrono::milliseconds(mux.frequency),
                    [this, &mux](){
                        this->highPriorityPublisher(mux.name);
                    });
            }
        }
    private:
        void TwistTopicCallback(const geometry_msgs::msg::Twist::SharedPtr msg, const std::string& topic, const std::string& mux_name);
        void FloatTopicCallback(const std_msgs::msg::Float64::SharedPtr msg, const std::string& topic, const std::string& mux_name);
        void highPriorityPublisher(const std::string& mux_name)
        {
            for (auto& mux : multiplexer_list)
            {
                if (mux.name == mux_name)
                {
                    int prev_priority = 0;
                    std::string output_topic;
                    for (auto it = mux.input_topics.begin(); it != mux.input_topics.end(); it++)
                    {
                        std::vector<std::pair<std::string, int>> published_topics;
                        std::string topic = it->first;
                        int priority = it->second;
                        auto topic_publishers = this->get_publishers_info_by_topic(topic);
                        if (topic_publishers.size() == 0)
                        {
                            auto it = std::find_if(mux.latest_messages.begin(), mux.latest_messages.end(),
                                [&topic](const std::pair<std::string, std::variant<std::monostate, geometry_msgs::msg::Twist, std_msgs::msg::Float64>>& element){
                                    return element.first == topic;
                                });
                            if (it != mux.latest_messages.end())
                            {
                                it->second = std::monostate();
                            }
                        }
                        else if (priority > prev_priority) 
                        { 
                            output_topic = topic;
                            prev_priority = priority;
                        }
                    }
                    auto it = std::find_if(mux.latest_messages.begin(), mux.latest_messages.end(),
                        [&output_topic](const std::pair<std::string, std::variant<std::monostate, geometry_msgs::msg::Twist, std_msgs::msg::Float64>>& element){
                            return element.first == output_topic;
                        });
                    if (it != mux.latest_messages.end())
                    {
                        if (auto* twist_msg = std::get_if<geometry_msgs::msg::Twist>(&(it->second))) { 
                            auto publisher = std::dynamic_pointer_cast<rclcpp::Publisher<geometry_msgs::msg::Twist>>(mux.output_publisher);
                            publisher->publish(*twist_msg);
                        }
                        else if (auto* float_msg = std::get_if<std_msgs::msg::Float64>(&(it->second))) { 
                            auto publisher = std::dynamic_pointer_cast<rclcpp::Publisher<std_msgs::msg::Float64>>(mux.output_publisher);
                            publisher->publish(*float_msg);
                        }
                    }
                }
            }
        }
        void load_config(const std::string &config_path, std::vector<multiplexer> &multiplexer_list)
        {
            try
            {
                YAML::Node config = YAML::LoadFile(config_path);
                for (YAML::const_iterator it = config.begin(); it != config.end(); it++)
                {
                    multiplexer m;
                    m.frequency = it->second["frequency_ms"].as<int>();
                    m.timeout = it->second["timeout"].as<double>();
                    m.name = it->second["name"].as<std::string>();
                    m.type = it->second["type"].as<std::string>();
                    m.output_topic = it->second["topics"]["output"]["topic"].as<std::string>();
                    YAML::Node in_topics = it->second["topics"]["inputs"];
                    for (YAML::const_iterator it_2 = in_topics.begin(); it_2 != in_topics.end(); it_2++)
                    {
                        YAML::Node value_node = it_2->second;
                        std::string topic_name = value_node["topic"].as<std::string>();
                        int topic_priority = value_node["priority"].as<int>();
                        m.input_topics.emplace_back(topic_name, topic_priority);
                    }
                    multiplexer_list.emplace_back(m);
                }
                
            }
            catch(const std::exception& e)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to load configuration file: %s", e.what());
            }    
        }
        std::vector<multiplexer> multiplexer_list;
    };
}