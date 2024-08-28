#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"

namespace multiplexer
{
    class TopicMultiplexer : public rclcpp::Node
    {
    public:
        // TODO : Use locks for topics (saffety)
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
        struct lock
        {
            std::vector<std::pair<std::string, int>> lock_topics;
            std::vector<std::pair<std::string, std_msgs::msg::Bool>> latest_messages;
            std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriber_list;
        };
        TopicMultiplexer(const std::string& multiplexer_config_path, const std::string& lock_config_path) : Node("topic_multiplexer")
        {
            load_config(multiplexer_config_path, lock_config_path, multiplexer_list, lock_list);
            // Multiplexers
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

            // Locks
            for (auto it = lock_list.lock_topics.begin(); it != lock_list.lock_topics.end(); it++)
            {
                std::string topic = it->first;
                lock_list.latest_messages.emplace_back(topic, std_msgs::msg::Bool());
                rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _subscriber =
                this->create_subscription<std_msgs::msg::Bool>(
                    topic, 10,
                    std::function<void(const std_msgs::msg::Bool::SharedPtr)>(
                        std::bind(&TopicMultiplexer::LockTopicCallback, this, std::placeholders::_1, topic)
                    )
                );
                lock_list.subscriber_list.emplace_back(_subscriber);
            }
        }
    private:
        void TwistTopicCallback(const geometry_msgs::msg::Twist::SharedPtr msg, const std::string& topic, const std::string& mux_name);
        void FloatTopicCallback(const std_msgs::msg::Float64::SharedPtr msg, const std::string& topic, const std::string& mux_name);
        void LockTopicCallback(const std_msgs::msg::Bool::SharedPtr msg, const std::string& topic);
        void highPriorityPublisher(const std::string& mux_name)
        {
            int lock_priority = 0;
            for (auto it = lock_list.lock_topics.begin(); it != lock_list.lock_topics.end(); it++)
            {
                std::string topic = it->first;
                int priority = it->second;
                auto it_2 = std::find_if(lock_list.latest_messages.begin(), lock_list.latest_messages.end(),
                    [&topic](const std::pair<std::string, std_msgs::msg::Bool>& element){
                        return element.first == topic;
                    });
                if(it_2->second.data)
                {
                    if (priority > lock_priority) { lock_priority = priority; }
                }
            }
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
                            if (priority > lock_priority)
                            {
                                output_topic = topic;
                                prev_priority = priority;
                            }
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
        void load_config(const std::string& multiplexer_config_path,
                        const std::string& lock_config_path,
                        std::vector<multiplexer>& multiplexer_list,
                        lock& lock_list)
        {
            try
            {
                // Multiplexers
                YAML::Node multiplexer_config = YAML::LoadFile(multiplexer_config_path);
                for (YAML::const_iterator it = multiplexer_config.begin(); it != multiplexer_config.end(); it++)
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

                // Locks
                YAML::Node lock_config = YAML::LoadFile(lock_config_path);
                for (YAML::const_iterator it = lock_config.begin(); it != lock_config.end(); it++)
                {
                    YAML::Node locks = it->second;
                    for (YAML::const_iterator it_2 = locks.begin(); it_2 != locks.end(); it_2++)
                    {
                        std::string topic = it_2->second["topic"].as<std::string>();
                        int priority = it_2->second["priority"].as<int>();
                        lock_list.lock_topics.emplace_back(topic, priority);
                    }
                }
            }
            catch(const std::exception& e)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to load configuration file: %s", e.what());
            }    
        }
        std::vector<multiplexer> multiplexer_list;
        lock lock_list;
    };
}