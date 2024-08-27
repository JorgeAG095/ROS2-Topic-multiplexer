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

class TopicMultiplexer : public rclcpp::Node
{
public:
    //TODO : Use locks for topics
    struct multiplexer
    {
        std::string name;
        std::string type;
        std::string output_topic;
        std::vector<std::pair<std::string, int>> input_topics;
        std::vector<std::pair<std::string,std::variant<std::monostate, geometry_msgs::msg::Twist, std_msgs::msg::Float64>>> latest_messages;
        std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriber_list;
        rclcpp::PublisherBase::SharedPtr output_publisher;
    };
    TopicMultiplexer(const std::string &config_path) : Node("topic_multiplexer")
    {
        load_config(config_path, multiplexer_list);
        int i = 0;
        for (multiplexer &mux : multiplexer_list)
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
                    multiplexer_list[i].subscriber_list.emplace_back(_subscriber);
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
                    multiplexer_list[i].subscriber_list.emplace_back(_subscriber);
                }
            }
            // Create Publishers
            std::string topic = mux.output_topic;
            if (mux.type == "Twist")
            {
                rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _publisher =
                this->create_publisher<geometry_msgs::msg::Twist>(topic, 10);
                multiplexer_list[i].output_publisher = _publisher;
            }
            else if (mux.type == "Float64")
            {
                rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _publisher =
                this->create_publisher<std_msgs::msg::Float64>(topic, 10);
                multiplexer_list[i].output_publisher = _publisher;
            }
            i++;
        }
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&TopicMultiplexer::highPriorityPublisher, this));
    }
private:
    void TwistTopicCallback(const geometry_msgs::msg::Twist::SharedPtr msg, const std::string& topic, const std::string& mux_name)
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
    void FloatTopicCallback(const std_msgs::msg::Float64::SharedPtr msg, const std::string& topic, const std::string& mux_name)
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
    void highPriorityPublisher()
    {
        for (auto& mux : multiplexer_list)
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
                    //published_topic.emplace_back(topic, priority); 
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
    void load_config(const std::string &config_path, std::vector<multiplexer> &multiplexer_list)
    {
        try
        {
            YAML::Node config = YAML::LoadFile(config_path);
            for (YAML::const_iterator it = config.begin(); it != config.end(); it++)
            {
                multiplexer m;
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
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    //std::string config_path = "config/multiplexer_config.yaml";
    std::string config_path = YAML_CONFIG_PATH;
    auto node = std::make_shared<TopicMultiplexer>(config_path);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
