#pragma once

#include <gz/sim/System.hh>
#include <gz/sim/config.hh>
#include <gz/transport/Node.hh>
#include <gz/transport/Publisher.hh>
#include <gz/msgs/float.pb.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/World.hh>

class HelloWorld : public gz::sim::System, public gz::sim::ISystemPostUpdate, public gz::sim::ISystemConfigure
{
    // Plugins inheriting ISystemPostUpdate must implement the PostUpdate
    // callback. This is called at every simulation iteration after the physics
    // updates the world. The _info variable provides information such as time,
    // while the _ecm provides an interface to all entities and components in
    // simulation.
public:
    void PostUpdate(const gz::sim::UpdateInfo &_info, const gz::sim::EntityComponentManager &_ecm) override;

    void Configure(const gz::sim::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   gz::sim::EntityComponentManager &_ecm,
                   gz::sim::EventManager & /*_eventMgr*/) override;


private:
    gz::transport::Node node_;
    gz::transport::Node::Publisher pub_example_;

    std::string topic_example_msgs{"my_topic"};

    float i = 0;

    std::shared_ptr<rclcpp::Node> ros_node;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> pub_example_ros_;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> pub_example_ros_2;
    //std_msgs::msg::String name_ent;
    std::string name_ent;
    gz::sim::Entity model_;
};
