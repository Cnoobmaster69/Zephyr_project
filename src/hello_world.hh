#pragma once

#include <gz/sim/System.hh>
#include <gz/sim/config.hh>
#include <gz/transport/Node.hh>
#include <gz/transport/Publisher.hh>
#include <gz/msgs/float.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/double_v.pb.h>
#include <gz/msgs/uint32.pb.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <memory>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/World.hh>
#include <nav_msgs/msg/odometry.hpp>
#include <mutex>



class HelloWorld : public gz::sim::System, public gz::sim::ISystemPostUpdate, public gz::sim::ISystemConfigure
{
    // Plugins inheriting ISystemPostUpdate must implement the PostUpdate
    // callback. This is called at every simulation iteration after the physics
    // updates the world. The _info variable provides information such as time,
    // while the _ecm provides an interface to all entities and components in
    // simulation.
public:
    HelloWorld ();
    void PostUpdate(const gz::sim::UpdateInfo &_info, const gz::sim::EntityComponentManager &_ecm) override;

    void Configure(const gz::sim::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   gz::sim::EntityComponentManager &_ecm,
                   gz::sim::EventManager & /*_eventMgr*/) override;

    void OnGazeboValvesMsg (const gz::msgs::UInt32 &_msg);


private:
    gz::transport::Node node_;
    gz::transport::Node::Publisher pub_example_;

    std::string topic_example_msgs{"my_topic"};

    float i = 0;

    std::shared_ptr<rclcpp::Node> ros_node;
    std::shared_ptr<rclcpp::Node> ros_node_2;
    std::shared_ptr<rclcpp::Node> ros_node_sub;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> pub_example_ros_;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> pub_example_ros_2;
    std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> sub_example_ros_;
    gz::transport::Node gz_node_;
    int valve_1_=0;
    int valve_2_=0;     // valor “recibido”
    int valve_3_=0;
    int valve_4_=0;
    gz::msgs::UInt32 latest_gz_valves_msg_;
    bool new_msg_available_{false};

    //std_msgs::msg::String name_ent;
    std::string name_ent;
    gz::sim::Entity model_;
};
