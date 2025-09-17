#include <string>

#include <gz/common/Console.hh>
#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/World.hh"

// This header is required to register plugins. It's good practice to place it
// in the cc file, like it's done here.
#include <gz/plugin/Register.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Pose3.hh>
#include <gz/sim/Util.hh>
#include <chrono>


// Don't forget to include the plugin's header.
#include "hello_world.hh"

// This is required to register the plugin. Make sure the interfaces match
// what's in the header.
GZ_ADD_PLUGIN(HelloWorld, gz::sim::System, HelloWorld::ISystemPostUpdate, HelloWorld::ISystemConfigure)

GZ_ADD_PLUGIN_ALIAS(HelloWorld, "gz::sim::system::HelloWorld")

static inline int bit_as_int(uint32_t m, unsigned i) {
  return ( (m >> i) & 0x1u ) ? 1 : 0;
}


HelloWorld::HelloWorld()
{
  // Constructor
  gz_node_.Subscribe ("/model/jaeger/valves_cmd_mask", &HelloWorld::OnGazeboValvesMsg, this);
}

// Here we implement the PostUpdate function, which is called at every
// iteration.
void HelloWorld::PostUpdate(const gz::sim::UpdateInfo& _info, const gz::sim::EntityComponentManager& _ecm)
{
  if (_info.paused)
    return;

  gz::msgs::Float value;
  std_msgs::msg::Float64 value_ros;
  std_msgs::msg::String string_msg;
  value.set_data(i);
  value_ros.data = i;
  i++;
  string_msg.data = name_ent;

  this->pub_example_.Publish(value);
  this->pub_example_ros_->publish(value_ros);
  this->pub_example_ros_2->publish(string_msg);

  this->valve_1_ = bit_as_int(latest_gz_valves_msg_.data(), 0);
  this->valve_2_ = bit_as_int(latest_gz_valves_msg_.data(), 1);
  this->valve_3_ = bit_as_int(latest_gz_valves_msg_.data(), 2);
  this->valve_4_ = bit_as_int(latest_gz_valves_msg_.data(), 3);

  // Reset the flag.
  new_msg_available_ = false;

  this->model_=14;

  gz::sim::Model my_model = gz::sim::Model(this->model_);

  igndbg << my_model.Name(_ecm) << std::endl;

  gz::sim::Entity link_ptr = my_model.LinkByName(_ecm, "base_link");

  gz::sim::Link link_base = gz::sim::Link(link_ptr);
  
  // Verificar y mostrar el nombre del enlace
  if (auto link_name = link_base.Name(const_cast<gz::sim::EntityComponentManager&>(_ecm)); link_name.has_value())
  {

  }
  else
  {
    ignerr << "No se pudo obtener el nombre del enlace" << std::endl;
  }

  gz::math::Pose3d world_pose = link_base.WorldPose(const_cast<gz::sim::EntityComponentManager&>(_ecm)).value();

  if(this->valve_1_==1)
  {
     // Fuerza en el marco local
     gz::math::Vector3d local_force(10.0, 0.0, 10.0); // Fuerza de 10 N en el eje X y Z local
     gz::math::Vector3d offset(0.0, 0.25, 0.0);
 
     // Convertir la fuerza al marco global
     gz::math::Vector3d world_force = world_pose.Rot().RotateVector(local_force);
     // Aplicar la fuerza en el marco local
     link_base.AddWorldForce(const_cast<gz::sim::EntityComponentManager&>(_ecm), local_force, offset);
  } else{
      // No aplicar fuerza si la válvula está cerrada
    gz::math::Vector3d local_force(0.0, 0.0, 10.0); // Fuerza de 10 N en el eje X y Z local
     gz::math::Vector3d offset(0.0, 0.25, 0.0);
 
     // Convertir la fuerza al marco global
     gz::math::Vector3d world_force = world_pose.Rot().RotateVector(local_force);
     // Aplicar la fuerza en el marco local
     link_base.AddWorldForce(const_cast<gz::sim::EntityComponentManager&>(_ecm), local_force, offset);
  }
  if(this->valve_2_==1)
  {
     // Fuerza en el marco local
     gz::math::Vector3d local_force(-10.0, 0.0, 10.0); // Fuerza de 10 N en el eje X y Z local
     gz::math::Vector3d offset(0.0, 0.25, 0.0);
 
     // Convertir la fuerza al marco global
     gz::math::Vector3d world_force = world_pose.Rot().RotateVector(local_force);
     // Aplicar la fuerza en el marco local
     link_base.AddWorldForce(const_cast<gz::sim::EntityComponentManager&>(_ecm), local_force, offset);
  } else{
      // No aplicar fuerza si la válvula está cerrada
    gz::math::Vector3d local_force(0.0, 0.0, 10.0); // Fuerza de 10 N en el eje X y Z local
     gz::math::Vector3d offset(0.0, 0.25, 0.0);
 
     // Convertir la fuerza al marco global
     gz::math::Vector3d world_force = world_pose.Rot().RotateVector(local_force);
     // Aplicar la fuerza en el marco local
     link_base.AddWorldForce(const_cast<gz::sim::EntityComponentManager&>(_ecm), local_force, offset);
  }
  if(this->valve_3_==1)
  {
     // Fuerza en el marco local
     gz::math::Vector3d local_force(10.0, 0.0, 10.0); // Fuerza de 10 N en el eje X y Z local
     gz::math::Vector3d offset(0.0, -0.25, 0.0);
 
     // Convertir la fuerza al marco global
     gz::math::Vector3d world_force = world_pose.Rot().RotateVector(local_force);
     // Aplicar la fuerza en el marco local
     link_base.AddWorldForce(const_cast<gz::sim::EntityComponentManager&>(_ecm), local_force, offset);
  } else{
      // No aplicar fuerza si la válvula está cerrada
    gz::math::Vector3d local_force(0.0, 0.0, 10.0); // Fuerza de 10 N en el eje X y Z local
     gz::math::Vector3d offset(0.0, -0.25, 0.0);
 
     // Convertir la fuerza al marco global
     gz::math::Vector3d world_force = world_pose.Rot().RotateVector(local_force);
     // Aplicar la fuerza en el marco local
     link_base.AddWorldForce(const_cast<gz::sim::EntityComponentManager&>(_ecm), local_force, offset);
  }
  if(this->valve_4_==1)
  {
     // Fuerza en el marco local
     gz::math::Vector3d local_force(-10.0, 0.0, 10.0); // Fuerza de 10 N en el eje X y Z local
     gz::math::Vector3d offset(0.0, -0.25, 0.0);
 
     // Convertir la fuerza al marco global
     gz::math::Vector3d world_force = world_pose.Rot().RotateVector(local_force);
     // Aplicar la fuerza en el marco local
     link_base.AddWorldForce(const_cast<gz::sim::EntityComponentManager&>(_ecm), local_force, offset);
  } else{
      // No aplicar fuerza si la válvula está cerrada
    gz::math::Vector3d local_force(0.0, 0.0, 10.0); // Fuerza de 10 N en el eje X y Z local
     gz::math::Vector3d offset(0.0, -0.25, 0.0);
 
     // Convertir la fuerza al marco global
     gz::math::Vector3d world_force = world_pose.Rot().RotateVector(local_force);
     // Aplicar la fuerza en el marco local
     link_base.AddWorldForce(const_cast<gz::sim::EntityComponentManager&>(_ecm), local_force, offset);
  }
    

  // Incrementar el contador de tiempo
static auto last_force_time = std::chrono::steady_clock::now();
auto current_time = std::chrono::steady_clock::now();
auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_force_time);

}

void HelloWorld::Configure(const gz::sim::Entity& _entity, const std::shared_ptr<const sdf::Element>& _sdf,
                           gz::sim::EntityComponentManager& _ecm, gz::sim::EventManager& /*_eventMgr*/)
{
  if (!_sdf->HasElement("ras_param"))
  {
    ignerr << "El parametro <ras_param> no existe" << std::endl;
    return;
  }

  if (!_sdf->HasElement("ras_param1"))
  {
    ignerr << "El parametro <ras_param1> no existe" << std::endl;
    return;
  }

  if (!_sdf->HasElement("ras_param2"))
  {
    ignerr << "El parametro <ras_param2> no existe" << std::endl;
    return;
  }

  if (!_sdf->HasElement("ras_param_pose"))
  {
    ignerr << "El parametro <ras_param_pose> no existe" << std::endl;
    return;
  }

  if (!_sdf->HasElement("ras_param_pose_t"))
  {
    ignerr << "El parametro <ras_param_pose> no existe" << std::endl;
    return;
  }

  if (_sdf->HasElement("topic"))
  {
    topic_example_msgs = _sdf->Get<std::string>("topic");
  }

  
  gz::sim::Model model1(_entity);
  name_ent = model1.Name(_ecm);
  std::string value = _sdf->Get<std::string>("ras_param");
  double value_1 = _sdf->Get<double>("ras_param1");
  int value_2 = _sdf->Get<int>("ras_param2");
  gz::math::Vector3d pose = _sdf->Get<gz::math::Vector3d>("ras_param_pose");
  gz::math::Pose3d pose_t = _sdf->Get<gz::math::Pose3d>("ras_param_pose_t");
  ignerr << "El valor de la variable es: " << value << std::endl;
  ignerr << "El valor de la variable1 es: " << value_1 << std::endl;
  ignerr << "El valor de la variable2 es: " << value_2 << std::endl;
  ignerr << "El valor de la variable3 es: " << pose << std::endl;
  ignerr << "El valor de la variable4 es: " << pose_t << std::endl;

  if (!rclcpp::ok())
  {
    rclcpp::init(0, nullptr);
  }
  std::string topic_xd = "zzz";
  this->pub_example_ = this->node_.Advertise<gz::msgs::Float>(topic_example_msgs);
  this->ros_node = std::make_shared<rclcpp::Node>("my_node");
  this->pub_example_ros_ = this->ros_node->create_publisher<std_msgs::msg::Float64>(topic_example_msgs, 10);
  this->pub_example_ros_2 = this->ros_node->create_publisher<std_msgs::msg::String>(topic_xd, 100);

  
  model_ = _entity;

  gz::sim::Model my_model = gz::sim::Model(this->model_);

  igndbg << my_model.Name(_ecm) << std::endl;

  gz::sim::Entity link_ptr = my_model.LinkByName(_ecm, "link");
  if (!link_ptr)
  {
    ignerr << "Link pointer not found" << std::endl;
    return;
  }
  gz::sim::Link link_base = gz::sim::Link(link_ptr);
  if (_ecm.Component<gz::sim::components::WorldPose>(link_base.Entity()))
  {
    _ecm.CreateComponent<gz::sim::components::WorldPose>(link_base.Entity(), gz::sim::components::WorldPose());
  }

  link_base.EnableVelocityChecks(_ecm, true);

  if (!_ecm.Component<gz::sim::components::AngularVelocity>(link_base.Entity()))
  {
    _ecm.CreateComponent(link_base.Entity(), gz::sim::components::AngularVelocity());
  }

  if (!_ecm.Component<gz::sim::components::LinearVelocity>(link_base.Entity()))
  {
    _ecm.CreateComponent(link_base.Entity(), gz::sim::components::LinearVelocity());
  }
}

void HelloWorld::OnGazeboValvesMsg (const gz::msgs::UInt32 &_msg)
{
    // Save the latest valves message:
    latest_gz_valves_msg_ = _msg;
    new_msg_available_ = true;
}
