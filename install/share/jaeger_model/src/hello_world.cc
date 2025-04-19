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
IGNITION_ADD_PLUGIN(HelloWorld, gz::sim::System, HelloWorld::ISystemPostUpdate, HelloWorld::ISystemConfigure)

IGNITION_ADD_PLUGIN_ALIAS(HelloWorld, "gz::sim::system::HelloWorld")

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

  this->model_=4;

  gz::sim::Model my_model = gz::sim::Model(this->model_);

  igndbg << my_model.Name(_ecm) << std::endl;

  gz::sim::Entity link_ptr = my_model.LinkByName(_ecm, "link");

  gz::sim::Link link_base = gz::sim::Link(link_ptr);

  gz::math::Pose3d pose = link_base.WorldPose(_ecm).value();

  igndbg << pose << std::endl;

  // Get World
  gz::sim::Entity world_ptr =_ecm.EntityByComponents(gz::sim::components::World());

  gz::sim::World world = gz::sim::World(world_ptr);

  std::vector<gz::sim::Entity> models = world.Models(_ecm);
  //Print Models
  for(uint64_t i=0; i < world.ModelCount(_ecm); i++) {
    gz::sim::Entity model_ptr = models[i];

    gz::sim::Model model = gz::sim::Model(model_ptr);

    igndbg << model.Name(_ecm) << std::endl;
  }
  // Incrementar el contador de tiempo
static auto last_force_time = std::chrono::steady_clock::now();
auto current_time = std::chrono::steady_clock::now();
auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_force_time);

  if (elapsed_time.count() >= 5) // Cada 5 segundos
  {
    // Obtener el enlace al que se aplicará la fuerza
    this->model_ = 10;
    gz::sim::Model my_model_f = gz::sim::Model(this->model_);

    gz::sim::Entity link_ptr_f = my_model_f.LinkByName(_ecm, "base_link");
    if (!link_ptr_f)
    {
      ignerr << "Link pointer not found para el enlace 'base_link'" << std::endl;
      return;
    }

    gz::sim::Link link_base = gz::sim::Link(link_ptr_f);

    // Obtener la pose del enlace desde el EntityComponentManager
    auto poseComp = _ecm.Component<gz::sim::components::Pose>(link_base.Entity());
    if (!poseComp)
    {
      ignerr << "No se pudo obtener el componente Pose del enlace" << std::endl;
      return;
    }

    // La pose está disponible en poseComp->Data()
    gz::math::Pose3d world_pose = poseComp->Data();
    igndbg << "Pose del enlace: " << world_pose << std::endl;

    // Fuerza en el marco local
    gz::math::Vector3d local_force(100.0, 100.0, 100.0); // Fuerza de 1000 N en el eje X local

    // Convertir la fuerza al marco global
    gz::math::Vector3d world_force = world_pose.Rot().RotateVector(local_force);

    // Aplicar la fuerza en el marco global
    link_base.AddWorldForce(const_cast<gz::sim::EntityComponentManager&>(_ecm), world_force);

    // Verificar y mostrar el nombre del enlace
    if (auto link_name = link_base.Name(const_cast<gz::sim::EntityComponentManager&>(_ecm)); link_name.has_value())
    {
      ignerr << "Fuerza aplicada: " << world_force << " al enlace: " << link_name.value() << std::endl;
    }
    else
    {
      ignerr << "No se pudo obtener el nombre del enlace" << std::endl;
    }

    // Actualizar el tiempo de la última fuerza aplicada
    //last_force_time = current_time;
  }
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
