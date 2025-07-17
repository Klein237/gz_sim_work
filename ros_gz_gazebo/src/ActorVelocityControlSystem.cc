#include <memory>
#include <chrono>
#include <mutex>

#include <gz/transport/Node.hh>
#include <gz/msgs/twist.pb.h>

#include <gz/sim/System.hh>
#include <gz/sim/Types.hh>
#include <gz/sim/Actor.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Actor.hh>
#include <gz/plugin/Register.hh>

// Plugin pour Gazebo Sim Harmonic v8
class ActorVelocityControlSystem
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
  ActorVelocityControlSystem()
  {
    node = std::make_shared<gz::transport::Node>();
  }

  // Configure: trouver l'entité actor1 et s'abonner
  void Configure(const gz::sim::Entity & /*_world*/,
                 const std::shared_ptr<const sdf::Element> & /*_sdf*/,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager & /*_evt*/) override
  {
    // Parcours des entités pour trouver actor1
    _ecm.Each<gz::sim::components::Name, gz::sim::components::Actor>(
      [&](const gz::sim::Entity &ent,
          const gz::sim::components::Name *_name,
          const gz::sim::components::Actor *) -> bool
      {
        if (_name->Data() == "box")
          actorEntity = ent;
        return true;
      });

    // Souscription au topic transport gz.msgs.Twist
    node->Subscribe("/actor1/cmd_vel",
      &ActorVelocityControlSystem::OnTwist, this);
  }

  // Callback: enregistrer le dernier Twist
  void OnTwist(const gz::msgs::Twist &_msg)
  {
    std::lock_guard<std::mutex> lock(mtx);
    lastTwist = _msg;
    haveTwist = true;
  }

  // Pré-update: intégration et application de la pose
  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override
  {
    if (actorEntity == gz::sim::kNullEntity || !haveTwist)
      return;

    std::lock_guard<std::mutex> lock(mtx);
    gz::sim::Actor actor{actorEntity};

    auto maybePose = actor.TrajectoryPose(_ecm);
    auto pose = maybePose.value_or(actor.WorldPose(_ecm).value());

    // dt en secondes
    double dt = std::chrono::duration<double>(_info.dt).count();

    auto lv = lastTwist.linear();
    auto av = lastTwist.angular();

    // Mouvement linéaire et rotation Z
    pose.Pos().X(pose.Pos().X() + lv.x() * dt);
    pose.Pos().Y(pose.Pos().Y() + lv.y() * dt);
    double yaw = pose.Rot().Euler().Z() + av.z() * dt;
    pose.Rot() = gz::math::Quaterniond(0, 0, yaw);

    actor.SetTrajectoryPose(_ecm, pose);
  }

private:
  std::shared_ptr<gz::transport::Node> node;
  gz::sim::Entity actorEntity{gz::sim::kNullEntity};
  gz::msgs::Twist lastTwist;
  bool haveTwist{false};
  std::mutex mtx;
};

// Enregistrement du plugin
GZ_ADD_PLUGIN(
  ActorVelocityControlSystem,
  gz::sim::System,
  ActorVelocityControlSystem::ISystemConfigure,
  ActorVelocityControlSystem::ISystemPreUpdate
)
