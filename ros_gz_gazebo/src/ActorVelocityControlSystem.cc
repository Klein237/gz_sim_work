#include <memory>
#include <chrono>
#include <mutex>

#include <gz/transport/Node.hh>
#include <gz/msgs/twist.pb.h>

#include <gz/sim/System.hh>
#include <gz/sim/Types.hh>
#include <gz/sim/components/Name.hh>
// Correction : inclure Pose.hh pour accéder à WorldPose
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Actor.hh>
#include <gz/sim/Actor.hh>
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

  // Configure : trouver l'entité person_standing et s'abonner
  void Configure(const gz::sim::Entity & /*_world*/,
                 const std::shared_ptr<const sdf::Element> & /*_sdf*/,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager & /*_evt*/) override
  {
    // Parcours des entités pour trouver l’actor
    _ecm.Each<gz::sim::components::Name, gz::sim::components::Actor>(
      [&](const gz::sim::Entity &ent,
          const gz::sim::components::Name *_name,
          const gz::sim::components::Actor *) -> bool
      {
        if (_name->Data() == "person")
          actorEntity = ent;
        return true;
      });

    // Souscription au topic (vérifier namespace exact avec `gz topic -l`)
    node->Subscribe("/model/person_standing/cmd_vel",
                    &ActorVelocityControlSystem::OnTwist, this);
  }

  // Callback : stocke le dernier Twist reçu
  void OnTwist(const gz::msgs::Twist &_msg)
  {
    std::lock_guard<std::mutex> lock(mtx);
    lastTwist = _msg;
    haveTwist = true;
  }

  // Pré-update : applique directement la nouvelle pose
  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override
  {
    if (actorEntity == gz::sim::kNullEntity || !haveTwist)
      return;

    std::lock_guard<std::mutex> lock(mtx);

    // Récupère le composant WorldPose
    auto poseComp =
      _ecm.Component<gz::sim::components::WorldPose>(actorEntity);
    if (!poseComp)
      return;

    // Copie de la pose actuelle
    auto pose = poseComp->Data();

    // Delta-time en secondes
    double dt = std::chrono::duration<double>(_info.dt).count();

    // Vitesses linéaires et angulaires
    auto lv = lastTwist.linear();
    auto av = lastTwist.angular();

    // Mise à jour de la position X/Y
    pose.Pos().X(pose.Pos().X() + lv.x() * dt);
    pose.Pos().Y(pose.Pos().Y() + lv.y() * dt);

    // Mise à jour du yaw (rotation Z)
    double yaw = pose.Rot().Euler().Z() + av.z() * dt;
    pose.Rot() = gz::math::Quaterniond(0, 0, yaw);

    // Écrase la composante WorldPose dans l’ECM
    _ecm.SetComponentData<gz::sim::components::WorldPose>(
      actorEntity, pose);
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
