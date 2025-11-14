// src/ball_leash_plugin.cc
#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/math/Vector3.hh>

namespace ball_leash
{
  class BallLeashPlugin : 
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
  {
    private:
      gz::math::Vector3d anchorPoint;
      double maxDistance{0.6};
      double stiffness{200.0};
      double damping{10.0};
      std::string ballModelName;
      gz::sim::Model model{gz::sim::kNullEntity};
      gz::sim::Entity ballLinkEntity{gz::sim::kNullEntity};
      bool initialized{false};

    public:
      void Configure(
        const gz::sim::Entity &/*_entity*/,
        const std::shared_ptr<const sdf::Element> &_sdf,
        gz::sim::EntityComponentManager &/*_ecm*/,
        gz::sim::EventManager &/*_eventMgr*/) override
      {
        if (_sdf->HasElement("ball_model_name"))
          this->ballModelName = _sdf->Get<std::string>("ball_model_name");
        else
          this->ballModelName = "end_ball";

        if (_sdf->HasElement("anchor_point"))
          this->anchorPoint = _sdf->Get<gz::math::Vector3d>("anchor_point");
        else
          this->anchorPoint = gz::math::Vector3d(0.78, 0.55, 2.10);

        if (_sdf->HasElement("max_distance"))
          this->maxDistance = _sdf->Get<double>("max_distance");

        if (_sdf->HasElement("stiffness"))
          this->stiffness = _sdf->Get<double>("stiffness");

        if (_sdf->HasElement("damping"))
          this->damping = _sdf->Get<double>("damping");

        gzmsg << "BallLeashPlugin configured:\n"
              << "  Model: " << this->ballModelName << "\n"
              << "  Anchor: " << this->anchorPoint << "\n"
              << "  Max distance: " << this->maxDistance << "m\n"
              << "  Stiffness: " << this->stiffness << "\n"
              << "  Damping: " << this->damping << "\n";
      }

      void PreUpdate(
        const gz::sim::UpdateInfo &/*_info*/,
        gz::sim::EntityComponentManager &_ecm) override
      {
        if (!this->initialized)
        {
          auto ballEntity = _ecm.EntityByComponents(
            gz::sim::components::Model(),
            gz::sim::components::Name(this->ballModelName));

          if (ballEntity == gz::sim::kNullEntity)
            return;

          this->model = gz::sim::Model(ballEntity);
          this->ballLinkEntity = this->model.CanonicalLink(_ecm);

          if (this->ballLinkEntity == gz::sim::kNullEntity)
            return;

          this->initialized = true;
          gzmsg << "BallLeashPlugin: Initialized with ball model\n";
        }

        if (!this->initialized)
          return;

        // Get link wrapper
        gz::sim::Link link(this->ballLinkEntity);

        // Get world pose
        auto worldPoseOpt = link.WorldPose(_ecm);
        if (!worldPoseOpt.has_value())
          return;
        
        gz::math::Pose3d worldPose = worldPoseOpt.value();
        gz::math::Vector3d ballPos = worldPose.Pos();

        // Get world linear velocity  
        auto worldVelOpt = link.WorldLinearVelocity(_ecm);
        if (!worldVelOpt.has_value())
          return;
        
        gz::math::Vector3d ballVel = worldVelOpt.value();

        double distance = ballPos.Distance(this->anchorPoint);

        // Apply spring force if exceeding max distance
        if (distance > this->maxDistance)
        {
          gz::math::Vector3d direction = (this->anchorPoint - ballPos).Normalized();
          double extension = distance - this->maxDistance;
          
          // Spring force
          gz::math::Vector3d springForce = direction * this->stiffness * extension;
          
          // Damping force
          double radialVel = ballVel.Dot(direction);
          gz::math::Vector3d dampingForce = direction * this->damping * radialVel;
          
          // Total force
          gz::math::Vector3d totalForce = springForce + dampingForce;
          
          // Apply force
          link.AddWorldForce(_ecm, totalForce);
        }
      }
  };
}

GZ_ADD_PLUGIN(
  ball_leash::BallLeashPlugin,
  gz::sim::System,
  ball_leash::BallLeashPlugin::ISystemConfigure,
  ball_leash::BallLeashPlugin::ISystemPreUpdate)

// Add this namespace export
GZ_ADD_PLUGIN_ALIAS(ball_leash::BallLeashPlugin, "ball_leash::BallLeashPlugin")