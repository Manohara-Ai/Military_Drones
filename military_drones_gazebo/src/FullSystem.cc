#include <string>
#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include "military_drones_gazebo/FullSystem.hh"

GZ_ADD_PLUGIN(
    military_drones_gazebo::FullSystem,
    gz::sim::System,
    military_drones_gazebo::FullSystem::ISystemConfigure,
    military_drones_gazebo::FullSystem::ISystemPreUpdate,
    military_drones_gazebo::FullSystem::ISystemUpdate,
    military_drones_gazebo::FullSystem::ISystemPostUpdate,
    military_drones_gazebo::FullSystem::ISystemReset
)

namespace military_drones_gazebo 
{

void FullSystem::Configure(const gz::sim::Entity &_entity,
                const std::shared_ptr<const sdf::Element> &_element,
                gz::sim::EntityComponentManager &_ecm,
                gz::sim::EventManager &_eventManager)
{
  gzdbg << "military_drones_gazebo::FullSystem::Configure on entity: " << _entity << std::endl;
}

void FullSystem::PreUpdate(const gz::sim::UpdateInfo &_info,
                           gz::sim::EntityComponentManager &_ecm)
{
  if (!_info.paused && _info.iterations % 1000 == 0)
  {
    gzdbg << "military_drones_gazebo::FullSystem::PreUpdate" << std::endl;
  }
}

void FullSystem::Update(const gz::sim::UpdateInfo &_info,
                        gz::sim::EntityComponentManager &_ecm)
{
  if (!_info.paused && _info.iterations % 1000 == 0)
  {
    gzdbg << "military_drones_gazebo::FullSystem::Update" << std::endl;
  }
}

void FullSystem::PostUpdate(const gz::sim::UpdateInfo &_info,
                            const gz::sim::EntityComponentManager &_ecm) 
{
  if (!_info.paused && _info.iterations % 1000 == 0)
  {
    gzdbg << "military_drones_gazebo::FullSystem::PostUpdate" << std::endl;
  }
}

void FullSystem::Reset(const gz::sim::UpdateInfo &_info,
                       gz::sim::EntityComponentManager &_ecm)
{
  gzdbg << "military_drones_gazebo::FullSystem::Reset" << std::endl;
}
}  // namespace military_drones_gazebo
