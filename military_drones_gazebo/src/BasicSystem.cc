#include <string>
#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include "military_drones_gazebo/BasicSystem.hh"

GZ_ADD_PLUGIN(
  military_drones_gazebo::BasicSystem,
  gz::sim::System,
  military_drones_gazebo::BasicSystem::ISystemPostUpdate)

namespace military_drones_gazebo
{

void BasicSystem::PostUpdate(const gz::sim::UpdateInfo &_info,
                             const gz::sim::EntityComponentManager &_ecm)
{
  if (!_info.paused && _info.iterations % 1000 == 0)
  {
    gzdbg << "[military_drones_gazebo::BasicSystem] PostUpdate" << std::endl;
  }
}

}  // namespace military_drones_gazebo
