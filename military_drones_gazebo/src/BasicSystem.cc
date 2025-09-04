// We'll use a string and the gzmsg command below for a brief example.
// Remove these includes if your plugin doesn't need them.
#include <string>
#include <gz/common/Console.hh>

// This header is required to register plugins. It's good practice to place it
// in the cc file, like it's done here.
#include <gz/plugin/Register.hh>

// Don't forget to include the plugin's header.
#include "military_drones_gazebo/BasicSystem.hh"

// This is required to register the plugin. Make sure the interfaces match
// what's in the header.
IGNITION_ADD_PLUGIN(
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
    igndbg << "military_drones_gazebo::BasicSystem::PostUpdate" << std::endl;
  }
}

}  // namespace military_drones_gazebo
