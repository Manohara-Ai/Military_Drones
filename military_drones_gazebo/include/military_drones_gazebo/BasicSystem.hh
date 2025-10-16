#ifndef MILITARY_DRONES_GAZEBO__BASIC_SYSTEM_HH_
#define MILITARY_DRONES_GAZEBO__BASIC_SYSTEM_HH_

#include <gz/sim/System.hh>

namespace military_drones_gazebo
{
  class BasicSystem
    : public gz::sim::System,
      public gz::sim::ISystemPostUpdate
  {
  public:
    void PostUpdate(const gz::sim::UpdateInfo &_info,
                    const gz::sim::EntityComponentManager &_ecm) override;
  };
}  // namespace military_drones_gazebo

#endif  // MILITARY_DRONES_GAZEBO__BASIC_SYSTEM_HH_
