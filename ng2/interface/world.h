#pragma once

#include <vector>
#include "../collision/object.h"

namespace ng2
{
class World
{
  public:
    const real &global_gravity;

    World(real time_scale = 1.f);
    // ~World(); // TODO

    void add_object(objptr pobj);

    void step(real dt);

    void set_global_gravity(real val);

  private:
    std::vector<objptr> moveable_objects;
    std::vector<objptr> fixed_objects;
    real global_gravity_;
    ng2::Vec2 grav_accel;
    real time_scale; // time speed is multiplied by this factor
};
} // namespace ng2


