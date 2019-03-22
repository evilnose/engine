#ifndef NG2_WORLD_HPP
#define NG2_WORLD_HPP

#include "../collision/object.hpp"
#include <list>

namespace ng2
{
class World
{
  public:
    const phys_t &global_gravity;

    World();
    ~World(); // TODO

    void add_object(objptr pobj);

    void step(phys_t dt);

    void set_global_gravity(phys_t val);

  private:
    std::list<objptr> objects;
    phys_t global_gravity_;
    ng2::Vec2 grav_accel;
};
} // namespace ng2

#endif