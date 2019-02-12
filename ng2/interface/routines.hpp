#ifndef NG2_ROUTINES_HPP
#define NG2_ROUTINES_HPP

#include "world.hpp"

namespace ng2
{
class Routine
{
  public:
    Routine(World &world_) : stopped(false), world(world_) {}

    virtual void init() = 0;

    // called for each physics time-step
    virtual void physics_update(phys_t dt) { world.step(dt); }

    // called when rendering (can be frequent than physics_update())
    virtual void render_update(phys_t alpha) = 0;

    // observed by timestepper, which terminates when stopped_ is true
    const bool &stopped = stopped_;

  private:
    bool stopped_;

  protected:
    World &world;
    void stop() { stopped_ = true; }
};
} // namespace ng2

#endif