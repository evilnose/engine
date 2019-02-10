#ifndef NG2_TIMESTEPPER_HPP
#define NG2_TIMESTEPPER_HPP

#include "routines.hpp"

namespace ng2
{
class TimeStepper
{
  public:
    TimeStepper(Routine &routine, int framerate, phys_t acum_cap=0.2f);
    void init();
    void loop_start();
    void loop_stop();
    bool& looping = in_loop;

    const int framerate;
    const phys_t dt;
    const phys_t acum_cap;
  private:
    Routine& routine;
    bool in_loop;
};
} // namespace ng2
#endif