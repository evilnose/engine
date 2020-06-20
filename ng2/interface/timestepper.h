#pragma once

#include "routines.h"

namespace ng2
{
class TimeStepper
{
  public:
    TimeStepper(Routine &routine, int framerate, real acum_cap=0.2f);
    void init();
    void loop_start();
    void loop_stop();
    bool& looping = in_loop;

    const int framerate;
    const real dt;
    const real acum_cap;
  private:
    Routine& routine;
    bool in_loop;
};
} // namespace ng2
