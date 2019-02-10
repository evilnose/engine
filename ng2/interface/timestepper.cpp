#include "timestepper.hpp"
#include "../utils/timer.hpp"
#include <ctime>

ng2::TimeStepper::TimeStepper(Routine& r, int fr, float ac) : routine(r), framerate(fr), dt(1.f / fr), acum_cap(ac)
{
}

void ng2::TimeStepper::init()
{
    routine.init();
}

void ng2::TimeStepper::loop_start()
{
    in_loop = true;
    float accumulator = 0;
    Timer timer;
    Timer persistent;
    float elapsed = 0;
    while (in_loop && !routine.stopped)
    {
        accumulator += timer.elapsed();
        timer.reset();

        if (accumulator > acum_cap) {
            accumulator = acum_cap;
        }

        while (accumulator > dt)
        {
            routine.physics_update(dt);
            accumulator -= dt;
            elapsed += dt;
        }
        routine.render_update(accumulator / dt);
    }
}

void ng2::TimeStepper::loop_stop()
{
    in_loop = false;
}
