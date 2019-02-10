#include <iostream>
//#include "../ng2/interface/world.hpp"
#include "../ng2/interface/timestepper.hpp"
#include "routines/basicRoutine.hpp"

int main()
{
    ng2::World world;
    // world.set_global_gravity(0.f);

    ng2::BasicRoutine routine(world, 600, 400, ng2::Vec2{2.f, 2.f});
    std::vector<ng2::Vec2> vert{ng2::Vec2{40.f, 0}, ng2::Vec2{40.f, 40.f}, ng2::Vec2{0.f, 0.f}};
    std::shared_ptr<ng2::Object> triangle = routine.add_polygon(vert, ng2::Vec2{40.f, 40.f}, 1.f);
    triangle->ang_velocity = 1.f;
    triangle->velocity = ng2::Vec2{50.f, 50.f};

    ng2::TimeStepper stepper(routine, 40);
    stepper.init();
    stepper.loop_start();
    return 0;
}
