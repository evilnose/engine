#include <iostream>
//#include "../ng2/interface/world.hpp"
#include "../ng2/interface/timestepper.hpp"
#include "routines/basicRoutine.hpp"

int main()
{
    ng2::World world;
    world.set_global_gravity(0.f);

    ng2::BasicRoutine routine(world, 600, 400, ng2::Vec2{2.f, 2.f});

    std::vector<ng2::Vec2> tri_vert{ng2::Vec2{40.f, 0}, ng2::Vec2{40.f, 40.f}, ng2::Vec2{0.f, 0.f}};
    ng2::objptr triangle = routine.add_polygon(tri_vert, ng2::Vec2{240.f, 240.f}, 0.f);
    // triangle->ang_velocity = 3.f;
    triangle->velocity = ng2::Vec2{50.f, 50.f};

    std::vector<ng2::Vec2> hexa_vert {
        ng2::Vec2{50.f, 0.f},
        ng2::Vec2{0.f, 0.f},
        ng2::Vec2{30.f, 57.f},
        ng2::Vec2{60.f, 40.f},
        ng2::Vec2{-10.f, 20.f},
        ng2::Vec2{20.f, 60.f},
    };
    ng2::objptr hexagon = routine.add_polygon(hexa_vert, ng2::Vec2{400.f, 300.f}, 0.f);
    hexagon->ang_velocity = 1.5f;
    hexagon->velocity = ng2::Vec2{-60.f, 0.f};

    // std::vector<ng2::Vec2> rect1_vert = {ng2::Vec2{0.f, 0.f}, ng2::Vec2{40.f, 0.f}, ng2::Vec2{40.f, 40.f}, ng2::Vec2{0.f, 40.f}};
    // ng2::objptr rect1 = routine.add_polygon(rect1_vert, ng2::Vec2{30.f, 30.f}, 0.f);

    // std::vector<ng2::Vec2> rect2_vert = {ng2::Vec2{0.f, 0.f}, ng2::Vec2{40.f, 0.f}, ng2::Vec2{40.f, 40.f}, ng2::Vec2{0.f, 40.f}};
    // ng2::objptr rect2 = routine.add_polygon(rect2_vert, ng2::Vec2{80.f, 30.f}, 0.f);
    // rect2->velocity.x = -2.f;

    ng2::TimeStepper stepper(routine, 40);
    stepper.init();
    stepper.loop_start();
    return 0;
}
