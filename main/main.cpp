#include <iostream>
//#include "../ng2/interface/world.hpp"
#include "../ng2/interface/timestepper.hpp"
#include "routines/basicRoutine.hpp"

int main()
{
    ng2::World world(1);

    ng2::BasicRoutine routine(world, 60, 40, ng2::Vec2{20.f, 20.f});

    // std::vector<ng2::Vec2> tri_vert{ng2::Vec2{4.f, 0}, ng2::Vec2{4.f, 4.f}, ng2::Vec2{0.f, 0.f}};
    // ng2::objptr triangle = routine.add_polygon(tri_vert, ng2::Vec2{24.f, 24.f}, 0.f);
    // triangle->ang_velocity = 3.f;
    // triangle->velocity = ng2::Vec2{5.f, 5.f};

    // std::vector<ng2::Vec2> hexa_vert {
    //     ng2::Vec2{5.f, 0.f},
    //     ng2::Vec2{0.f, 0.f},
    //     ng2::Vec2{3.f, 5.7f},
    //     ng2::Vec2{6.f, 4.f},
    //     ng2::Vec2{-1.f, 2.f},
    //     ng2::Vec2{2.f, 6.f},
    // };
    // ng2::objptr hexagon = routine.add_polygon(hexa_vert, ng2::Vec2{30.f, 30.f}, 0.f);
    // hexagon->ang_velocity = 1.5f;
    // hexagon->velocity = ng2::Vec2{-6.f, 0.f};

    // std::vector<ng2::Vec2> rect1_vert = {ng2::Vec2{0.f, 0.f}, ng2::Vec2{4.f, 0.f}, ng2::Vec2{4.f, 4.f}, ng2::Vec2{0.f, 4.f}};
    // ng2::objptr rect1 = routine.add_polygon(rect1_vert, ng2::Vec2{20.f, 30.f}, 0.f, true, 0.f);
    // rect1->velocity.x = 8.f;
    // rect1->ang_velocity = 10.f;

    // std::vector<ng2::Vec2> rect2_vert = {ng2::Vec2{0.f, 0.f}, ng2::Vec2{4.f, 0.f}, ng2::Vec2{4.f, 4.f}, ng2::Vec2{0.f, 4.f}};
    // ng2::objptr rect2 = routine.add_polygon(rect2_vert, ng2::Vec2{40.f, 30.f}, 0.f);
    // rect2->velocity.x = -10.f;
    // rect2->ang_velocity = -5.f;

    std::vector<ng2::Vec2> rect1_vert = {ng2::Vec2{0.f, 0.f}, ng2::Vec2{15.f, 0.f}, ng2::Vec2{15.f, 7.f}, ng2::Vec2{0.f, 7.f}};
    ng2::objptr fixed_rect = routine.add_polygon(rect1_vert, ng2::Vec2{30.f, 20.f}, 0.f, true, 0.f);

    std::vector<ng2::Vec2> rect2_vert = {ng2::Vec2{0.f, 0.f}, ng2::Vec2{4.f, 0.f}, ng2::Vec2{4.f, 4.f}, ng2::Vec2{0.f, 4.f}};
    ng2::objptr rect2 = routine.add_polygon(rect2_vert, ng2::Vec2{40.f, 30.f}, 0.f);
    rect2->velocity.x = -5.f;
    rect2->ang_velocity = -5.f;

    ng2::TimeStepper stepper(routine, 40);
    stepper.init();
    stepper.loop_start();
    return 0;
}
