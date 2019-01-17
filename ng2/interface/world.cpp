#include "scene.hpp"
#include "../collision/collision.hpp"
#include <ctime>
#include <algorithm>

#define FPS (time_t) 100.f
#define DT 1 / FPS
#define MAX_ACUM (time_t) 0.2f

ng2::Scene::Scene()
{
    state.objects = &objects;
}

void ng2::Scene::start()
{
    hd_init(state);
    std::time_t cur_time;
    static std::time_t accumulator = 0;
    std::time_t last_frame = std::time(nullptr);
    running = true;
    while (running)
    {   
        cur_time = std::time(nullptr);
        accumulator += std::min(cur_time - last_frame, MAX_ACUM); //TODO OOP?
        last_frame = cur_time;

        // let physics catch up
        while (accumulator > DT)
        {
            hd_fixed_update(state);
            physics_update();
            accumulator -= DT;
        }

        if (hd_render)
        {
            hd_render(state, 0.f);
        }
    }
}

void ng2::Scene::stop()
{
    running = false;
}

void ng2::Scene::add_object(ng2::Object obj) {
    objects.emplace_back(obj);
}

void ng2::Scene::physics_update() {
    // TODO this bad boy can fit so much parallelization in it

    // update velocity
    for (Object obj : objects)
    {
        obj.velocity += obj.force * obj.mass_inv;
    }

    // TODO update position
    // TODO ordering

    // create manifolds
    std::list<Manifold> manifolds;
    for (std::list<Object>::iterator i = objects.begin(); i != objects.end(); i++) {
        std::list<Object>::iterator j = i;
        for (++j; j != objects.end(); j++) {
            Manifold m{*i, *j}; // leave the other two as default
            resolve_collision(m);
            positional_correction(m);
        }
    }
}
