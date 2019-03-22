#include "world.hpp"
#include "../collision/collision.hpp"
#include <ctime>
#include <algorithm>

ng2::World::World() : global_gravity_{9.8f}, global_gravity{global_gravity}, grav_accel{0.f, -9.8f}
{
}

ng2::World::~World()
{
    objects.clear(); // free object pointers
}

void ng2::World::add_object(objptr pobj)
{
    objects.emplace_back(pobj);
}

void ng2::World::step(phys_t dt)
{
    // TODO this bad boy can fit so much parallelization in it

    // update velocity
    for (auto &obj : objects)
    {
        obj->velocity += (obj->force * obj->mass_inv + grav_accel * obj->grav_scale) * dt;
        // TODO update angular velocity
        obj->tf.position += obj->velocity * dt;

        obj->tf.ang_position += obj->ang_velocity * dt;
        obj->pcollider->update_collider(obj->tf.ang_position);
        // printf("%f, %f\n", obj.tf.position.x, obj.tf.position.y);
    }

    std::list<objp_pair> pairs;
    generate_pairs(objects, pairs);

    for (auto const &pair : pairs)
    {
        // ColState state = detect_collision(*pair.first, *pair.second);
        // bool colliding = detect_collision(*pair.first, *pair.second) || detect_collision(*pair.second, *pair.first);
        // resolve_collision(m);
        // positional_correction(m);
    }
}

void ng2::World::set_global_gravity(phys_t val)
{
    global_gravity_ = val;
    grav_accel.y = val;
}
