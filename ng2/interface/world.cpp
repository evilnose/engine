#include "world.hpp"
#include "../collision/collision.hpp"
#include <ctime>
#include <algorithm>

ng2::World::World(real tscale) : global_gravity_{9.8f}, global_gravity{global_gravity_}, grav_accel{0.f, -9.8f},
    time_scale(tscale)
{
}

// ng2::World::~World()
// {
// }

void ng2::World::add_object(objptr pobj)
{
    if (pobj->movable)
        moveable_objects.emplace_back(pobj);
    else
        fixed_objects.emplace_back(pobj);
}

void ng2::World::step(real dt)
{
    dt *= time_scale;
    // TODO this bad boy can fit so much parallelization in it
    resolve_collisions(moveable_objects, fixed_objects, dt);

    // update velocity
    for (auto &obj : moveable_objects)
    {
        obj->velocity += (obj->force * obj->get_mass_inv() + grav_accel * obj->grav_scale) * dt;
        obj->tf.position += obj->velocity * dt;

        obj->tf.ang_position += obj->ang_velocity * dt;
        obj->pcollider->update_collider(obj->tf.ang_position);
        // printf("%f, %f\n", obj.tf.position.x, obj.tf.position.y);
    }

    /*
    std::list<objp_pair> pairs;
    generate_pairs(moveable_objects, pairs);

    for (auto const &pair : pairs)
    {
        // ColState state = detect_collision(*pair.first, *pair.second);
        // bool colliding = detect_collision(*pair.first, *pair.second) || detect_collision(*pair.second, *pair.first);
        // resolve_collision(m);
        // positional_correction(m);
    }
    */
}

void ng2::World::set_global_gravity(real val)
{
    global_gravity_ = val;
    grav_accel.y = val;
}
