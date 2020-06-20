#include "world.h"
#include "../collision/collision.h"
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
    std::vector<Manifold> manifolds;
    resolve_all_collisions(moveable_objects, fixed_objects, dt, manifolds);

    // update velocity
    for (auto op : moveable_objects)
    {
        op->velocity += (op->force * op->get_mass_inv() + grav_accel * op->grav_scale) * dt;
        op->tf.position += op->velocity * dt;

        op->tf.ang_position += op->ang_velocity * dt;
        op->pcollider->update_collider(op->tf.ang_position);
        // printf("%f, %f\n", obj.tf.position.x, obj.tf.position.y);
    }

    for (Manifold& man : manifolds) {
        positional_correction(man);
    }
}

void ng2::World::set_global_gravity(real val)
{
    global_gravity_ = val;
    grav_accel.y = val;
}
