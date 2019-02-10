#include "object.hpp"
#include <iostream>

ng2::Object::Object(id_t id, std::shared_ptr<Collider> col, const Material &mat, phys_t m, int lyrs, phys_t gscale) : id(id), pcollider(col),
                                                                                   material(mat), layers(lyrs), grav_scale(gscale)
                                                                                
{
    set_mass(m);
}

ng2::Object::~Object()
{
    // delete collider;
}

void ng2::Object::set_mass(phys_t value)
{
    _mass = value;
    _mass_inv = 1 / value;
}

void ng2::Object::apply_impulse(Vec2 j)
{
    velocity += j * mass_inv;
}

// // TODO to these to a new file
// //AABB
// ng2::AABB::AABB(phys_t w, phys_t h) : Collider{}, width(w), height(h)
// {
// }

// //CircleCollider
// ng2::CircleCollider::CircleCollider(phys_t r) : Collider{}, r(r) {}

// void ng2::Object::update_collider()
// {
//     switch (pcollider->type)
//     {
//     case AABB_T:
//     {
//         AABB &c = *std::static_pointer_cast<AABB>(pcollider);
//         phys_t hw = c.width / 2;
//         phys_t hh = c.height / 2;
//         c.max.x = tf.position.x + hw;
//         c.min.x = tf.position.x - hw;
//         c.max.y = tf.position.y + hh;
//         c.min.y = tf.position.y - hh;
//         break;
//     }
//     case CIRCLE_T:
//     {
//         break;
//     }
//     }
// }
