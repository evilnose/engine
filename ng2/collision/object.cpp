#include "object.hpp"
#include "polygon.hpp"
#include <cmath>
#include <iostream>

ng2::Object::Object(id_t id, colptr col, const Material &mat, real m,
        int lyrs, real gscale, bool movable_) : 
    id(id), pcollider(col), material(mat), layers(lyrs), grav_scale(gscale),
    movable(movable_)
                                                                                
{
    set_mass(m);
}

ng2::Object::~Object()
{
    // delete collider;
}

ng2::real ng2::Object::get_mass()
{
    return mass;
}

ng2::real ng2::Object::get_mass_inv()
{
    return _mass_inv;
}

ng2::real ng2::Object::get_inertia()
{
    return inertia;
}

ng2::real ng2::Object::get_inertia_inv()
{
    return _inertia_inv;
}

void ng2::Object::set_mass(real value)
{
    mass = value;
    _mass_inv = 1 / value;

    // calculate inertia
    if (pcollider->ctype == POLYGON)
    {
        real denom = 0.f;
        real numer = 0.f;
        auto poly = (Polygon&) *pcollider;
        for (int i = 0; i < poly.n_vertices(); i++)
        {
            int j = (i + 1) % poly.n_vertices(); // TODO optimize this loop
            // using two variables

            Vec2 v1 = poly.vertex_at(i);
            Vec2 v2 = poly.vertex_at(j);
            real a = (real) fabs(determinant(v1, v2));
            real b = v1.dot(v1) + v1.dot(v2) + v2.dot(v2);
            denom += a * b;
            numer += a;
        }

        inertia = mass / 6.f * denom / numer;
        _inertia_inv = 1 / inertia;
    }
    else
    {
        printf("ERROR: inertia of circle of implemented\n");
    }    
}

void ng2::Object::apply_impulse(Vec2 j, Vec2 contact)
{
    velocity += j * _mass_inv;
    ang_velocity += determinant(j, contact) * _inertia_inv;
}

// // TODO to these to a new file
// //AABB
// ng2::AABB::AABB(real w, real h) : Collider{}, width(w), height(h)
// {
// }

// //CircleCollider
// ng2::CircleCollider::CircleCollider(real r) : Collider{}, r(r) {}

// void ng2::Object::update_collider()
// {
//     switch (pcollider->type)
//     {
//     case AABB_T:
//     {
//         AABB &c = *std::static_pointer_cast<AABB>(pcollider);
//         real hw = c.width / 2;
//         real hh = c.height / 2;
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
