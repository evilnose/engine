#include "object.hpp"
#include <iostream>

ng2::Object::Object(id_t id, Collider &col, const Material &mat) : id(id), collider(col),
                                                                   material(mat) {
}

void ng2::Object::set_mass(phys_t value) {
    _mass = value;
    _mass_inv = 1 / value;
}

//AABB
ng2::AABB::AABB(phys_t h, phys_t w) : Collider{}, height(h), width(w) {
}

//CircleCollider
ng2::CircleCollider::CircleCollider(phys_t r) : Collider{}, r(r) {}

void ng2::Object::update_collider() {
    switch (collider.type) {
        case AABB_T: {
            auto c = (AABB &) collider;
            phys_t hw = c.width / 2;
            phys_t hh = c.height / 2;
            c.max.x = tf.position.x + hw;
            c.min.x = tf.position.x - hw;
            c.max.y = tf.position.y + hh;
            c.min.y = tf.position.y - hh;
            break;
        }
        case CIRCLE_T: {
            break;
        }
    }
}
