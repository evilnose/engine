#ifndef NG2_OBJECT_H
#define NG2_OBJECT_H

#include <stdexcept>

#include "math2d.hpp"
#include "state.hpp"
#include "material.hpp"

namespace ng2
{
class Collider; // definition below; grouped with subclasses

struct Transform
{
    Vec2 position;
    Vec2 ang_position;
};

class Object
{
  public:
    Object(id_t, Collider&, const Material&);

    const id_t id;

    // properties
    Collider& collider; // TODO should be const ref once generic colliders are done
    const Material& material;
    phys_t grav_scale; // 1.0 for normal gravity
    int layers;
    const phys_t &mass = _mass;
    const phys_t &mass_inv = _mass_inv;

    // states
    Transform tf;
    Vec2 velocity;
    Vec2 force;

    void update_collider();
    void set_mass(phys_t value);

  private:
    phys_t _mass;
    phys_t _mass_inv;
};

// TODO no longer needed once generic collision detection
// algorithm is implemented. Remove then.
enum ColliderType {
    AABB_T,
    CIRCLE_T
};

struct Collider {
    ColliderType type;
};

struct AABB : public Collider
{
    AABB(phys_t h, phys_t w);
    phys_t height;
    phys_t width;
    Vec2 min;
    Vec2 max;
};

struct CircleCollider : public Collider
{
    CircleCollider(phys_t r);
    phys_t r; // radius
};

struct Manifold
{
    Object &a;
    Object &b;
    phys_t penetration;
    Vec2 normal;
};
} // namespace ng2

#endif
