#ifndef NG2_OBJECT_HPP
#define NG2_OBJECT_HPP

#include <stdexcept>
#include <memory>

#include "math2d.hpp"
#include "state.hpp"
#include "material.hpp"
#include "collider.hpp"

namespace ng2
{
class Object;
typedef std::shared_ptr<Object> objptr;
typedef std::shared_ptr<Collider> colptr;
struct Transform
{
  Vec2 position;
  phys_t ang_position;
};

class Object
{
public:
  Object(id_t id, colptr pcollider, const Material &mat, phys_t mass = 1.f, int layers = 1, phys_t grav_scale = 1.f);
  ~Object();

  const id_t id;

  // properties
  colptr pcollider; // TODO should be const once generic collider are done
  const Material &material;
  phys_t grav_scale; // 1.0 for normal gravity
  int layers;
  const phys_t &mass = _mass;
  const phys_t &mass_inv = _mass_inv;

  // states
  Transform tf;
  Vec2 velocity;
  Vec2 force;
  phys_t ang_velocity;

  // apply to object an impulse j, changing it velocity
  void apply_impulse(Vec2 j);

  // void apply_torque();

  void update_collider();

  void set_mass(phys_t value);

private:
  phys_t _mass;
  phys_t _mass_inv;
};

// struct Collider {
//     ColliderType type;
// };

// struct AABB : public Collider
// {
//     AABB(phys_t w, phys_t h);
//     phys_t width;
//     phys_t height;
//     Vec2 min;
//     Vec2 max;
// };

// struct CircleCollider : public Collider
// {
//     CircleCollider(phys_t r);
//     phys_t r; // radius
// };

struct Manifold
{
  Object &a;
  Object &b;
  phys_t penetration;
  Vec2 normal;
};
} // namespace ng2

#endif
