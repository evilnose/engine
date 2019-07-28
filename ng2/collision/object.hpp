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
  real ang_position;
};

class Object
{
public:
  Object(id_t id, colptr pcollider, const Material &mat, real mass = 1.f,
          int layers = 1, real grav_scale = 1.f, bool movable = true);
  ~Object();

  const id_t id;

  // properties
  colptr pcollider; // TODO should be const once generic collider are done
  const Material &material;
  real grav_scale; // 1.0 for normal gravity
  int layers;
  bool movable;

  // states
  Transform tf;
  Vec2 velocity;
  Vec2 force;
  real ang_velocity;

  // apply to object an impulse j, changing it velocity
  void apply_impulse(Vec2 j, Vec2 contact);

  // void apply_torque();

  void update_collider();

  real get_mass();
  real get_mass_inv();
  real get_inertia();
  real get_inertia_inv();
  void set_mass(real value);

private:
  real mass;
  real _mass_inv;
  real inertia;
  real _inertia_inv;
};

// struct Collider {
//     ColliderType type;
// };

// struct AABB : public Collider
// {
//     AABB(real w, real h);
//     real width;
//     real height;
//     Vec2 min;
//     Vec2 max;
// };

// struct CircleCollider : public Collider
// {
//     CircleCollider(real r);
//     real r; // radius
// };

struct Manifold
{
  real penetration;
  Vec2 normal;
  Vec2 contact_pts[2];
  uint8_t count;
};
} // namespace ng2

#endif
