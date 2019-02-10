#ifndef NG2_COLLIDER_HPP
#define NG2_COLLIDER_HPP

#include "math2d.hpp"

namespace ng2
{
enum ColliderType
{
    POLYGON,
    CIRCLE,
};
class Collider
{
  public:
    Collider(ColliderType ctype);
    virtual void update_collider(phys_t angular_pos) = 0;
    ColliderType ctype;
};
} // namespace ng2

#endif