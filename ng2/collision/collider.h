#pragma once

#include "math2d.h"

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
    virtual void update_collider(real angular_pos) = 0;
    ColliderType ctype;
};
} // namespace ng2

