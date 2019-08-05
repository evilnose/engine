#ifndef NG2_MATERIAL_HPP
#define NG2_MATERIAL_HPP

#include "math2d.hpp"

namespace ng2
{
struct Material
{
    real density;
    real restitution;
    real mu_kinetic;
    real mu_static;
};

namespace mat
{
static const Material ROCK{0.6, 0.1, 0.4, 0.5};
static const Material WOOD{0.3, 0.2, 0.2, 0.2};
static const Material METAL{1.2, 0.05, 0.6, 0.7};
static const Material BOUNCY_BALL{0.3, 0.8, 0.5, 0.5};
static const Material PILLOW{0.1, 0.2, 0.3, 0.3};
} // namespace mat
} // namespace ng2

#endif