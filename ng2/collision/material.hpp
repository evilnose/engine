#ifndef NG2_MATERIAL_H
#define NG2_MATERIAL_H

#include "math2d.hpp"

namespace ng2 {
    struct Material {
        phys_t density;
        phys_t restitution;
    };

   namespace mat {
       Material ROCK{0.6, 0.1};
       Material WOOD{0.3, 0.2};
       Material METAL{1.2, 0.05};
       Material BOUNCY_BALL{0.3, 0.8};
       Material PILLOW{0.1, 0.2};
   }
} // namespace ng2

#endif