#ifndef NG2_COLLISION_HPP
#define NG2_COLLISION_HPP

#include "object.hpp"
#include <vector>
#include <list>

namespace ng2
{
typedef std::pair<objptr, objptr> objp_pair;
typedef std::unique_ptr<Object> u_objptr;

struct ColState
{
    u_objptr oa;
    u_objptr ob;
    short n_pen;      // number of b vertices in a (capped at 2)
    int b_verts[2]; // vertices of b in a
    int a_sides[2]; // sides of a closest to b_verts
    phys_t pen[2];  // penetration distances
};
// bool AABBvAABB(Manifold &m);
// bool CirclevCircle(Manifold &m);

// detect points in b that penetrate a's sides. normally this should be
// called twice with objects swapped
bool objects_collide(const Object& oa, const Object& ob);
void detect_collision(ColState& state);
void resolve_collision(const ColState &m);
void positional_correction(ColState &m);

//broad phase
void generate_pairs(const std::list<objptr> &bodies, std::list<objp_pair> &pairs);
} // namespace ng2

#endif