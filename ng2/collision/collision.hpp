#ifndef NG2_COLLISION_HPP
#define NG2_COLLISION_HPP

#include "object.hpp"
#include <list>

namespace ng2
{ 
typedef std::shared_ptr<ng2::Object> obj_ptr;
typedef std::pair<obj_ptr, obj_ptr> objp_pair;
// bool AABBvAABB(Manifold &m);
// bool CirclevCircle(Manifold &m);
// bool AABBvCircle(Manifold &m);
void detect_collision(const Manifold &m);
void resolve_collision(const Manifold &m);
void positional_correction(Manifold &m);

//broad phase
void generate_pairs(const std::list<obj_ptr>& bodies, std::list<objp_pair>& pairs);
} // namespace ng2

#endif