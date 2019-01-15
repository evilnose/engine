#include "object.hpp"
#include <list>


namespace ng2
{ 
typedef std::pair<ng2::Object, ng2::Object> body_pair;
bool AABBvAABB(Manifold &m);
bool CirclevCircle(Manifold &m);
bool AABBvCircle(Manifold &m);
void resolve_collision(const Manifold &m);
void positional_correction(Manifold &m);

//broad phase
void generate_pairs(std::list<Object>& bodies, std::list<body_pair>& pairs);
} // namespace ng2

