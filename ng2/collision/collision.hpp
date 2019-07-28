#ifndef NG2_COLLISION_HPP
#define NG2_COLLISION_HPP

#include "object.hpp"
#include "polygon.hpp"
#include <vector>

namespace ng2
{
typedef std::pair<objptr, objptr> objp_pair;
typedef std::unique_ptr<Object> u_objptr;
typedef std::shared_ptr<Polygon> ppoly;

struct ColState
{
    u_objptr oa;
    u_objptr ob;
    short n_pen;      // number of b vertices in a (capped at 2)
    int b_verts[2]; // vertices of b in a
    int a_sides[2]; // sides of a closest to b_verts
    real pen[2];  // penetration distances
};
// bool AABBvAABB(Manifold &m);
// bool CirclevCircle(Manifold &m);

void resolve_collisions(std::vector<objptr>& moveable_objects,
        std::vector<objptr>& fixed_objects, real dt);

real polygon2polygon(const Object& oa, const Object& ob, Manifold& man);

/*
Detect points in b that penetrate a's faces. This should be
called twice with objects swapped.

oa          one object
ob          the other object. This function checks if any points of ob is
            inside oa
face_idx    if any face of oa is penetrated by ob, this number will be updated
            with the index of oa's face which is penetrated most deeply
    
return      the largest penetration distance of a vertex of ob into oa. It is
            positive if there is a penetration, and -1 if there is not
*/
real objects_collide(const Object& oa, const Object& ob, int& face_idx);

// for polygons only
void find_incident_face(const Vec2& ref_normal, ppoly inc_poly, Vec2 out_face[2]);

// clip the points of inc_face against the line defined by ref_point and normal
// i.e. part of inc_face beyond ref_point in the direction of negative normal
// is clipped, and the modification is reflected in inc_face
bool clip_face(Vec2 inc_face[2], const Vec2& ref_point, const Vec2& normal);

} // namespace ng2

#endif
