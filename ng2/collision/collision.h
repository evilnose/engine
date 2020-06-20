#pragma once

#include "object.h"
#include "polygon.h"
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

// populated before collision resolution; only used
// in Manifold struct
struct ContactData {
  Vec2 point;  // initialized during polygon2polygon, etc.
  Vec2 pivot_a;  // initialized during init();
  Vec2 pivot_b; 
  // specialized denominator value used during impulse computation.
  real denom;  // initialized during init()
};

// NOTE only Manifold.ob could be a fixed object
struct Manifold
{
  Object& oa;  // must be set when Manifold is first constructed
  Object& ob;  // NOTE this could be a fixed object

  real r; // restitution;
  real mu_static;
  real mu_kinetic;

  real penetration;  // same as point (above)
  Vec2 normal;  // same as (above) point
  uint8_t count;  // number of contact points (1-2); initialized same time as point

  // Intermediate values stored/modified during computation
  ContactData contact_data[2];

  // The accumulated impulse (from the collision resolution trials). This is 
  // read from and changed over iterations of impulse resolution.
  real accum_j;

  // initialize frictional constants (r, mu_*)
  void init_frictional();

  // compute contact data. This is done after
  // penetration, normal, and count are set
  void init_contact_data();
};

// bool AABBvAABB(Manifold &m);
// bool CirclevCircle(Manifold &m);

// manifolds is an output parameter, to which we push the constructed manifolds
void resolve_all_collisions(std::vector<objptr>& movable_objects, std::vector<objptr>& fixed_objects,
  real dt, std::vector<Manifold>& manifolds);

void positional_correction(Manifold& man);

// TODO this could all be made private, probably
void polygon2polygon(Manifold& man);

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


