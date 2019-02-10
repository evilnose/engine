#include "collision.hpp"
#include <cmath>
#include <utility>
#include <set>
#include <algorithm>
#include <iostream>
#include "math2d.hpp"

#define POS_CORRECTION_RATIO 0.2
#define POS_CORRECTION_SLOP 0.01

//FUNCTIONS

// returns true if a and b intersect, and false otherwise.
// returns normal in the axis that takes least movement to un-overlap
// bool ng2::AABBvAABB(Manifold &m)
// {
//     const AABB &a = *std::static_pointer_cast<AABB>(m.a.pcollider);
//     const AABB &b = *std::static_pointer_cast<AABB>(m.b.pcollider);
//     phys_t xpos = a.max.x - b.min.x; // try to move b to a right
//     phys_t xneg = a.min.x - b.max.x; // try to move b to a left
//     phys_t ypos = a.max.y - b.min.y;
//     phys_t yneg = a.min.y - b.max.y;

//     if (xpos <= 0 || xneg >= 0 || ypos <= 0 || yneg >= 0)
//         return false;
    

//     // prefer positives when equal
//     phys_t xdir = fabs(xpos) <= fabs(xneg) ? xpos : xneg;
//     phys_t ydir = fabs(ypos) <= fabs(yneg) ? ypos : yneg;
//     auto xabs = (phys_t)fabs(xdir), yabs = (phys_t)fabs(ydir);

//     if (xabs <= yabs) // prefer x when equal
//     {
//         m.penetration = xabs;
//         m.normal = Vec2{xdir, 0};
//     }
//     else
//     {
//         m.penetration = yabs;
//         m.normal = Vec2{0, ydir};
//     }

//     return true;
// }

// // update manifold and return true if collide
// bool ng2::CirclevCircle(Manifold &m)
// {
//     const CircleCollider &a = *std::static_pointer_cast<CircleCollider>(m.a.pcollider);
//     const CircleCollider &b = *std::static_pointer_cast<CircleCollider>(m.b.pcollider);

//     Vec2 n = m.a.tf.position - m.b.tf.position;
//     phys_t r = a.r + b.r;

//     phys_t d = n.len_sq(); // square of length b/c cheaper
//     if (d > r * r)
//         return false;

//     if (d != 0)
//     {
//         d = (phys_t)sqrt(d); // take sqrt of d for penetration
//         m.penetration = r - d;
//         m.normal = n / d; // unit vec
//     }
//     else
//     {
//         // circles are in same position
//         m.penetration = std::min(a.r, b.r);
//         m.normal = Vec2{1, 0};
//     }
//     return true;
// }

// // NOTE m->a should be AABB* and m->b CircleCollider
// bool ng2::AABBvCircle(Manifold &m)
// {
//     const AABB &a = *std::static_pointer_cast<AABB>(m.a.pcollider);
//     const CircleCollider &b = *std::static_pointer_cast<CircleCollider>(m.b.pcollider);

//     Vec2 ab = m.a.tf.position - Vec2{(a.min.x + a.max.x) / 2, (a.min.y + a.max.y) / 2};
//     phys_t hwidth = (a.max.x - a.min.x) / 2;
//     phys_t hheight = (a.max.y - a.min.y) / 2;

//     Vec2 closest{clip(ab.x, -hwidth, hwidth), clip(ab.y, -hheight, hheight)};
//     bool inside = (ab == closest);
//     if (inside)
//     {
//         // move to closest edge instead
//         if (hwidth - fabs(ab.x) <= hheight - fabs(ab.y))
//             // closer to x
//             closest.x = (closest.x >= 0) * hwidth; // prefer right
//         else
//             closest.y = (closest.y >= 0) * hheight;
//     }

//     Vec2 normal = ab - closest;
//     // dist from center of circle to closest pt on AABB squared
//     phys_t dsq = normal.len_sq();
//     if (!inside && pow(b.r, 2) > dsq)
//         return false; // no collision

//     if (inside)
//         normal = -normal; // flip normal to move inside circle out

//     m.normal = normal;
//     m.penetration = (phys_t)(b.r - sqrt(dsq));
//     return true;
// }
void ng2::detect_collision(const Manifold &m)
{
    if (m.a.pcollider->ctype == POLYGON && m.a.pcollider->ctype == POLYGON)
    {
        
    }
    else
    {
        std::cerr << "Collider Types currently not supported." << std::endl;
    }
    
}

void ng2::resolve_collision(const Manifold &m)
{
    Vec2 n = m.normal;
    phys_t e = std::min(m.a.material.restitution, m.b.material.restitution);
    phys_t rel_v = (m.b.velocity - m.a.velocity).dot(n);
    if (rel_v > 0)
        return; // a and b are already moving apart
    phys_t j = -(1 + e) * ((m.b.velocity - m.a.velocity).dot(n)) /
               (m.a.mass_inv + m.b.mass_inv); // solve for impulse scalar
    Vec2 impulse = n * j;
    m.a.velocity -= impulse * m.a.mass_inv;
    m.b.velocity += impulse * m.b.mass_inv;
}

void ng2::positional_correction(Manifold &m)
{
    if ((m.penetration -= POS_CORRECTION_SLOP) > 0)
    {
        Vec2 corr = m.penetration / (m.a.mass_inv + m.b.mass_inv) *
                    POS_CORRECTION_RATIO * m.normal;
        m.a.tf.position += m.a.mass_inv * corr;
        m.b.tf.position += m.b.mass_inv * corr;
    }
}

void ng2::generate_pairs(const std::list<obj_ptr> &bodies, std::list<objp_pair> &pairs)
{
    pairs.clear();

    for (auto i = bodies.begin(); i != bodies.end(); i++)
    {
        auto j = i;
        for (++j; j != bodies.end(); j++)
        {
            // not in same layer
            if (!((*i)->layers & (*j)->layers))
                continue;

            // now i and j both have updated bounding boxes

            // TODO modify condition
            if (true)
                pairs.emplace_back(*i, *j);
        }
    }
}
