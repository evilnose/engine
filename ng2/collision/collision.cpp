#include <cmath>
#include <utility>
#include <set>
#include <algorithm>
#include <iostream>
#include <cassert>

#include "math2d.h"
#include "collision.h"
#include "polygon.h"

static constexpr ng2::real POS_CORRECTION_RATIO = 0.2;
static constexpr ng2::real POS_CORRECTION_SLOP = 0.01;
static constexpr ng2::real Y_EPSILON = 0.005; // completely arbitrary
static constexpr ng2::real BIAS_FACTOR = 0.1;
static constexpr ng2::real BIAS_SLOP = 0.05;
static constexpr int K_ITERATIONS = 5;

void ng2::Manifold::init_frictional()
{
    r = fmin(oa.material.restitution, ob.material.restitution);
    mu_static = (oa.material.mu_static + ob.material.mu_static) / 2;
    mu_kinetic = (oa.material.mu_kinetic + ob.material.mu_kinetic) / 2;
}

void ng2::Manifold::init_contact_data() {
    // make sure collision detection is done at this point
    assert(count != 0);
    for (int i = 0; i < count; i++) {
        contact_data[i].pivot_a = contact_data[i].point - oa.tf.position;
        contact_data[i].pivot_b = contact_data[i].point - ob.tf.position;

        contact_data[i].denom = pow(determinant(contact_data[i].pivot_a, normal), 2) * oa.get_inertia_inv() +
                        pow(determinant(contact_data[i].pivot_b, normal), 2) * ob.get_inertia_inv() +
                        oa.get_mass_inv() + ob.get_mass_inv();
    }
}

// Private functions
namespace {
using namespace ng2;

void apply_friction(Manifold& man, real dt) {
    for (int i = 0; i < man.count; i++) {
        Object& oa = man.oa;
        Object& ob = man.ob;
        Vec2 rel_v = ob.velocity - oa.velocity;

        ContactData& cdata = man.contact_data[i];

        real mass_inv_a = oa.get_mass_inv();
        real mass_inv_b = ob.get_mass_inv();
        real inertia_inv_a = oa.get_inertia_inv();
        real inertia_inv_b = ob.get_inertia_inv();

        real denom = pow(determinant(cdata.pivot_a, man.normal), 2) * inertia_inv_a +
                        pow(determinant(cdata.pivot_b, man.normal), 2) * inertia_inv_b +
                        mass_inv_a + mass_inv_b;

        //friction
        // subtract normal component
        Vec2 tangent = rel_v - rel_v.dot(man.normal) * man.normal;
        if (tangent.len_sq() < 1e-7)
            return;
        tangent.normalize();

        // magnitude of impulse caused by tangential velocity diff
        real impulse_offset = -rel_v.dot(tangent) / denom / man.count;

        Vec2 friction;
        // j, the normal impulse * mu_static is the tangential static friction impulse
        if (fabs(impulse_offset) < man.mu_static * man.accum_j)
        {
            friction = tangent * impulse_offset; // offset the tangential velocity difference
        }
        else
        {
            friction = -tangent * man.mu_kinetic * man.accum_j;
        }
        oa.apply_impulse(-friction, cdata.pivot_a);
        ob.apply_impulse(friction, cdata.pivot_b);
    }
}

void resolve_collision(Manifold &man, real dt)
{
    Object& oa = man.oa;
    Object& ob = man.ob;
    real pen = man.penetration;
    int t;

    if (man.count != 0)
    {
        assert(pen >= 0);
        real mass_inv_a = oa.get_mass_inv();
        real mass_inv_b = ob.get_mass_inv();
        real inertia_inv_a = oa.get_inertia_inv();
        real inertia_inv_b = ob.get_inertia_inv();

        for (int i = 0; i < man.count; i++)
        {
            ContactData& cdata = man.contact_data[i];
            Vec2 contact_pt = cdata.point;

            // velocity of b relative to a
            Vec2 rel_v = ob.velocity + angular2tangential(cdata.pivot_b, ob.ang_velocity) -
                         oa.velocity - angular2tangential(cdata.pivot_a, oa.ang_velocity);
            real rel_v_normal = rel_v.dot(man.normal);

            real bias_vel = BIAS_FACTOR / dt * fmin(0.f, pen - BIAS_SLOP);
            bias_vel = 0;
            if (rel_v_normal < 0.f)
                bias_vel = -bias_vel;

            real dj = (-(1 + man.r) * (rel_v_normal + bias_vel)) / cdata.denom / man.count;

            // Thanks Erin Catto! From https://www.youtube.com/watch?v=SHinxAhv1ZE
            real old_j = man.accum_j;
            man.accum_j += dj;
            man.accum_j = fmax(0, man.accum_j);  // accum. impulse should not be negative

            // update delta j to account for the changes
            dj = man.accum_j - old_j;

            Vec2 impulse = dj * man.normal;
            oa.apply_impulse(-impulse, cdata.pivot_a);
            ob.apply_impulse(impulse, cdata.pivot_b);
        }
    }
}

}  // namespace

// Public functions
void ng2::resolve_all_collisions(std::vector<objptr> &objects, std::vector<objptr> &fixed_objects, real dt, std::vector<Manifold>& manifolds)
{
    // TODO do some broad-phase culling here
    for (int i = 0; i < objects.size(); i++)
    {
        for (int j = i + 1; j < objects.size(); j++)
        {
            Manifold man{*objects[i], *objects[j]};
            man.init_frictional();
            polygon2polygon(man);
            if (man.penetration > 0)
            {
                manifolds.push_back(man);
            }
        }

        for (int j = 0; j < fixed_objects.size(); j++)
        {
            Manifold man{*objects[i], *fixed_objects[j]};
            man.init_frictional();
            polygon2polygon(man);
            if (man.penetration > 0)
            {
                manifolds.push_back(man);
            }
        }
    }

    for (int i = 0; i < K_ITERATIONS; i++)
    {
        for (Manifold &man : manifolds)
        {
            resolve_collision(man, dt);
        }
    }

    // tentative friction
    for (Manifold &man : manifolds) {
        apply_friction(man, dt);
    }
    // TODO handle fixed objects
}

void ng2::positional_correction(Manifold& man) {
    real pen = man.penetration - POS_CORRECTION_SLOP;
    real mass_inv_a = man.oa.get_mass_inv();
    real mass_inv_b = man.ob.get_mass_inv();
    if (pen > 0) {
        Vec2 corr = pen / (mass_inv_a + mass_inv_b) *
            POS_CORRECTION_RATIO * man.normal;

        man.oa.tf.position -= mass_inv_a * corr;
        man.ob.tf.position += mass_inv_b * corr;
    }
}

/*
Clip a face penetrating another polygon (incident) against a line. The line originates
from an endpoint of the penetrated face, and is normal to the penetrated face.
If the incident "exceeds" the line, i.e. hs an endpoint outside the line, it is 
clipped to be on the line, here:
            
            /   <- incident face, exceeding penetrated face
           /
          /
         / 
    ----/--             <- penetrated face
       /

          
          /      <- incident face, now clipped (right endpoint changed)
         / 
    ----/--             <- penetrated face
       /

It is possible for both endpoints of the incident face to be clipped due to
numerical errors, in which case, false is returned to indicate this. 
 */
bool ng2::clip_face(Vec2 inc_face[2], const Vec2 &ref_point, const Vec2 &normal)
{
    Vec2 dir1 = inc_face[0] - ref_point;
    Vec2 dir2 = inc_face[1] - ref_point;
    real dot1 = dir1.dot(normal);
    real dot2 = dir2.dot(normal);
    Vec2 out[2];
    int8_t count = 0;

    // don't need to clip
    if (dot1 >= 0)
        out[count++] = inc_face[0];
    if (dot2 >= 0)
        out[count++] = inc_face[1];

    // gotta clip
    if (dot1 * dot2 < 0)
    {
        assert(count != 2);
        out[count++] = inc_face[0] + dot1 / (dot1 - dot2) * (inc_face[1] - inc_face[0]);
    }
    assert(count != 3);
    inc_face[0] = out[0];
    inc_face[1] = out[1];
    return count >= 2;
}

void ng2::find_incident_face(const Vec2 &ref_normal, ppoly inc_poly, Vec2 out_face[2])
{

    // get index of incident normal that is most opposite in direction to
    // ref_normal
    std::vector<Vec2>::const_iterator inc_it = std::min_element(inc_poly->normal_begin(),
                                                                inc_poly->normal_end(), [ref_normal](const Vec2 &n1, const Vec2 &n2) {
                                                                    return ref_normal.dot(n1) < ref_normal.dot(n2);
                                                                });

    int index = std::distance(inc_poly->normal_begin(), inc_it);
    out_face[0] = inc_poly->vertex_at(index);
    out_face[1] = inc_poly->vertex_at((index + 1) % inc_poly->n_vertices());
}

/*
Procedure:
1)  Find penetrations using separating axis theorem
2)  Decide incident and reference polygons
3)  Find the two incident faces in the incident polygon
4)  Clip (i.e. find intersection with if there exists) the incident face against the side planes
    of the reference face.
5)  For the clipped points (intersection or original), if it has a positive penetration, count it
    as a penetrating point. Take average penetration of penetrating points as the final penetration
    and the normal of the reference face as the penetration normal
*/
void ng2::polygon2polygon(Manifold &man)
{
    Object& oa = man.oa;
    Object& ob = man.ob;
    man.penetration = -1; // assume not colliding by default
    // step 1
    int idx_a;
    real pen_a = objects_collide(oa, ob, idx_a);
    if (pen_a < 0)
        return;

    int idx_b;
    real pen_b = objects_collide(ob, oa, idx_b);
    if (pen_b < 0)
        return;

    // step 2
    std::shared_ptr<Polygon> ref_poly;
    std::shared_ptr<Polygon> inc_poly;
    Vec2 ref_pos;
    Vec2 inc_pos;
    int ref_idx; // index of the reference face

    real pen_diff = pen_a - pen_b;
    real pen_eps = pen_a * 0.07;
    bool flipped; // by default, assume a is reference

    // if penetration difference is not sufficiently large, prefer the
    // polygon on top to be reference for improved coherence
    // (I actually have no idea if it has to be this convoluted)
    if (fabs(pen_diff) < pen_eps)
    {
        real y_diff = oa.tf.position.y - ob.tf.position.y;
        if (fabs(y_diff) < Y_EPSILON)
        {
            // tiebreaker. If a is to the left of b, then a is ref
            flipped = oa.tf.position.x < ob.tf.position.x;
        }
        else
        {
            flipped = y_diff < 0;
        }
    }
    else
    {
        flipped = pen_diff < 0;
    }

    if (flipped)
    {
        ref_poly = std::dynamic_pointer_cast<Polygon>(oa.pcollider);
        inc_poly = std::dynamic_pointer_cast<Polygon>(ob.pcollider);
        ref_pos = oa.tf.position;
        inc_pos = ob.tf.position;
        ref_idx = idx_a;
    }
    else
    {
        ref_poly = std::dynamic_pointer_cast<Polygon>(ob.pcollider);
        inc_poly = std::dynamic_pointer_cast<Polygon>(oa.pcollider);
        ref_pos = ob.tf.position;
        inc_pos = oa.tf.position;
        ref_idx = idx_b;
    }

    // step 3
    Vec2 inc_face[2];
    // assert(ref_idx < ref_poly->n_vertices());
    find_incident_face(ref_poly->normal_at(ref_idx), inc_poly, inc_face);
    inc_face[0] += inc_pos;
    inc_face[1] += inc_pos;

    // step 4
    Vec2 ref_p = ref_poly->vertex_at(ref_idx) + ref_pos;
    Vec2 ref_q = ref_poly->vertex_at((ref_idx + 1) % ref_poly->n_vertices()) + ref_pos;
    Vec2 ref_face[2]{ref_p, ref_q};
    Vec2 pq = (ref_q - ref_p);
    pq.normalize();

    if (!clip_face(inc_face, ref_p, pq) || !clip_face(inc_face, ref_q, -pq))
        return;

    // step 5
    // the normal should be from a to b
    Vec2 ref_normal = ref_poly->normal_at(ref_idx).normalized();
    man.normal = flipped ? ref_normal : -ref_normal;
    real penetration_sum = 0.f;
    man.penetration = 0.f;
    for (uint8_t i = 0; i <= 1; i++)
    {
        real pen = ref_normal.dot(inc_face[i] - ref_p);
        if (pen <= 0.f)
        {
            man.contact_data[man.count++].point = inc_face[i];
            man.penetration -= pen;
        }
    }
    if (man.count == 0)
    {
        printf("REEEEE NUMERICAL ISSUES\n");
        man.penetration = 1;
    }
    man.penetration /= man.count;

    man.init_contact_data();
}

// If colliding, return positive real as penetration, or negative if not colliding.
// Also face_idx would be updated as the index of the face of oa that is penetrated the farthest.
ng2::real ng2::objects_collide(const ng2::Object &oa, const ng2::Object &ob, int &face_idx)
{
    if (oa.pcollider->ctype == ng2::POLYGON && ob.pcollider->ctype == ng2::POLYGON)
    {
        auto acol = (const ng2::Polygon &)*oa.pcollider;
        auto bcol = (const ng2::Polygon &)*ob.pcollider;

        ng2::real best_comp = -INFINITY;
        ng2::real comp;
        // iterate over a's faces
        for (int i = 0; i < acol.n_vertices(); i++)
        {
            const ng2::Vec2 &normal = acol.normal_at(i);
            // get vertex of b that is farthest along negative normal of a
            int support = bcol.get_support(-normal);
            // compute penetration
            comp = normal.dot((bcol.vertex_at(support) + ob.tf.position) -
                              (acol.vertex_at(i) + oa.tf.position));

            // TODO maybe there's a safer way to do this?
            if (comp > 1e-7)
            {
                return -1;
            }
            if (comp > best_comp)
            {
                best_comp = comp;
                face_idx = i;
            }
        }
        assert(best_comp < 0);
        return -best_comp;
    }
    else
    {
        std::cerr << "Collider Types currently not supported." << std::endl;
        return -1;
    }
}
