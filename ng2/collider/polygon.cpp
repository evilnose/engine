#include <cmath>
#include <cassert>
#include <map>
#include "polygon.hpp"
#include "../collision/utils.hpp"

typedef ng2::ID<ng2::Vec2> VecID;

ng2::Polygon::Polygon(const std::vector<Vec2> &vert, bool sorted_cw, float angpos) : Collider(POLYGON), angular_pos(0.f), vertices(vert)
{
    if (!sorted_cw)
        sort_vertices(vertices);

    phys_t area = 0.f;
    unsigned int sz = vertices.size();
    for (int i = 0, j; i < sz; i++)
    {
        j = (i + 1) % sz; // loop around for last one
        area += vertices.at(i).x * vertices.at(j).y -
                vertices.at(j).x * vertices.at(i).y;
    }
    area /= 2.f;
    assert(area > 0);

    for (int i = 0, j; i < sz; i++)
    {
        j = (i + 1) % sz; // loop around for last one
        phys_t second_term = vertices.at(i).x * vertices.at(j).y -
                             vertices.at(j).x * vertices.at(i).y;
        com.x += (vertices.at(i).x + vertices.at(j).x) * second_term;
        com.y += (vertices.at(i).y + vertices.at(j).y) * second_term;
    }

    com.x /= (6.f * area);
    com.y /= (6.f * area);

    // if (com.x < 0 || com.y < 0)
    // {
    //     com.x = -com.x;
    //     com.y = -com.y;
    // }
    // assert(com.x >= 0);
    // assert(com.y >= 0);

    for (auto &vertex : vertices)
    {
        vertex -= com; // offset
    }

    Vec2 line;
    for (int i = 0, j; i < sz; i++)
    {
        j = (i + 1) % sz;
        line = vertices.at(j) - vertices.at(i);
        // since points are sorted counter-clockwise
        normals.emplace_back(Vec2{line.y, -line.x});
    }

    // copy vertices/normals
    rotated_vertices.reserve(sz);
    std::copy(vertices.begin(), vertices.end(), std::back_inserter(rotated_vertices));

    rotated_normals.reserve(sz);
    std::copy(normals.begin(), normals.end(), std::back_inserter(rotated_normals));

    update_collider(angpos);
}

unsigned int ng2::Polygon::n_vertices() const
{
    return rotated_vertices.size();
}

const ng2::Vec2 &ng2::Polygon::vertex_at(unsigned int index) const
{
    return rotated_vertices.at(index);
}

const ng2::Vec2 &ng2::Polygon::normal_at(unsigned int index) const
{
    return rotated_normals.at(index);
}

void ng2::Polygon::sort_vertices(std::vector<Vec2> &to_sort)
{
    // get an arbitrary point that is inside
    Vec2 inside_pt;
    for (const Vec2 &vertex : to_sort)
    {
        inside_pt += vertex;
    }
    inside_pt /= to_sort.size();

    std::vector<VecID> vids;
    int id = 0;
    for (const Vec2 &v : to_sort)
        vids.emplace_back(VecID{v, id++});

    std::map<int, float> key_map;
    for (const auto &v : vids)
    {
        // cache atan results
        key_map[v.id] = atan2(v.item.y - inside_pt.y, v.item.x - inside_pt.x);
    }

    std::sort(vids.begin(), vids.end(), [key_map](const VecID &v1, const VecID &v2) {
        auto e1 = key_map.find(v1.id);
        auto e2 = key_map.find(v2.id);

        assert(e1 != key_map.end());
        assert(e2 != key_map.end());

        return e1->second < e2->second;
    });

    to_sort.clear();
    for (const VecID &vid : vids)
    {
        to_sort.emplace_back(vid.item);
    }
}

int ng2::Polygon::get_support(const Vec2 &dir) const
{
    const auto cmp = [dir](const Vec2 &v1, const Vec2 &v2) { return dir.dot(v1) > dir.dot(v2); };
    const auto max_it = std::max_element(rotated_vertices.begin(),
                                         rotated_vertices.end(),
                                         cmp);
    return std::distance(rotated_vertices.begin(), max_it);
}

void ng2::Polygon::update_collider(phys_t new_ang)
{
    if (angular_pos == new_ang)
        return;
    angular_pos = new_ang;

    // make rotation matrix
    phys_t sinval = sin(new_ang);
    phys_t cosval = cos(new_ang);
    Vec2 xcol{cosval, sinval};
    Vec2 ycol{-sinval, cosval};

    for (unsigned int i = 0; i < vertices.size(); i++)
    {
        // update vertices
        const Vec2 &v = vertices.at(i);
        Vec2 &rv = rotated_vertices.at(i);
        rv.x = v.dot(xcol);
        rv.y = v.dot(ycol);

        // update normals
        const Vec2 &n = normals.at(i);
        Vec2 &rn = rotated_normals.at(i);
        rn.x = n.dot(xcol);
        rn.y = n.dot(ycol);
    }
}