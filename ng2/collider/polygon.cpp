#include "polygon.hpp"
#include <cmath>
#include <cassert>

ng2::Polygon::Polygon(const std::vector<Vec2>& vert, bool sorted_cw, float angpos) :
    Collider(POLYGON), angular_pos(0.f), vertices(vert)
{
    if (!sorted_cw)
        sort_vertices(vertices);

    phys_t area = 0.f;
    unsigned int range = vertices.size() - 1;
    for (int i = 0, j; i <= range; i++)
    {
        j = (i + 1) % (range + 1); // loop around for last one
        area += vertices.at(i).x * vertices.at(j).y -
            vertices.at(i).y * vertices.at(j).x;
    }
    area /= 2.f;

    for (int i = 0, j; i <= range; i++)
    {
        j = (i + 1) % (range + 1); // loop around for last one
        phys_t second_term = vertices.at(i).x * vertices.at(j).y +
            vertices.at(i).y * vertices.at(j).x;
        com.x += (vertices.at(i).x + vertices.at(j).x) * second_term;
        com.y += (vertices.at(i).y + vertices.at(j).y) * second_term;
    }
    
    com.x /= (6.f * area);
    com.y /= (6.f * area);

    if (com.x < 0 || com.y < 0)
    {
        com.x = -com.x;
        com.y = -com.y;
    }
    assert(com.x >= 0);
    assert(com.y >= 0);

    for (auto& vertex : vertices)
    {
        vertex -= com; // offset
    }
    rotated_vertices.reserve(vertices.size());
    std::copy(vertices.begin(), vertices.end(), std::back_inserter(rotated_vertices));
    update_collider(angpos);
}

unsigned int ng2::Polygon::nvertices() const
{
    return rotated_vertices.size();
}

const ng2::Vec2& ng2::Polygon::vertex_at(unsigned int index) const
{
    return rotated_vertices.at(index);
}

void ng2::Polygon::sort_vertices(std::vector<Vec2>& to_sort)
{
    Vec2 inside_pt;
    for (const auto& vertex : to_sort)
    {
        inside_pt += vertex;
    }
    inside_pt /= to_sort.size();
    
    // TODO OPTIMIZE: cache atan2 results
    std::sort(to_sort.begin(), to_sort.end(), [](const Vec2& v1, const Vec2& v2) {
        return atan2(v1.y, v1.x) < atan2(v2.y, v2.x);
    });
}

const std::vector<ng2::Vec2>& ng2::Polygon::get_vertices()
{
    return rotated_vertices;
}

void ng2::Polygon::update_collider(phys_t new_ang)
{
    if (angular_pos == new_ang) return;
    angular_pos = new_ang;

    // make rotation matrix
    phys_t sinval = sin(new_ang);
    phys_t cosval = cos(new_ang);
    Vec2 xcol{cosval, sinval};
    Vec2 ycol{-sinval, cosval};

    for (unsigned int i = 0; i < vertices.size(); i++)
    {
        const Vec2& v = vertices.at(i);
        Vec2& rv = rotated_vertices.at(i);
        rv.x = v.dot(xcol);
        rv.y = v.dot(ycol);
    }
}