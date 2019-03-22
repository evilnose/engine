#ifndef NG2_POLYGON_HPP
#define NG2_POLYGON_HPP

#include "../collision/collider.hpp"
#include <vector>

namespace ng2
{
    class Polygon : public Collider
    {
        public:
            // TODO replace vectors with hidden unique_ptr arrays
            Polygon(const std::vector<Vec2>& vertices, bool sorted_clockwise=false, phys_t angular_pos=0.f);

            virtual void update_collider(phys_t new_ang) override;

            unsigned int n_vertices() const;

            const Vec2& vertex_at(unsigned int index) const;

            const Vec2& normal_at(unsigned int index) const;

            // return vertex *index* that is farthest along dir
            int get_support(const Vec2& dir) const;

        private:
            Vec2 com;
            // vertices in given orientation. sorted counter-clockwise
            std::vector<Vec2> vertices;

            std::vector<Vec2> normals;

            // vertices rotated by angle given by last update_collider()
            std::vector<Vec2> rotated_vertices;

            std::vector<Vec2> rotated_normals;
            phys_t angular_pos;

            void sort_vertices(std::vector<Vec2>& vertices);
    };
}

#endif