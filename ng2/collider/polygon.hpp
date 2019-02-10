#ifndef NG2_POLYGON_HPP
#define NG2_POLYGON_HPP

#include "../collision/collider.hpp"
#include <vector>

namespace ng2
{
    class Polygon : public Collider
    {
        public:
            Polygon(const std::vector<Vec2>& vertices, bool sorted_clockwise=false, phys_t angular_pos=0.f);

            virtual void update_collider(phys_t new_ang) override;

            // get points in clockwise order TODO remove
            const std::vector<Vec2>& get_vertices();

            unsigned int nvertices() const;

            const Vec2& vertex_at(unsigned int index) const;

        private:
            Vec2 com;
            // vertices in given orientation. sorted cw
            std::vector<Vec2> vertices;
            // vertices rotated by angle given by last update_collider()
            std::vector<Vec2> rotated_vertices;
            phys_t angular_pos;

            void sort_vertices(std::vector<Vec2>& vertices);
    };
}

#endif