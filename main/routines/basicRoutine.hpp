#ifndef NG2_AXISALIGNED_HPP
#define NG2_AXISALIGNED_HPP

#include <memory>
#include <SFML/Graphics.hpp>
#include "../../ng2/interface/routines.hpp"
#include "../../ng2/collision/math2d.hpp"

namespace ng2
{
class BasicRoutine : public Routine
{
  public:
    BasicRoutine(World& world, phys_t width, phys_t height, Vec2 scale);
    // note that size should be {width, height}
    // TODO add variadic overload
    std::shared_ptr<Object> add_polygon(const std::vector<Vec2>& points,
      Vec2 init_pos=Vec2{0.f, 0.f}, phys_t init_angpos=0, bool sorted_clockwise=false);
    void generate_polygon(const Vec2& init_pos);
    void add_circle(phys_t r, Vec2 init_pos = Vec2{0.f, 0.f}, Vec2 init_vel = Vec2{0.f, 0.f});

  private:
    float width;
    float height;
    Vec2 scale;

    std::list<std::shared_ptr<Object>> polygon_list;
    // std::list<std::shared_ptr<Object>> circle_list;

    id_t last_id;

    sf::RenderWindow window;
    bool window_focus;

    virtual void init() override;
    
    // not going to override physics update
    // virtual void physics_update() override;
    virtual void render_update(phys_t alpha) override;

    void draw_vector(const Vec2& origin, const Vec2& vec, 
      const sf::Color& c_origin = sf::Color::Black, const sf::Color& c_end = sf::Color::Red);

    void draw_normals(const std::vector<Vec2>& normals);

    // normalize a 2d point
    Vec2 normalize_point(Vec2 pt) const;

    // normalize a 2d vector
    Vec2 normalize_vec(Vec2 vec) const;

    sf::Vector2f vec2ToVector2f(const Vec2& vec) const;
};
} // namespace ng2
#endif