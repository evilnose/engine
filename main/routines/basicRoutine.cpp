#include "basicRoutine.hpp"
#include "../../ng2/collider/polygon.hpp"
#include "../../ng2/collision/math2d.hpp"
// #include "../../ng2/collision/material.hpp"

#define FRAMERATE 60

ng2::BasicRoutine::BasicRoutine(World &wd, phys_t w, phys_t h, Vec2 s) : Routine(wd), width(w), height(h), scale(s), last_id(0) {
}

// void ng2::BasicRoutine::add_AABB(Vec2 size, Vec2 init_pos, Vec2 init_vel) {
//     // TODO thread safety for last_id; maybe create a new class
//     std::shared_ptr<AABB> pcollider(new AABB(size.x, size.y));
//     std::shared_ptr<Object> o(new Object(last_id++, pcollider, mat::ROCK));
//     o->tf.position = init_pos;
//     o->velocity = init_vel;
//     world.add_object(o);
//     aabb_list.emplace_back(o);
// }

// void ng2::BasicRoutine::add_circle(phys_t radius, Vec2 init_pos, Vec2 init_vel) {
//     std::shared_ptr<CircleCollider> pcollider(new CircleCollider(radius));
//     std::shared_ptr<Object> o(new Object(last_id++, pcollider, mat::ROCK));
//     o->tf.position = init_pos;
//     o->velocity = init_vel;
//     world.add_object(o);
//     circle_list.emplace_back(o);
// }

std::shared_ptr<ng2::Object> ng2::BasicRoutine::add_polygon(const std::vector<Vec2>& points, Vec2 init_pos, phys_t init_angpos, bool sorted_clockwise)
{
    std::shared_ptr<Polygon> pcol(new Polygon(points, sorted_clockwise, init_angpos));
    std::shared_ptr<Object> pobj(new Object(last_id++, pcol, mat::ROCK));
    pobj->tf.position = init_pos;
    pobj->pcollider = pcol;
    world.add_object(pobj);
    polygon_list.emplace_back(pobj);
    return pobj;
}


void ng2::BasicRoutine::generate_polygon(const Vec2& init_pos)
{
    //TODO
}

void ng2::BasicRoutine::init() {
    window.create(sf::VideoMode((unsigned int) (width * scale.x), (unsigned int) (height * scale.y)),
                  "ng2 Window (AxisAligned)");
    window.setFramerateLimit(FRAMERATE);
    window_focus = true;
}

void ng2::BasicRoutine::render_update(phys_t alpha) {
    static const sf::Color prmycolor = sf::Color(66, 134, 244);
    sf::Event event;
    while (window.pollEvent(event)) {
        switch (event.type) {
            case sf::Event::Closed:
                stop();
                window.close();
                break;
            case sf::Event::GainedFocus:
                window_focus = true;
                break;
            case sf::Event::LostFocus:
                window_focus = false;
                break;
        }
    }
    window.clear(sf::Color::White);
    for (const auto&o : polygon_list)
    {
        const Polygon& collider = *std::static_pointer_cast<Polygon>(o->pcollider);
        
        sf::ConvexShape poly;
        unsigned int sz = collider.nvertices();
        poly.setPointCount(sz);
        for (int i = 0; i < sz; i++)
        {
            const Vec2& vertex = collider.vertex_at(i);
            sf::Vector2f pt(vertex.x * scale.x, -vertex.y * scale.y);
            poly.setPoint(i, pt);
        }
        poly.setOutlineColor(prmycolor);
        poly.setOutlineThickness(2);
        poly.setPosition(x_norm(o->tf.position.x), y_norm(o->tf.position.y));

        sf::CircleShape circle(2); // center of mass
        circle.setFillColor(prmycolor);
        circle.setPosition(x_norm(o->tf.position.x), y_norm(o->tf.position.y));

        window.draw(poly);
        window.draw(circle);
    }
    // for (auto &o : aabb_list) {
    //     const AABB& aabb = *std::static_pointer_cast<AABB>(o->pcollider);
    //     sf::RectangleShape rect;
    //     rect.setSize(sf::Vector2f(scale.x * aabb.width, scale.y * aabb.height));
    //     rect.setOutlineColor(sf::Color::Red);
    //     rect.setOutlineThickness(1);
    //     rect.setPosition(x_norm(o->tf.position.x), y_norm(o->tf.position.y));
    //     window.draw(rect);
    // }
    window.display();
}

float ng2::BasicRoutine::x_norm(float x)
{
    return scale.x * x;
}

float ng2::BasicRoutine::y_norm(float y)
{
    return scale.y * (height - y);
}
