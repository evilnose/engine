#include <iostream>
#include "basicRoutine.hpp"
#include "../../ng2/collision/polygon.hpp"
#include "../../ng2/collision/math2d.hpp"
// #include "../../ng2/collision/material.hpp"

#define FRAMERATE 60

ng2::BasicRoutine::BasicRoutine(World &wd, real w, real h, Vec2 s) : Routine(wd), width(w), height(h), scale(s), last_id(0)
{
}

// void ng2::BasicRoutine::add_AABB(Vec2 size, Vec2 init_pos, Vec2 init_vel) {
//     // TODO thread safety for last_id; maybe create a new class
//     std::shared_ptr<AABB> pcollider(new AABB(size.x, size.y));
//     ng2::objptr o(new Object(last_id++, pcollider, mat::ROCK));
//     o->tf.position = init_pos;
//     o->velocity = init_vel;
//     world.add_object(o);
//     aabb_list.emplace_back(o);
// }

// void ng2::BasicRoutine::add_circle(real radius, Vec2 init_pos, Vec2 init_vel) {
//     std::shared_ptr<CircleCollider> pcollider(new CircleCollider(radius));
//     ng2::objptr o(new Object(last_id++, pcollider, mat::ROCK));
//     o->tf.position = init_pos;
//     o->velocity = init_vel;
//     world.add_object(o);
//     circle_list.emplace_back(o);
// }

ng2::objptr ng2::BasicRoutine::add_polygon(
    const std::vector<Vec2> &points, Vec2 init_pos, real init_angpos, bool fixed, bool sorted_clockwise)
{
    std::shared_ptr<Polygon> pcol(new Polygon(points, sorted_clockwise, init_angpos));
    ng2::objptr pobj(new Object(last_id++, pcol, mat::ROCK, 1.f, !fixed));
    pobj->tf.position = init_pos;
    pobj->tf.ang_position = init_angpos;
    pobj->velocity = ng2::Vec2{0.f, 0.f};
    pobj->ang_velocity = 0.f;
    pobj->pcollider = pcol;
    world.add_object(pobj);
    polygon_list.emplace_back(pobj);
    return pobj;
}

void ng2::BasicRoutine::generate_polygon(const Vec2 &init_pos)
{
    //TODO
}

void ng2::BasicRoutine::init()
{
    window.create(sf::VideoMode((unsigned int)(width * scale.x), (unsigned int)(height * scale.y)),
                  "ng2 Window -- Basic Routine");
    window.setFramerateLimit(FRAMERATE);
    window_focus = true;
}

void ng2::BasicRoutine::render_update(real alpha)
{
    static const sf::Color prmycolor = sf::Color(66, 134, 244);
    sf::Event event;
    while (window.pollEvent(event))
    {
        switch (event.type)
        {
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
    for (const ng2::objptr o : polygon_list)
    {
        const Polygon &collider = *std::static_pointer_cast<Polygon>(o->pcollider);

        sf::ConvexShape poly;
        unsigned int sz = collider.n_vertices();
        poly.setPointCount(sz);
        for (int i = 0; i < sz; i++)
        {
            const Vec2 &vertex = collider.vertex_at(i);
            sf::Vector2f pt(vertex.x * scale.x, -vertex.y * scale.y);
            poly.setPoint(i, pt);
        }
        poly.setOutlineColor(prmycolor);
        poly.setOutlineThickness(2);
        Vec2 norm_pos = normalize_point(o->tf.position);
        poly.setPosition(norm_pos.x, norm_pos.y);

        sf::CircleShape circle(2); // center of mass
        circle.setFillColor(prmycolor);
        circle.setPosition(norm_pos.x, norm_pos.y);

        // draw normals to debug
        // draw_normals(collider.get_normals();

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

void ng2::BasicRoutine::draw_vector(const Vec2 &origin, const Vec2 &vec,
                                    const sf::Color &c_origin, const sf::Color &c_end)
{
    sf::Vertex line[] =
        {
            sf::Vertex(vec2ToVector2f(origin), c_origin),
            sf::Vertex(vec2ToVector2f(origin + vec), c_end)};
    window.draw(line, 2, sf::Lines);
}

// void ng2::BasicRoutine::draw_normals(const std::vector<Vec2>& normals)
// {
//     const Vec2 origin = normalize_point(Vec2{100.f, 100.f});
//     for (const auto& normal : normals)
//     {
//         draw_vector(origin, normalize_vec(normal));
//     }
// }

// normalize a 2d point
ng2::Vec2 ng2::BasicRoutine::normalize_point(Vec2 pt) const
{
    return Vec2{pt.x * scale.x, (height - pt.y) * scale.y};
}

// normalize a 2d vector
ng2::Vec2 ng2::BasicRoutine::normalize_vec(Vec2 vec) const
{
    return Vec2{vec.x * scale.x, -vec.y * scale.y};
}

sf::Vector2f ng2::BasicRoutine::vec2ToVector2f(const Vec2 &vec) const
{
    return sf::Vector2f(vec.x, vec.y);
}
