// #include <SFML/Graphics.hpp>
#include "math2d.hpp"

#ifndef NG2_UTILS_HPP
#define NG2_UTILS_HPP
namespace ng2
{
// currently unused
// sf::Vector2f vec2ToVector2f(const Vec2& u, real scale=1.f);

template <class T>
struct ID
{
    T item;
    int id;

    ID(T it, int id) : item(it), id(id) {}

    bool operator()(T &lhs, T &rhs) const
    {
        return lhs.id > rhs.id;
    }
};

template <class T>
struct IDComp
{
    bool operator() (ID<T> &lhs, ID<T> &rhs) const
    {
        return lhs.id > rhs.id;
    }
};
} // namespace ng2
#endif