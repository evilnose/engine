#include "math2d.hpp"
#include <cmath>

ng2::Vec2::Vec2(const Vec2& u)
{
    x = u.x;
    y = u.y;
}

ng2::Vec2 ng2::operator*(const ng2::Vec2 &u, phys_t n)
{
    return ng2::Vec2{u.x * n, u.y * n};
}

ng2::Vec2 ng2::operator*(phys_t n, const ng2::Vec2 &u)
{
    return u * n;
}

ng2::Vec2 &ng2::Vec2::operator*=(phys_t n)
{
    x *= n, y *= n;
    return *this;
}

bool ng2::Vec2::operator==(const Vec2& u) const
{
    return (x == u.x && y == u.y);
}

ng2::Vec2 ng2::operator/(const ng2::Vec2 &u, phys_t n)
{
    return ng2::Vec2{u.x / n, u.y / n};
}

ng2::Vec2 ng2::operator/(phys_t n, const ng2::Vec2 &u)
{
    return u / n;
}

ng2::Vec2 &ng2::Vec2::operator/=(phys_t n)
{
    x /= n, y /= n;
    return *this;
}

ng2::Vec2 ng2::Vec2::operator+(const Vec2 &u) const
{
    return Vec2{x + u.x, y + u.y};
}

ng2::Vec2 &ng2::Vec2::operator+=(const Vec2 &u)
{
    x += u.x, y += u.y;
    return *this;
}

ng2::Vec2 ng2::Vec2::operator-(const Vec2 &v) const
{
    return Vec2{x - v.x, y - v.y};
}

ng2::Vec2 ng2::Vec2::operator-(void) const
{
    return Vec2{-x, -y};
}

ng2::Vec2 &ng2::Vec2::operator-=(const Vec2 &v)
{
    x -= v.x;
    y -= v.y;
    return *this;
}

ng2::phys_t ng2::Vec2::dot(const ng2::Vec2& u) const
{
    return x * u.x + y * u.y;
}

ng2::phys_t ng2::Vec2::len_sq() const
{
    return x * x + y * y;
}

ng2::phys_t ng2::Vec2::len() const
{
    return (phys_t) sqrt(len_sq());
}

ng2::phys_t ng2::dist_sq(Vec2 u, Vec2 v)
{
    return pow(v.x - u.x, 2) + pow(v.y - u.y, 2);
}

ng2::phys_t ng2::dist(Vec2 u, Vec2 v)
{
    return sqrt(dist_sq(u, v));
}
