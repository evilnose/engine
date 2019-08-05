#include "math2d.hpp"
#include <cmath>
#include <cassert>

ng2::Vec2::Vec2(const Vec2& u)
{
    x = u.x;
    y = u.y;
}

ng2::Vec2 ng2::operator*(const ng2::Vec2 &u, real n)
{
    return ng2::Vec2{u.x * n, u.y * n};
}

ng2::Vec2 ng2::operator*(real n, const ng2::Vec2 &u)
{
    return u * n;
}

ng2::Vec2 &ng2::Vec2::operator*=(real n)
{
    x *= n, y *= n;
    return *this;
}

bool ng2::Vec2::operator==(const Vec2& u) const
{
    return (x == u.x && y == u.y);
}

ng2::Vec2 ng2::operator/(const ng2::Vec2 &u, real n)
{
    return ng2::Vec2{u.x / n, u.y / n};
}

ng2::Vec2 ng2::operator/(real n, const ng2::Vec2 &u)
{
    return u / n;
}

ng2::Vec2 &ng2::Vec2::operator/=(real n)
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

ng2::real ng2::Vec2::dot(const ng2::Vec2& u) const
{
    return x * u.x + y * u.y;
}

ng2::real ng2::Vec2::len_sq() const
{
    return x * x + y * y;
}

void ng2::Vec2::normalize()
{
    real norm = len();
    assert(norm != 0);
    x /= norm;
    y /= norm;
}

ng2::real ng2::determinant(Vec2 u, Vec2 v)
{
    return u.x * v.y - u.y * v.x;
}

// angular velocity to tangential velocity
ng2::Vec2 ng2::angular2tangential(const Vec2& radius, real ang_v)
{
    return Vec2{radius.y, -radius.x} * ang_v;
} 

ng2::Vec2 ng2::Vec2::normalized() const
{
    real norm = len();
    return Vec2{ x / norm, y / norm };
}

ng2::real ng2::Vec2::len() const
{
    return (real) sqrt(len_sq());
}

ng2::real ng2::dist_sq(Vec2 u, Vec2 v)
{
    return pow(v.x - u.x, 2) + pow(v.y - u.y, 2);
}

ng2::real ng2::dist(Vec2 u, Vec2 v)
{
    return sqrt(dist_sq(u, v));
}
