#ifndef NG2_MATH2D_HPP
#define NG2_MATH2D_HPP

#include <algorithm>

namespace ng2
{
typedef float real;
struct Vec2
{
    real x;
    real y;
    Vec2() : x(0.f), y(0.f){};
    Vec2(real x, real y) : x(x), y(y){};
    Vec2(const Vec2 &u);
    Vec2 &operator*=(real n);
    Vec2 &operator/=(real n);
    Vec2 operator+(const Vec2 &u) const;
    Vec2 &operator+=(const Vec2 &u);
    Vec2 operator-(const Vec2 &u) const;
    Vec2 operator-() const;
    Vec2 &operator-=(const Vec2 &u);
    bool operator==(const Vec2 &u) const;

    real dot(const Vec2 &u) const;
    real len_sq() const;
    real len() const;
    void normalize();
    Vec2 normalized() const;
};
Vec2 operator*(const Vec2 &u, real n);
Vec2 operator*(real n, const Vec2 &u);
Vec2 operator/(const Vec2 &u, real n);
Vec2 operator/(real n, const Vec2 &u);
real dist_sq(Vec2 u, Vec2 v);
real dist(Vec2 u, Vec2 v);
real determinant(Vec2 u, Vec2 v);
Vec2 angular2tangential(const Vec2& u, real n);

template <typename T>
T clip(const T& n, const T& lower, const T& upper) {
  return std::max(lower, std::min(n, upper));
}
} // namespace ng2

#endif