#ifndef NG2_MATH2D_H
#define NG2_MATH2D_H

namespace ng2
{
typedef float phys_t;
struct Vec2
{
    phys_t x;
    phys_t y;
    Vec2() : x(0.f), y(0.f){};
    Vec2(phys_t x, phys_t y) : x(x), y(y){};
    Vec2(const Vec2 &u) = default;
    Vec2 &operator*=(phys_t n);
    Vec2 &operator/=(phys_t n);
    Vec2 operator+(const Vec2 &u) const;
    Vec2 &operator+=(const Vec2 &u);
    Vec2 operator-(const Vec2 &u) const;
    Vec2 operator-() const;
    Vec2 &operator-=(const Vec2 &u);
    bool operator==(const Vec2 &u);

    phys_t dot(const Vec2 &u);
    phys_t len_sq();
    phys_t len();
};
Vec2 operator*(const Vec2 &u, phys_t n);
Vec2 operator*(phys_t n, const Vec2 &u);
Vec2 operator/(const Vec2 &u, phys_t n);
Vec2 operator/(phys_t n, const Vec2 &u);
phys_t dist_sq(Vec2 u, Vec2 v);
phys_t dist(Vec2 u, Vec2 v);

template <typename T>
T clip(const T &n, const T &lower, const T &upper);
} // namespace ng2

#endif