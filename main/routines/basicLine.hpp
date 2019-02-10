#ifndef NG2_BASICLINE_HPP
#define NG2_BASICLINE_HPP

#include "../../ng2/interface/routines.hpp"

namespace ng2
{

class BasicLineWorld : public Routine
{
    public:
    void add_polygon(std::list<Vec2> vertices);

    private:
    std::list<Vec2> polygons;
    std::list<Vec2> circles;
};
} // namespace ng2

#endif