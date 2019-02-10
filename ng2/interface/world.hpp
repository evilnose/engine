#ifndef NG2_WORLD_HPP
#define NG2_WORLD_HPP

#include "../collision/object.hpp"
#include <list>

namespace ng2 {

    class World {
    public:
        const phys_t& global_gravity;

        World();
        ~World(); // TODO

        void add_object(std::shared_ptr<Object> pobj);

        void step(phys_t dt);

        void set_global_gravity(phys_t val);
    private:
        std::list<std::shared_ptr<Object>> objects;
        phys_t global_gravity_;
        ng2::Vec2 grav_accel;
    };
} // namespace ng2

#endif