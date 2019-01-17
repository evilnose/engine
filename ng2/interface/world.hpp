#ifndef NG2_SCENE_H
#define NG2_SCENE_H

#include "../collision/object.hpp"
#include <list>

namespace ng2 {
    struct SceneState {
        const std::list<Object> *objects;
    };

    class Scene {
    public:
        Scene();

        void start();

        void stop();

        void add_object(Object obj);

        // EVENT HANDLERS
        // called when the scene starts running
        void (*hd_init)(SceneState);

        // called when rendering the graphics
        void (*hd_render)(SceneState, float);

        // called before each physics update
        void (*hd_fixed_update)(SceneState);
        //TODO
        //stopping scene
        //resetting scene
        //stepping  
    private:
        std::list<Object> objects;
        SceneState state;
        bool running;

        void physics_update();

    };
} // namespace ng2

#endif