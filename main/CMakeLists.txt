add_executable(main main.cpp shader.cpp)

list(APPEND CMAKE_PREFIX_PATH ${CMAKE_CURRENT_SOURCE_DIR}/deps/glew-2.1.0)
list(APPEND CMAKE_PREFIX_PATH ${CMAKE_CURRENT_SOURCE_DIR}/deps/glfw-3.2.1)

find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
#find_package(glfw3 3.2 REQUIRED)

find_path(GLFW_INCLUDE_DIR NAMES GLFW/glfw3.h)
find_library(GLFW_LIBRARY NAMES glfw glfw3)
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GLFW DEFAULT_MSG GLFW_LIBRARY GLFW_INCLUDE_DIR)
mark_as_advanced(GLFW_INCLUDE_DIR GLFW_LIBRARY)

include_directories(${OPENGL_INCLUDE_DIR} ${GLEW_INCLUDE_DIRS} ${GLFW_INCLUDE_DIR})

link_directories(${ng2_demo_SOURCE_DIR}/ng2/collision ${ng2_demo_SOURCE_DIR}/ng2/interface)
# ${GLFW_LIBRARY} may not be required. Remove later to test if true
target_link_libraries(main ng2 ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} ${GLFW_LIBRARY} m pthread glfw3dll)
