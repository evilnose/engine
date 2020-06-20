set(SFMLRoot "C:/Users/Gary/Opt/SFML")
list(APPEND CMAKE_PREFIX_PATH "${SFMLRoot}")
find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)

# Have to manually set include path for MinGW-64 for some reason. Welp.
include_directories("${SFMLRoot}/include")
add_executable(main routines/basicRoutine.cpp main.cpp)
link_directories(${ng2_demo_SOURCE_DIR}/ng2/collision ${ng2_demo_SOURCE_DIR}/ng2/collider ${ng2_demo_SOURCE_DIR}/ng2/interface)
target_link_libraries(main ng2 sfml-graphics sfml-window sfml-system  ${SFML_LIBRARIES} m pthread)

