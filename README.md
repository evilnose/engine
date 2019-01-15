## Note to self
Need OpenGL, GLEW, and glfw3 to build main (for graphics). GLEW and glfw3 are _mostly_ self-contained in CMakeLists.txt
(just install their platform-dependent headers and binaries in `main/dep/`). For MinGW, need to paste `libglfw3.a` and
`libglfw3dll.a` to `mingw/lib`, and `glfw3.dll` and `glew32.dll` to `System32` (I know, pain in the arse). Also need
32 bit glfw3 for 32 bit mingw and 64 bit for 64 bit. (Note that there is a way to add runtime DLL's in CMake. Maybe try
that in the future.)

Don't forget to set working directory to the directory this README is in to load the shader files when running in IDE.