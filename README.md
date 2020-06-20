# 2D Physics Engine for Practice

## Building
* Create directory `bin` for built executables
* Install SFML-2.5.1. If you have windows, you might need to copy some SFML `.dll` files to `bin`. See below for more info 
* `mkdir build && cd build`. Then run `cmake ..` and then `make`. When you run `cmake` for the first time, you might need to configure additional arguments


## Usage
This engine is created for practice. It is not tested and not intended for serious use. For examples,
look into `main/routines` on implmenting the routine interface.


## SFML on windows
For some reason, when using SFML on windows, you need to have the SFML dlls in the same folder as the executable. To do that,
copy all the files from `SFML/bin` and paste them into `bin` in the working directory.

## TODOs
* Implement this: https://gamedev.stackexchange.com/questions/114925/in-a-2d-physics-engine-how-do-i-avoid-useless-collision-resolutions-when-object
