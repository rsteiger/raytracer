Richie's Raytracer
=================

This project is a simple raytracer that draws spheres. A ray is cast for each
pixel and is checked for intersection with each sphere (accelerated with a
Bounding Volume Hierarchy.) At the intersection, shadow feeler rays are cast to
each light. The color of the light calculated for the ray gets added to an
opengl framebuffer.


Compilation
----------

The CUDA toolkit is required for this project. Compile with 

    $> make

I've only tested this on my MacBook Air, but I imagine it will work on other
Macs at the very least.

Running
------

Run with

    $> ./a.out

a window will open that you can navigate with the mouse and arrow keys.


TODO
---

* A more intuitive interface
* Render more primitives
* Test other platforms
* Make options more easily configurable
* Better documentation
* Move Tests out of BVH.h

