This folder contains the code for the "Cow Spherization" Shadertoy shader, which can be viewed here:  
https://www.shadertoy.com/view/WXtBWf

This shader is a modification of my earlier shader here:
https://www.shadertoy.com/view/t3tfD4

This version of the shader morphs the SDF "Spot" approximation (which is otherwise nearly the same as in the
original shader) into a sphere. It is inspired by the animation in spot_to_sphere.mov on Crane's website
(URL above), but it does not try to replicate that video (the shape and texture are warped in a very different way).

The original signed distance function has been modified so it is smoother away from the surface and takes a
"spherize" parameter.

To morph the texture, the Image shader maps the 3D point back to the undistorted surface by following the gradient
    towards the closest point, using several intermediate surfaces and the undistorted surface.
This becomes quite expensive, especially since the SDF is slower than the original.

To speed things up, the shader performs ray marching and texture location mapping at quarter resolution, and uses
    this data in the Image shader to:
- quickly discard pixels where the ray definitely misses the shape
- get rough bounds on the intersection distance, to speed up full-resolution ray marching
- for many pixels, get a good enough texture location using linear interpolation of the low-res data
- for many pixels, get approximations of the texture location partial derivatives
