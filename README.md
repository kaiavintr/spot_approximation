This repository contains shader code (in [spot_approximation.glsl](spot_approximation.glsl)) for an approximation of Keenan Crane's "Spot" cow model, using SDF-like functions, along with a ray-marching demo that can be viewed in Shadertoy.

![Spot approximation raymarching demo screenshot](image.png)

I don't know if it will be useful to anyone else, but just in case, I am making it available here under an MIT license. The demo Shadertoy code (in [shadertoy_image_shader.glsl](shadertoy_image_shader.glsl)) is also MIT licensed, and includes two example ray marchers (basic sphere tracer and a version that overshoots and uses bisection). It also shows how to use the signed value and gradient returned by the "texture" function to antialias the edges of the texture shapes.

By "approximation" I really mean trying to find a good balance between efficiency and looking similar to the original model, while prioritizing aesthetics (especially smoothness). The shapes are generally simpler and more rounded than the original.

If you have any suggestions for improving this code or making it more useful, please open an issue here or leave a comment on the Shadertoy shader. You can also message me on [Bluesky](https://bsky.app/profile/kaiavintr.bsky.social) or [Mastodon](https://mastodon.social/@kaiavintr).

The original Spot model can be found here: https://www.cs.cmu.edu/~kmcrane/Projects/ModelRepository/
Keenan Crane has stated that acknowledgement is appreciated but not required.

# Implementation

The 3D implicit surface uses "smooth min" and "smooth max" functions to blend spheres, planes, and one cylinder, returning an SDF bound. The gradient is also returned (the compiler should remove the gradient computation when not needed). It is similar to typical Shadertoy-style SDF functions, and the code should be easy to understand. There is also an optimized version that skips some shapes based the 3D point where the function is being evaluated, which appears to improve performance for coherent rays.

For most of the texture shapes (e.g. the spots), the approximation also uses smooth min, blending unnormalized ellipse functions (this is done in 2D, using a different projection for each shape). For some shapes, functions similar to circular harmonics are used. The eyes and udder use simple ellipses.

Both 2D and 3D blending use quartic splines (typical "smooth min" uses quadratic functions). This provides continuous second derivatives (as does the cubic version of smooth min) with one extra degree of freedom that can be used to modify the shape of the blend region.

# Approach

There are obviously any number of ways you could approximate a shape like this using SDF primitives (or an arbitrary function). I initially tried using ellipsoids and some more unusual blending techniques, but ended up simplifying to use mainly spheres. Ellipsoids can give a shape that is a closer fit, but they are more expensive than spheres, and blending ellipsoids tends to produce weird creases that are hard to control (and might result in an invalid SDF bound function). Popular SDF shapes like capsules have discontinuities in the second derivative, which I think makes them look less organic. More spheres could be added to improve the fit (e.g. in the legs) but there are diminishing returns. I'm very happy with this version of the tail shape, using two spheres and two planes, but it is quite expensive to ray march (earlier, I tried using a single ellipsoid, which looks fine but it isn't the right shape). The ears are tricky and I'd still like to improve them.

Some of the texture shapes (horns and udder) can't really be improved unless the surface is made more accurate. The shape of some of the spots could probably be refined by blending more ellipses.  I haven't explored using Bezier curves directly, because I assumed it would be much more expensive (the original model comes with an SVG file that uses cubic Bezier curves for the spots).

I used numerical optimization to help find good parameters for the shapes (variations of hill climbing and randomized searches, along with linear regression when possible). However, it was a very manual process overall, with a lot of trial and error.

# Coordinate system

Coordinate space (including scale and offset) is the same as used in the original model. Positive y is up, and positive z is towards the tail of the cow. Positive x is the left side of the cow (making it a left-handed coordinate system) but the surface SDF function (and the original subdivision surface) are symmetrical in the x direction, so this only affects the texture function.

Bounding boxes taken from a large number of subdivisions and samples (not analytic limits):
* Original model (Catmull–Clark subdivision surface): (-0.46344, 0.46344) x (-0.72985, 0.95089) x (-0.66707, 1.04768)
* Approximation: (-0.45702, 0.45702) x (-0.72980, 0.94990) x (-0.66336, 1.04673)

# License

The source code is released under an MIT license.

The original Spot model is public domain. My intent was to produce shapes as similar as possible to the original model, but they do not match the original exactly, and I don't know if the differences are significant enough to constitute a new creative work. To avoid any ambiguity, I am releasing the surface shape and texture produced by the code into the public domain. If you use the code to produce images, videos, meshes, point clouds, etc., you don't need to give me credit for those outputs. Giving credit to Keenan Crane (with clarification that it is an approximation, not the original model) would be appropriate, but is not required.

# Image comparisons

Below are comparisons between the original model (left) and the approximation (right), both rendered in Blender. To do this, I generated a distorted copy of the original mesh (mapping each vertex to a nearby point on the approximation surface), and a texture produced by evaluating the texture function at points on the surface, and mapping back to the texture coordinates (the mesh and texture can be found in the [comparison](comparison) directory).

![Original model, front](comparison/reference_front.png) ![Approximation, front](comparison/approx_front.png)
![Original model, left](comparison/reference_left.png) ![Approximation, left](comparison/approx_left.png)
![Original model, back](comparison/reference_back.png) ![Approximation, back](comparison/approx_back.png)
![Original model, right](comparison/reference_right.png) ![Approximation, right](comparison/approx_right.png)
![Original model, top](comparison/reference_top.png) ![Approximation, top](comparison/approx_top.png)
![Original model, bottom](comparison/reference_bottom.png) ![Approximation, bottom](comparison/approx_bottom.png)

