/*
License for source code (see below for information about output):

Copyright (c) 2026 Kaia Vintr

Permission is hereby granted, free of charge, to any person obtaining a copy of 
this software and associated documentation files (the "Software"), to deal in 
the Software without restriction, including without limitation the rights to 
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies 
of the Software, and to permit persons to whom the Software is furnished to do 
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all 
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE 
SOFTWARE.

------------------------------------------------------------------------------

The above license and copyright applies to the source code only, not to the 
output of the code. The code defines a 3D "virtual object" (mathematical 
implicit surface, with color defined at each point on the surface). The intent 
is that this virtual object should be closely similar to the "Spot" model, 
which was created by Keenan Crane and released into the public domain (defined 
by Catmull-Clark control points and an SVG file). The virtual object defined by 
the code is a modification of that original creative work and is also hereby 
released into the public domain. This virtual object (whether rendered as a 
static or moving image, or output as data such as a mesh, point cloud, or 
texture) may be incorporated into new creative works (including works with a 
different license or copyright) without restriction.

The original Spot model can be found here: 
https://www.cs.cmu.edu/~kmcrane/Projects/ModelRepository/
Keenan Crane has stated that acknowledgement is appreciated but not required.
    
Github repository for this code:
https://github.com/kaiavintr/spot_approximation

Please contact Kaia Vintr with questions regarding the code via:
- comment on the Shadertoy shader
- GitHub issue
- Bluesky: https://bsky.app/profile/kaiavintr.bsky.social
- Mastodon: https://mastodon.social/@kaiavintr
    
*/

/*
    This file contains the Spot approximation functions and related definitions.
    
    Raymarching and rendering demo code is in "shadertoy_image_shader.glsl"

*/


// Define this to enable an optimized version of the signed distance function
// May be less efficient for incoherent rays in path tracing
#define SPOT_SDF_OPTIMIZED

// The optimized version may slightly overestimate distances.
// If this is an issue, change the constant below to (e.g.) 0.99
const float SPOT_SDF_OPTIMIZED_SCALE = 1.0;

// Define this to make the spot_get_color_index function return a signed value and gradient as well as the color index
// Value and gradient can be used for antialiasing.
// For path tracing, you would probably not use this
// When defined, spot_get_eye_highlight_grad must be called to test the eye highlight shape.
#define SPOT_TEXTURE_EDGE_GRADIENT

// Coordinates of AABB of the Spot approximation shape
// (slightly larger than bound taken from a large number of samples)
// TODO: Find actual bound using gradient descent at extreme points
const vec3 SPOT_BOUNDING_BOX_LOW = vec3(-0.458, -0.73, -0.664);
const vec3 SPOT_BOUNDING_BOX_HIGH = vec3(0.458, 0.95,  1.047);


// Values used to represent colors
const int SPOT_COLOR_BASE = 0;
const int SPOT_COLOR_SPOTS = 1;
const int SPOT_COLOR_NOSE = 2;
const int SPOT_COLOR_UDDER = 3; // same color as nose
const int SPOT_COLOR_HOOVES = 4;
const int SPOT_COLOR_HORNS = 5;
const int SPOT_COLOR_EYES = 6;
const int SPOT_COLOR_EYE_HIGHLIGHT = 7;
const int SPOT_COLOR_EARS = 8;
const int SPOT_COLOR_NOSTRILS = 9;

// Map a color index to a diffuse color
// Colors are taken from the original Spot texture SVG file, converted from sRGB to linear
vec3 spot_color_index_to_rgb(int color_index) {
    
    switch (color_index) {

    case SPOT_COLOR_SPOTS:
        return vec3(0.051);
    
    case SPOT_COLOR_NOSE:
    case SPOT_COLOR_UDDER:
        return vec3(1., 0.564, 0.387);
    
    case SPOT_COLOR_HOOVES:
        return vec3(0.138);
    
    case SPOT_COLOR_HORNS:
        return vec3(0.337, 0.102, 0.036);
    
    case SPOT_COLOR_EYES:
        return vec3(0.03);
    
    case SPOT_COLOR_EYE_HIGHLIGHT:
        return vec3(1);
    
    case SPOT_COLOR_EARS:
        return vec3(0.337);
    
    case SPOT_COLOR_NOSTRILS:
        return vec3(0.234, 0.068, 0.023);
    
    //case SPOT_COLOR_BASE:
    default:
        return vec3(1., 0.855, 0.792);
    }
}

// Quartic "smooth minimum" blending function for 2D SDF blending
// Blends two vec3s which have 2D gradient in xy and signed distance in z
// "dist" is a blending distance parameter
//      (if difference between the values is greater than this, then the minimum will be used)
// "q" controls the shape of the blending. Valid range is 0.125...0.59
//      q values outside of this range will cause the blended SDF to be invalid (no longer a bound of the true SDF)
//      values > 0.23 will cause a bulge between the two shapes.
//      Value of 1/6 sets the quartic coefficient to 0 (equivalent to cubic smooth min)
// If dist and q are constants then most of the calculations should occur at compile time, and it should be relatively cheap
//      (this is not the case for the texture function evaluation though).
vec3 blend_smin(vec3 f1, vec3 f2, float dist, float q) {
    float x = f1.z;
    float y = f2.z;
    
    float s = 1. / dist;
    float s2 = s*s;

    float A = (3.*q - 0.5) * s2 * s;
    float B = (-4.*q + 0.5) * s2;

    float a = max(dist - abs(x-y), 0.);
    
    A *= a;
    
    float t = (-4.*A - 3.*B) * a*a;

    return vec3(mix(f1.xy, f2.xy, x < y ? t : 1. - t), min(x, y) + (A + B) * a*a*a);
}

// Quartic "smooth minimum" or "smooth maximum" blending function for 3D SDF blending (see above)
// Blends two vec4s which have 3D gradient in xyz and signed distance in w
// If dist and q are constants then most of the calculations should occur at compile time, and it should be relatively cheap.
vec4 blend_sminmax(vec4 f1, vec4 f2, float dist, float q, bool use_min) {
    float x = f1.w;
    float y = f2.w;
    
    float s = 1. / dist;
    float s2 = s*s;

    float A = (3.*q - 0.5) * s2 * s;
    float B = (-4.*q + 0.5) * s2;

    float a = max(dist - abs(x-y), 0.);
    
    A *= a;
    
    float t = (-4.*A - 3.*B) * a*a;
    
    if (use_min) {
        return vec4(mix(f1.xyz, f2.xyz, x < y ? t : 1. - t), min(x, y) + (A + B) * a*a*a);
    } else {
        return vec4(mix(f1.xyz, f2.xyz, x > y ? t : 1. - t), max(x, y) - (A + B) * a*a*a);
    }
}

// Quartic "smooth minimum" blending function for 3D SDF blending (see above)
vec4 blend_smin(vec4 f1, vec4 f2, float dist, float q) {
    return blend_sminmax(f1, f2, dist, q, true);
}

// Quartic "smooth maximum" blending function for 3D SDF blending (see above)
vec4 blend_smax(vec4 f1, vec4 f2, float dist, float q) {
    return blend_sminmax(f1, f2, dist, q, false);
}

// Signed distance function for sphere
vec4 sdf_sphere(vec3 P, vec3 C, float r) {
    P = P - C;
    return vec4(normalize(P), length(P)-r);
}

// Signed distance function for half-space defined by a plane (normal vector and distance)
// Everything on one side of the plane is inside the shape.
vec4 sdf_halfspace(vec3 P, vec3 V, float d) {
    return vec4(V, dot(P, V) - d);
}

// Signed distance function for a cylinder that extends infinitely in the x direction
vec4 sdf_xcylinder(vec3 P, vec2 C, float r) {
    vec2 Q = P.yz - C;
    
    return vec4(vec3(0, normalize(Q)), length(Q)-r);
}

// Convenience function for cumulative SDF blending.
// Takes an SDF value and gradient and blends it with a sphere.
vec4 blend_sphere(vec4 f, vec3 P, vec3 C, float r, float dist, float q) {
    return blend_smin(f, sdf_sphere(P, C, r), dist, q);
}

// Convenience function for blending a pair of spheres, mirrored across the x=0 plane
vec4 sphere_pair(vec3 P, vec3 C, float r, float dist_pair, float q_pair) {
    return blend_smin(sdf_sphere(P, C, r), sdf_sphere(P, C*vec3(-1,1,1), r), dist_pair, q_pair);
}

// Convenience function for cumulative SDF blending.
// Takes an SDF value and gradient and blends it with a pair of spheres, mirrored across the x=0 plane
vec4 blend_sphere_pair(vec4 f, vec3 P, vec3 C, float r, float dist_pair, float q_pair, float dist, float q) {
    return blend_smin(f, sphere_pair(P, C, r, dist_pair, q_pair), dist, q);
}

// Ellipse function that is not an SDF (or bound) but will be treated like an SDF for blending.
// Used for making blobbly 2D shapes, not for ray marching.
// Returns 2d gradient in xy, signed "distance" value in z.
vec3 ellipse_function(vec2 P, vec3 M, float r) {
    vec2 E = mat2(M.xy, M.yz) * P;
    
    float d = sqrt(dot(P, E));
    
    return vec3(1./d * E, d - r);
}


#ifndef SPOT_SDF_OPTIMIZED // use straightforward version of signed distance function

// "SDF" function (bound) for the Spot approximation.
// Returns gradient in xyz and distance (lower bound) in w
// IMPORTANT: Since this is not a true SDF, you need to normalize the gradient before using it as a normal vector.
vec4 spot_sdf_and_gradient(vec3 P) {
    float x_sign = P.x < 0. ? -1. : 1.;
    
    vec4 f = sphere_pair(P, vec3(0.09054, -0.06191, 0.7222), 0.1924, 0.2536, 0.2769); // body rear
    f = blend_sphere_pair(f, P, vec3(0.2373, -0.3205, 0.3429), -0.00176, 0.1056, 0.4102, 0.6691, 0.2359); // body mid lower
    f = blend_sphere_pair(f, P, vec3(-0.05041, 0.1178, 0.1419), 0.167, 0.05, 0.125, 0.7952, 0.1992); // body mid upper
    f = blend_sphere(f, P, vec3(0, 0.00405, 0.04695), 0.1007, 0.1422, 0.59); // chest
    f = blend_sphere(f, P, vec3(0, 0.4546, -0.2041), 0.2553, 0.1301, 0.23); // head mid
    f = blend_sphere_pair(f, P, vec3(0.09402, 0.651, -0.408), 0.1293, 0.1836, 0.23, 0.15, 0.22); // head top front
    f = blend_sphere(f, P, vec3(0, 0.7061, -0.1507), 0.01744, 0.2003, 0.2578); // head top back
    f = blend_smax(f, sdf_halfspace(P, vec3(0, 0.1445, -0.9895), 0.5541), 0.1367, 0.2176); // plane that flattens the face
    f = blend_sphere_pair(f, P, vec3(0.1866, 0.274, -0.5157), 0.08283, 0.4182, 0.1252, 0.7553, 0.1632); // head front
    f = blend_sphere(f, P, vec3(0, 0.2919, -0.49), 0.08861, 0.2059, 0.2794); // nose middle
    f = blend_sphere(f, P, vec3(0, -0.2878, 0.4162), 0.221, 0.05, 0.125); // udder
    f = blend_sphere_pair(f, P, vec3(0.2352, -0.454, 0.8239), 0.01743, 0.1009, 0.18, 0.8526, 0.1563); // back leg top
    f = blend_sphere(f, P, vec3(0.2 * x_sign, -0.6063, 0.791), 0.1235, 0.1626, 0.22); // back leg bottom
    f = blend_sphere_pair(f, P, vec3(0.212, -0.221, 0.004702), 0.003749, 0.3448, 0.29, 0.6583, 0.2142); // front leg top
    f = blend_sphere_pair(f, P, vec3(0.2792, -0.4758, -0.009692), 0.07521, 0.2497, 0.29, 0.3503, 0.22); // front leg mid
    f = blend_sphere(f, P, vec3(0.2354 * x_sign, -0.5914, 0.03336), 0.1362, 0.1222, 0.22); // front leg bottom
    f = blend_sphere_pair(f, P, vec3(0.1695, 0.8511, -0.2575), -0.02274, 0.3244, 0.29, 0.2488, 0.2932); // horn bottom
    f = blend_sphere(f, P, vec3(0.1819 * x_sign, 0.9089, -0.2723), 0.03964, 0.1145, 0.23); // horn top

    // tail
    {
        vec4 f2 = sdf_sphere(P, vec3(0, 0.05389, 0.8271), 0.2886);
        f2 = blend_sphere(f2, P, vec3(0, -0.06683, 1.023), 0.02922, 0.2081, 0.1647);
        f2 = blend_smax(f2, sdf_halfspace(P, vec3(0, -0.8901, -0.4557), -0.3423), 0.03333, 0.23);
        f2 = blend_smax(f2, sdf_halfspace(P, vec3(0, 0.9783, 0.2073), 0.2804), 0.4062, 0.2252);
        f = blend_smin(f, f2, 0.03, 0.1662);
    }


    // ears
    {
        vec4 f2 = sdf_xcylinder(P, vec2(0.6402, -0.246), 0.1085);
        f2 = blend_smax(f2, sdf_sphere(P, vec3(0.242 * x_sign, 0.8241, -0.133), 0.2649), 0.09, 0.2078);
        f = blend_smin(f, f2, 0.1105, 0.179);
    }
    
    // add some roundness to the ears
    f = blend_sphere(f, P, vec3(0.336 * x_sign, 0.6661, -0.2359), 0.01172, 0.134, 0.2554);
    
    return f;
}

#else // use optimized version of Spot signed distance function

// "SDF" function (bound) for the Spot approximation.
// Returns gradient in xyz and distance (lower bound) in w
// IMPORTANT: You need to normalize the gradient before using it as a normal vector.
vec4 spot_sdf_and_gradient(vec3 P) {
    float x_sign = P.x < 0. ? -1. : 1.;
    
    vec4 f = sphere_pair(P, vec3(0.09054, -0.06191, 0.7222), 0.1924, 0.2536, 0.2769); // body rear
    
    f = blend_sphere_pair(f, P, vec3(0.2373, -0.3205, 0.3429), -0.00176, 0.1056, 0.4102, 0.6691, 0.2359); // body mid lower
    
    f = blend_sphere_pair(f, P, vec3(-0.05041, 0.1178, 0.1419), 0.167, 0.05, 0.125, 0.7952, 0.1992); // body mid upper
    
    if (P.y > -0.63 && P.y < 0.32 && P.z < 0.02) {
        f = blend_sphere(f, P, vec3(0, 0.00405, 0.04695), 0.1007, 0.1422, 0.59); // chest
    }
    
    if (P.y > -0.34 && P.z < 0.59) {
        f = blend_sphere(f, P, vec3(0, 0.4546, -0.2041), 0.2553, 0.1301, 0.23); // head mid
    }
    
    if (P.y > 0.13 && P.z < 0.05) {
        f = blend_sphere_pair(f, P, vec3(0.09402, 0.651, -0.408), 0.1293, 0.1836, 0.23, 0.15, 0.22); // head top front
    }
    
    if (P.y > 0.57 && P.z < 0.6 && P.z > -0.37) {
        f = blend_sphere(f, P, vec3(0, 0.7061, -0.1507), 0.01744, 0.2003, 0.2578); // head top back
    }
    
    if (abs(P.x) < 0.41 && P.y > 0.20 && P.z < -0.33) {
        // plane that flattens the face
        f = blend_smax(f, sdf_halfspace(P, vec3(0, 0.1445, -0.9895), 0.5541), 0.1367, 0.2176);
    }
    
    if (P.z < 0.6) {
        f = blend_sphere_pair(f, P, vec3(0.1866, 0.274, -0.5157), 0.08283, 0.4182, 0.1252, 0.7553, 0.1632); // head front
    }
    
    if (abs(P.x) < 0.28 && P.y > -0.31 && P.y < 0.61 && P.z < -0.24) {
        f = blend_sphere(f, P, vec3(0, 0.2919, -0.49), 0.08861, 0.2059, 0.2794); // nose middle
    }
    
    if (abs(P.x) < 0.33 && P.y < -0.38 && P.z > -0.1 && P.z < 0.84) {
        f = blend_sphere(f, P, vec3(0, -0.2878, 0.4162), 0.221, 0.05, 0.125); // udder
    }
    
    if (P.y < 0.8 && P.z > -0.16) {
        f = blend_sphere_pair(f, P, vec3(0.2352, -0.454, 0.8239), 0.01743, 0.1009, 0.18, 0.8526, 0.1563); // back leg top
    }
    
    if (P.y < -0.35 && P.z > 0.35) {
        f = blend_sphere(f, P, vec3(0.2 * x_sign, -0.6063, 0.791), 0.1235, 0.1626, 0.22); // back leg bottom
    }
    
    if (P.y < 0.54 && P.z < 0.75) {
        f = blend_sphere_pair(f, P, vec3(0.212, -0.221, 0.004702), 0.003749, 0.3448, 0.29, 0.6583, 0.2142); // front leg top
    }
    
    if (P.y < -0.05 && P.z < 0.48) {
        f = blend_sphere_pair(f, P, vec3(0.2792, -0.4758, -0.009692), 0.07521, 0.2497, 0.29, 0.3503, 0.22); // front leg mid
    }
    
    if (P.y < -0.38 && P.z < 0.43) {
        f = blend_sphere(f, P, vec3(0.2354 * x_sign, -0.5914, 0.03336), 0.1362, 0.1222, 0.22); // front leg bottom
    }
    
    if (P.y > 0.57 && P.z < 0.55) {
        f = blend_sphere_pair(f, P, vec3(0.1695, 0.8511, -0.2575), -0.02274, 0.3244, 0.29, 0.2488, 0.2932); // horn bottom
    }
    
    if (P.y > 0.71 && P.z < 0.25) {
        f = blend_sphere(f, P, vec3(0.1819 * x_sign, 0.9089, -0.2723), 0.03964, 0.1145, 0.23); // horn top
    }

    // tail
    if (P.y > -0.28 && P.y < 0.55 && P.z > 0.85) {
        vec4 f2 = sdf_sphere(P, vec3(0, 0.05389, 0.8271), 0.2886);
        f2 = blend_sphere(f2, P, vec3(0, -0.06683, 1.023), 0.02922, 0.2081, 0.1647);
        f2 = blend_smax(f2, sdf_halfspace(P, vec3(0, -0.8901, -0.4557), -0.3423), 0.03333, 0.23);
        f2 = blend_smax(f2, sdf_halfspace(P, vec3(0, 0.9783, 0.2073), 0.2804), 0.4062, 0.2252);
        f = blend_smin(f, f2, 0.03, 0.1662);
    }

    // ears
    if (P.y > 0.37 && P.z < 0.51) {
        vec4 f2 = sdf_xcylinder(P, vec2(0.6402, -0.246), 0.1085);
        f2 = blend_smax(f2, sdf_sphere(P, vec3(0.242 * x_sign, 0.8241, -0.133), 0.2649), 0.09, 0.2078);
        f = blend_smin(f, f2, 0.1105, 0.179);
    
        if (abs(P.x) > 0.2 && P.z < 0.1 && P.z > -0.46) {
            // add some roundness to the ears
            f = blend_sphere(f, P, vec3(0.336 * x_sign, 0.6661, -0.2359), 0.01172, 0.134, 0.2554);
        }
    }
    
    return SPOT_SDF_OPTIMIZED_SCALE * f;
}
#endif // end of optimized version of Spot SDF


// "texture" function for getting the color index at a particular point on the surface
// If SPOT_TEXTURE_EDGE_GRADIENT is defined, this returns a signed distance and a gradient, and the returned index
//      is the color for the interior of the shape
// Otherwise, it returns the color index for the point

int spot_get_color_index(vec3 P
#ifdef SPOT_TEXTURE_EDGE_GRADIENT
        , out vec4 f_grad
#endif
        ) {
        
#ifndef SPOT_TEXTURE_EDGE_GRADIENT
    vec4 f_grad;
#endif

    f_grad = vec4(1, 1, 1, -1e20);

    bool need_test_spots = false;
    
    if (P.y < -0.42) { // udder and hooves
        if (P.y > -0.49) { // udder
            // Using a simple ellipse here, because using the correct projected shape would require a
            //    closer fit for the 3D surface (and it isn't very visible)
            vec2 q = P.xz - vec2(0, 0.41);
            
            f_grad.w = dot(q,q*vec2(24,18)) - 1.;
            
#ifdef SPOT_TEXTURE_EDGE_GRADIENT
            f_grad.xyz = 2.*24.*q.x*vec3(1,0,0) - 2.*18.*q.y*vec3(0,0,1);
            return SPOT_COLOR_UDDER;
#else
            return f_grad.w < 0. ? SPOT_COLOR_UDDER : SPOT_COLOR_BASE;
#endif
            
        } else if (P.z < 0.2 || P.z > 0.625) { // hooves
            if (P.y > -0.61) {
                // Y coordinate of the edge is a function of angle
                // Equivalent to using circular harmonics, but expressed as polynomial in sin(angle) and cos(angle)

                vec4 param1, param2;
                vec3 param3;
                vec2 offset;

                if (P.z < 0.2) {
                    offset = vec2(0.241, 0.031);
                    
                    if (P.x > 0.) { // front left
                        param1 = vec4(-0.5329, 0.00237, 0.00374, -0.0132);
                        param2 = vec4(-0.0189, 0.0319, 0.000537, 0.00892);
                        param3 = vec3(0.0145, -0.0247, 0.00303);
                    } else { // front right
                        param1 = vec4(-0.5489, 0.0064, -0.00334, 0.00312);
                        param2 = vec4(-0.0234, -0.00934, 0.0256, 0.00612);
                        param3 = vec3(0.0252, -0.00074, -0.0294);
                    }
                } else {
                    offset = vec2(0.2014, 0.7855);
      
                    if (P.x > 0.) { // back left
                        param1 = vec4(-0.5519, -0.005, -0.00348, 0.000603);
                        param2 = vec4(0.0129, -0.00268, 0.00723, 0.0105);
                        param3 = vec3(0.00688, -0.00489, -0.0136);
                    } else { // back right
                        param1 = vec4(-0.5499, -0.0236, -0.0044, 0.0104);
                        param2 = vec4(-0.0051,  0.0343, 0.00274, -0.0112);
                        param3 = vec3(0.0151, -0.0178, 0.00958);
                    }
                }
                
                vec2 q = vec2(abs(P.x) - offset.x, offset.y - P.z);
                
                float s = 1. / length(q);
                
                float x = s*q.x, y = s*q.y;
                
                float A = param1.z + (param2.x + (param2.z + (param3.x + param3.z*x)*x)*x)*x; // factor out common expression
                
                f_grad.w = (P.y - param1.x - A*y - (param1.y + (param1.w + (param2.y + (param2.w + param3.y*x)*x)*x)*x)*x);
                
                vec3 basis1 = vec3(P.x < 0. ? -1. : 1., 0., 0.);
                const vec3 basis2 = vec3(0, 0, 1);
                
                vec3 grad_d = s * (q.x*basis1 + q.y*basis2);
                vec3 grad_s = -s*s * grad_d;
                
                vec3 grad_x = q.x*grad_s + s*basis1;
                vec3 grad_y = q.y*grad_s + s*basis2;

#ifdef SPOT_TEXTURE_EDGE_GRADIENT
                f_grad.xyz = (vec3(0,1,0) - A*grad_y
                      - (y*(param2.x + (2.*param2.z + (3.*param3.x + 4.*param3.z*x)*x)*x)
                         + param1.y + (2.*param1.w + (3.*param2.y + (4.*param2.w + 5.*param3.y*x)*x)*x)*x)*grad_x);
                
                return SPOT_COLOR_HOOVES;
#else
                return f_grad.w < 0. ? SPOT_COLOR_HOOVES : SPOT_COLOR_BASE;
#endif
            } else {
                return SPOT_COLOR_HOOVES;
            }
        } else {
            return SPOT_COLOR_UDDER;
        }
    } else if (P.z > -0.175) {
        need_test_spots = true;
    } else if (P.y > 0.09) { // head area
        float k = P.y + P.z;
            
        if (k < -0.055) { // nose area
            if (k < -0.2) { // safe nose area
                if (P.z < -0.62) { // test if point is inside nostril shape
                    // Use the same SDF-like code as is used for the spots, to reduce code size
                    //      (even though it's overkill for these shapes)
                    need_test_spots = true;
                } else {
                    return SPOT_COLOR_NOSE;
                }
            } else { // test edge of nose
                // Rotate coordinate to new basis, then distance along one of the axes is a
                //      function of angle around that axis
                // Equivalent to using circular harmonics, but expressed as polynomial in sin(angle) and cos(angle)

                const vec3 basis1 = vec3(1, 0, 0);
                const vec3 basis2 = vec3(0, -0.7805, 0.6251);
                const vec3 basis3 = vec3(0, 0.6251, 0.7805);
            
                vec2 q0 = vec2(dot(P, basis1), dot(P, basis2) + 0.5194);
                
                float d = length(q0);
                float s = 1. / d;
                
                float y = s * q0.y;
                
                f_grad.w = 0.1213 + dot(P, basis3) + y*(-0.0372 + y*(0.09249 
                        + y*(0.1033 + y*(-0.007562 + y*(-0.1298 + y*(-0.217 + y*(0.07828 + y* 0.1697)))))));
                
                vec3 grad_d = s * (q0.x*basis1 + q0.y*basis2);
                vec3 grad_s = -s*s * grad_d;
                vec3 grad_y = q0.y*grad_s + s*basis2;
                
#ifdef SPOT_TEXTURE_EDGE_GRADIENT
                f_grad.xyz = (basis3 + (-0.0372 + (2.*0.09249 + (3.*0.1033 + (-4.*0.007562 
                        + (-5.*0.1298 + (-6.*0.217 + (7.*0.07828 + 8.*0.1697*y)*y)*y)*y)*y)*y)*y)*grad_y);
                
                return SPOT_COLOR_NOSE;
#else
                return f_grad.w < 0. ? SPOT_COLOR_NOSE : SPOT_COLOR_BASE;
#endif
            }
        } else {
            if (k < 0.27) {
                // Eyes and eye highlights are ellipses projected onto surface
                // Using arbitrary direction vectors for the major and minor axes of the ellipses.
                // (i.e. the shape is the intersection of an arbitrary elliptical cylinder and the surface)
                vec3 coeff1, coeff2;
                vec2 offset;
                
                if (P.x > 0.) {
                    coeff1 = vec3(0.506, -9.169, -3.82);
                    coeff2 = vec3(22.4, 0.82, 1.003);
                    offset = vec2(3.712, -2.104);
                } else {
                    coeff1 = vec3(-0.662, -9.22, -3.813);
                    coeff2 = vec3(22.8, -0.939, -1.684);
                    offset = vec2(3.7407, 2.217);
                }
                
                vec2 q = vec2(dot(P, coeff1), dot(P, coeff2)) + offset;
                
                f_grad.w = dot(q, q) - 1.;

#ifdef SPOT_TEXTURE_EDGE_GRADIENT
                f_grad.xyz = 2.*(q.x*coeff1 + q.y*coeff2);

                // (eye highlight is tested by a separate function)
                
                return SPOT_COLOR_EYES;
#else
                
                if (f_grad.w > 0.) {
                    return SPOT_COLOR_BASE;
                } else {
                    // Test if point is inside eye highlight (another ellipse)
                    
                    if (P.x > 0.) {
                        coeff1 = vec3(1.922, -27.251, -10.984);
                        coeff2 = vec3(64.291, 2.989, 3.835);
                        offset = vec2(12.3613, -6.992);
                    } else {
                        coeff1 = vec3(-1.73, -27.289, -10.995);
                        coeff2 = vec3(63.893, -3.172, -2.179);
                        offset = vec2(12.3966,  6.8743);
                    }

                    q = vec2(dot(P, coeff1), dot(P, coeff2)) + offset;
                    
                    return dot(q, q) < 1. ? SPOT_COLOR_EYE_HIGHLIGHT : SPOT_COLOR_EYES;
                }
#endif
                
            } else if (k < 0.431) { // shapes for underside of the ears
                // Use the same SDF-like code as is used for the spots
                need_test_spots = true;
            } else { // horns
                vec3 bound_low = vec3(0.07, 0.76, -0.34);
                vec3 bound_high = vec3(0.23, 1.27, -0.21);
                
                if (abs(P.x) > bound_low.x && abs(P.x) < bound_high.x && P.y > bound_low.y && P.y < bound_high.y && P.z > bound_low.z && P.z < bound_high.z) {
                    // Rotate coordinate to new basis, then distance along one of the axes is a
                    //      function of angle around that axis
                    // Equivalent to using circular harmonics, but expressed as polynomial in sin(angle) and cos(angle)
                
                    vec4 param1;
                    vec3 param2;
                    
                    if (P.x > 0.) { // left
                        param1 = vec4(0.81487859, -0.00204274, -0.0014182, 0.00378644);
                        param2 = vec3(-0.00747133, 0.00363717, 0.00332907);
                    } else { // right
                        param1 = vec4(8.14601653e-01, -3.32745045e-03, -2.74123036e-04, 4.47013305e-03);
                        param2 = vec3(-8.06562515e-03, 3.30476957e-03, 3.64574427e-03);
                    }
     
                    vec3 basis1 = vec3(P.x < 0. ? 0.01294 : -0.01294, -0.03464, -0.9993);
                    vec3 basis2 = vec3(P.x < 0. ? -0.9368 : 0.9368, -0.3499, 0.);
                    vec3 basis3 = vec3(P.x < 0. ? -0.3496 : 0.3496, 0.9362, -0.03697);
                    
                    vec3 q0 = vec3(dot(basis1, P) - 0.244, dot(basis2, P) + 0.14715, dot(basis3, P));

                    float s = 1. / length(q0.xy);

                    float x = s*q0.x, y = s*q0.y;
                    
                    float A = param1.z + (param2.x + param2.z*x)*x; // factor out shared expression

                    f_grad.w = param1.x + (param1.y + (param1.w + param2.y*x)*x)*x + y*A - q0.z;
                    
                    // Removing brackets around s*s*s causes shader compilation time to jump by 8 or 9 seconds
                    //      in WebGL on Windows (presumably due to a quirk in the fxc compiler)
                    // Thanks to @Nguyen2007 on Shadertoy for suggesting I try the brackets!
                    vec3 grad_s = -(s*s*s)*(q0.x*basis1 + q0.y*basis2);
                    
                    vec3 grad_x = q0.x*grad_s + s*basis1;
                    vec3 grad_y = q0.y*grad_s + s*basis2;

#ifdef SPOT_TEXTURE_EDGE_GRADIENT
                    f_grad.xyz = ((param1.y + (2.*param1.w + 3.*param2.y*x)*x) * grad_x
                                + grad_y*A
                                + grad_x*y*(param2.x + 2.*param2.z*x)
                                - basis3);
                    
                    return SPOT_COLOR_HORNS;
#else
                    return f_grad.w < 0. ? SPOT_COLOR_HORNS : SPOT_COLOR_BASE;
#endif
                }
            }
        }
    }
    
    if ( ! need_test_spots) {
        return SPOT_COLOR_BASE;
    } else {
        int spotnum;
        
        if (P.z < -0.175) { // ears or nostrils
            if (P.z < -0.5) { // nostrils
                if (P.x > 0.) {
                    spotnum = 11; // left side 
                } else {
                    spotnum = 12; // right side
                }
            } else { // shapes for underside of ears
                if (P.x > 0.) {
                    spotnum = 9; // left side 
                } else {
                    spotnum = 10; // right side
                }
            }
        } else if (P.y + 0.5*P.x > 0.51) { // spot on back of head (spot10)
            spotnum = 13;
        } else if (P.x > 0.04) { // left side
            if (P.z < 0.42) { // large spot on left side at front (spot1)
                spotnum = 14;
            } else { // four smaller spots
                if (P.y < -0.01) { // lower spots
                    if (P.z < 0.75) { // small spot on left side, lower down (spot9)
                        spotnum = 1;
                    } else { // medium-sized spot above left leg, at back (spot8)
                        spotnum = 2;
                    }
                } else { // upper spots
                    if (P.z < 0.625) { // small spot on top of back, left side (spot7)
                        spotnum = 3;
                    } else { // medium-sized spot on rump, left side (spot2)
                        spotnum = 4;
                    }
                }
            }
        } else { // right side
            if (P.z < 0.15) {
                if (P.y > 0.22) { // spot on back of head, lower down (spot11)
                    spotnum = 5;
                } else { // small spot on right side at front (spot4)
                    spotnum = 6;
                }
            } else {
                if (P.z < 0.72) {
                    // (The P.y > 0.225 cutoff is a bit iffy - may break if the surface gets distorted slightly)
                    if (P.y > 0.225) { // small spot on top near front, right side (spot5)
                        spotnum = 7;
                    } else { // large spot on right side (spot3)
                        spotnum = 15;
                    }
                } else { // medium sized spot on rump, right side (spot6)
                    spotnum = 8;
                }
            }
        }
        
        vec3 planeN; // normal for plane that we're projecting onto
        vec4 coeff1, coeff2; // coefficients for two ellipses
        vec2 center1, center2, mix1; // centers for two ellipses, and blending parameters
        
        switch (spotnum) {
        case 1: // small spot on left side, lower down (spot9)
            planeN = vec3(0.98218, 0.18135, 0.04944);
            center1 = vec2(-0.5091, -0.1256);
            coeff1 = vec4(0.1366, -0.0554, 0.2765, 0);
            center2 = vec2(-0.6079, -0.1809);
            coeff2 = vec4(0.1101, 0.0782, 0.1274, 0.000392);
            mix1 = vec2(0.1385, 0.23);
            break;
        case 2: // medium-sized spot above left leg, at back (spot8)
            planeN = vec3(0.49251, -0.02653, 0.8699);
            center1 = vec2(-0.2795, -0.2912);
            coeff1 = vec4(0.1927, 0.0549, 0.2137, 0.0132);
            center2 = vec2(-0.2137, -0.1526);
            coeff2 = vec4(0.0166, 0.0376, 0.135, 0);
            mix1 = vec2(0.1597, 0.23);
            break;
        case 3: // small spot on top of back, left side (spot7)
            planeN = vec3(0.34075, 0.92455, 0.1706);
            center1 = vec2(-0.4377, -0.021);
            coeff1 = vec4(0.1177, 0.0134, 0.0717, 0.00951);
            center2 = vec2(-0.5001, -0.0418);
            coeff2 = vec4(0.1044, 0.00905, 0.0619, 0.00893);
            mix1 = vec2(0.031, 0.1399);
            break;
        case 4: // medium-sized spot on rump, left side (spot2)
            planeN = vec3(0.58299, 0.67175, 0.45702);
            center1 = vec2(-0.6881, -0.0719);
            coeff1 = vec4(0.1068, -0.00287, 0.0939, 0.0304);
            center2 = vec2(-0.5679, -0.0973);
            coeff2 = vec4(0.177, 0.0107, 0.1122, 0.0362);
            mix1 = vec2(0.0272, 0.23);
            break;
        case 5: // spot on back of head, lower down (spot11)
            planeN = vec3(-0.34182, -0.02999, 0.93929);
            center1 = vec2(-0.0333, -0.4003);
            coeff1 = vec4(0.1088, -0.00894, 0.0596, 0);
            center2 = vec2(0.1086, -0.431);
            coeff2 = vec4(0.0636, -0.0315, 0.0875, 0.00565);
            mix1 = vec2(0.1062, 0.23);
            break;
        case 6: // small spot on right side at front (spot4)
            planeN = vec3(-0.79750, 0.34981, -0.49156);
            center1 = vec2(-0.1655, 0.1492);
            coeff1 = vec4(0.1504, -0.0393, 0.1478, 0.00295);
            center2 = vec2(-0.0621, 0.0698);
            coeff2 = vec4(0.0846, -0.0602, 0.0765, 0.00239);
            mix1 = vec2(0.1216, 0.23);
            break;
        case 7: // small spot on top near front, right side (spot5)
            planeN = vec3(-0.18690, 0.91381, 0.36059);
            center1 = vec2(-0.2437, 0.0116);
            coeff1 = vec4(0.1528, -0.0222, 0.1654, 0.0117);
            center2 = vec2(-0.3009, 0.0382);
            coeff2 = vec4(0.1839, 0.0582, 0.1253, 0.00967);
            mix1 = vec2(0.0174, 0.23);
            break;
        case 8: // medium sized spot on rump, right side (spot6)
            planeN = vec3(-0.48805, 0.7036, 0.51649);
            center1 = vec2(-0.5997, 0.1152);
            coeff1 = vec4(0.0874, 0.0317, 0.1675, 0.0174);
            center2 = vec2(-0.6598, 0.0174);
            coeff2 = vec4(0.0295, 0.0411, 0.1506, 0);
            mix1 = vec2(0.0783, 0.23);
            break;
        case 9: // left ear
            planeN = vec3(0.40919, -0.76636, -0.49523);
            center1 = vec2(0.3761, 0.5658);
            coeff1 = vec4(0.0511, -0.0299, 0.0297, 0);
            center2 = vec2(0.4791, 0.5732);
            coeff2 = vec4(0.0414, 0.0223, 0.0595, 0);
            mix1 = vec2(0.0574, 0.28);
            break;
        case 10: // right ear
            planeN = vec3(-0.40919, -0.76636, -0.49523);
            center1 = vec2(0.4539, -0.6041);
            coeff1 = vec4(0.0624, -0.0443, 0.0659, 0.00144);
            center2 = vec2(0.372, -0.5548);
            coeff2 = vec4(0.1054, 0.0375, 0.1181, 0);
            mix1 = vec2(0.0815, 0.2735);
            break;
        case 11: // left nostril
            planeN = vec3(0.12487, 0.19492, -0.97284);
            center1 = vec2(-0.1959, 0.0567);
            coeff1 = vec4(0.0891, -0.00184, 0.0739, 0.00699);
            center2 = vec2(-0.2156, 0.0443);
            coeff2 = vec4(0.0574, 0.00654, 0.0429, 0.00594);
            mix1 = vec2(0.0145, 0.2265);
            break;
        case 12: // right nostril
            planeN = vec3(0.12487, -0.19492, 0.97284);
            center1 = vec2(-0.2199, 0.0413);
            coeff1 = vec4(0.0908, 0.0152, 0.0779, 0.00808);
            center2 = vec2(-0.2005, 0.0534);
            coeff2 = vec4(0.1348, 0.00514, 0.1301, 0.00916);
            mix1 = vec2(0.0169, 0.2296);
            break;
        case 13: // spot on back of head (spot10)
            planeN = vec3(0.13503, 0.54333, 0.82859);
            center1 = vec2(0.5555, 0.2104);
            coeff1 = vec4(0.00243, 0.000921, 0.00377, 0);
            center2 = vec2(0.4877, 0.0684);
            coeff2 = vec4(0.1182, 0.0541, 0.1076, 0.0126);
            mix1 = vec2(0.01, 0.1618);
            break;
        case 14: // large spot on left side at front (spot1)
            planeN = vec3(0.88268, 0.42832, -0.19346);
            center1 = vec2(-0.1894, 0.1312);
            coeff1 = vec4(0.3596, -0.0157, 0.4675, 0.0626);
            center2 = vec2(-0.0924, -0.1426);
            coeff2 = vec4(0.1051, -0.00376, 0.1909, 0.0394);
            mix1 = vec2(0.0511, 0.2459);
            break;
        case 15: // large spot on right side (spot3)
            planeN = vec3(-0.92778, 0.36424, 0.08099);
            center1 = vec2(-0.322, 0.2731);
            coeff1 = vec4(0.5084, 0.0655, 0.567, 0.0364);
            center2 = vec2(-0.3653, -0.0556);
            coeff2 = vec4(0.2083, 0.0297, 0.1228, 0.0205);
            mix1 = vec2(0.0665, 0.3003);
            break;
        }
            
        /*
            Construct a tangent basis from the given normal vector and project P into that basis.
            Simplification of this (assuming planeN is normalized):
                T = -normalize(vec3(0,0,1) - planeN.z*planeN); // orthonormalization of (0,0,-1) and planeN
                B = cross(planeN, T);
                
            putting T and B in a matrix for convenience
        */
        mat3x2 M = transpose(mat2x3(planeN.z*planeN - vec3(0,0,1), vec3(-planeN.y, planeN.x, 0))) / sqrt(1. - planeN.z*planeN.z);
        
        vec2 q = M * P;
    
        vec3 f_pd = blend_smin(ellipse_function(q - center1, coeff1.xyz, coeff1.w),
                               ellipse_function(q - center2, coeff2.xyz, coeff2.w), mix1.x, mix1.y);
        
        // Blend with more ellipses if needed
        if (spotnum >= 13) {
            if (spotnum == 13) { // spot on back of head (spot10)
                center1 = vec2(0.4555, 0.071);
                coeff1 = vec4(0.036, -0.00225, 0.0288, 0.00132);
                mix1 = vec2(0.0657, 0.1502);
            } else if (spotnum == 14) { // large spot on left side at front (spot1)
                center1 = vec2(-0.2102, -0.00911);
                coeff1 = vec4(0.0108, 0.0135, 0.0224, 0);
                mix1 = vec2(0.0866, 0.2428);
            } else if (spotnum == 15) { // large spot on right side (spot3)
                center1 = vec2(-0.2824, 0.0226);
                coeff1 = vec4(0.1228, 0.072, 0.1511, 0.0216);
                mix1 = vec2(0.0583, 0.2272);
            }
                    
            vec3 f_pd2 = ellipse_function(q - center1, coeff1.xyz, coeff1.w);

            if (spotnum >= 14) {
                vec2 mix2;
                
                if (spotnum == 14) { // large spot on left side at front (spot1)
                    center1 = vec2(-0.3441, -0.2393);
                    coeff1 = vec4(0.0392, 0.0036, 0.0383, 0.0153);
                    mix2 = vec2(0.0315, 0.125);
                } else if (spotnum == 15) { // large spot on right side (spot3)
                    center1 = vec2(-0.4797, 0.1786);
                    coeff1 = vec4(0.1872, 0.0276, 0.5069, 0.0762);
                    mix2 = vec2(0.0651, 0.2821);
                }
                                        
                vec3 f = ellipse_function(q - center1, coeff1.xyz, coeff1.w);
                
                f_pd2 = blend_smin(f_pd2, f, mix2.x, mix2.y);
            }
            
            f_pd = blend_smin(f_pd, f_pd2, mix1.x, mix1.y);
        }

#ifdef SPOT_TEXTURE_EDGE_GRADIENT
        f_grad = vec4(f_pd.xy * M, f_pd.z);
#else
        if (f_pd.z > 0.) { // point is not inside shape
            return spotnum == 11 || spotnum == 12 ? SPOT_COLOR_NOSE : SPOT_COLOR_BASE;
        }
#endif

        return spotnum == 9 || spotnum == 10 ? SPOT_COLOR_EARS : spotnum == 11 || spotnum == 12 ? SPOT_COLOR_NOSTRILS : SPOT_COLOR_SPOTS;
    }
}

#ifdef SPOT_TEXTURE_EDGE_GRADIENT

// Separate function for testing high highlight shape, so it can be antialiased when edge is close to eye edge
vec4 spot_get_eye_highlight_grad(vec3 P) {
    vec3 coeff1, coeff2;
    vec2 offset;
    
    if (P.x > 0.) {
        coeff1 = vec3(1.922, -27.251, -10.984);
        coeff2 = vec3(64.291, 2.989, 3.835);
        offset = vec2(12.3613, -6.992);
    } else {
        coeff1 = vec3(-1.73, -27.289, -10.995);
        coeff2 = vec3(63.893, -3.172, -2.179);
        offset = vec2(12.3966,  6.8743);
    }

    vec2 q = vec2(dot(P, coeff1), dot(P, coeff2)) + offset;
    
    return vec4(2.*(q.x*coeff1 + q.y*coeff2), dot(q, q) - 1.);
}

#endif
