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
    See "spot_approximation.glsl" for the shape approximation functions and related definitions.
    
    This file contains the raymarching and rendering demo code only.

*/


// Uncomment one of these lines:
//#define ray_march ray_march_simple
#define ray_march ray_march_bisect

// ray_march_simple may need many sphere tracing steps to remove visible artifacts near edges
const int RAY_MARCH_SIMPLE_MAX_ITERATIONS = 500;

// ray_march_bisect uses fewer iterations, but may step past some surface intersections near edges
//   causing apparent shape to shrink slightly (depending on value of RAY_MARCH_BISECT_MIN_STEP).
const int RAY_MARCH_BISECT_MAX_ITERATIONS = 100;

// For simple ray marcher: stop ray marching if absolute value is less than this
const float RAY_MARCH_SIMPLE_SDF_THRESHOLD = 0.0001;

// For simple ray marcher: if absolute value is less than this when iteration max is reached, treat as an intersection
const float RAY_MARCH_SIMPLE_SDF_THRESHOLD2 = 0.001;

// For bisecting ray marcher: stop ray marching if absolute value is less than this (during initial phase before bisection)
const float RAY_MARCH_BISECT_SDF_THRESHOLD = 0.00005;

// For bisecting ray marcher: stop bisecting if difference between upper and lower distance bounds is less than this
const float RAY_MARCH_BISECT_DISTANCE_THRESHOLD = 0.00002;

// For bisecting ray marcher: minimum step size when not yet bisecting
// (larger value may cause it to skip over fine details or edges)
const float RAY_MARCH_BISECT_MIN_STEP = 0.01;

// For bisecting ray marcher: take this many steps before applying RAY_MARCH_BISECT_MIN_STEP
const int RAY_MARCH_BISECT_INITIAL_STEP_COUNT = 8;

// Define this to scale up the step size in increments after RAY_MARCH_BISECT_INITIAL_STEP_COUNT reached
// May be slightly slower, but gives more precise outline for nearest edge
//  (will not help for later edges, if ray passes close to multiple edges)
#define RAY_MARCH_BISECT_RAMP_UP_STEP

// Multiplier to use if RAY_MARCH_BISECT_RAMP_UP_STEP is defined
const float RAY_MARCH_BISECT_RAMP_UP_MULT = 1.25;



// Set this to non-zero to show the approximate number of iterations used for each pixel
//     red: 100 iterations or more
//     yellow: 50 iterations or more
//     green: 20 iterations or more 
//     cyan: 10 iterations or more
//     blue: 5 iterations or more
//     cow color: fewer than 5 iterations
#define SHOW_ITERATION_COUNT 0

// If set to false, the model will initially be revolving, but will stop as soon as you click on the image.
// If set to true, you will not be able to rotate manually.
const bool FORCE_TURNTABLE_MODE = false;

const float TURNTABLE_SPEED1 = 1./5.; // rotation speed
const float TURNTABLE_SPEED2 = 1./15.; // speed of oscillation in secondary direction
const float TURNTABLE_DEPTH = 0.3; // how much the angle changes in the secondary direction

const float CAMERA_SHIFT_Y = 0.;
const float CAMERA_Z_DISTANCE = 4.5;
const float VIEW_ANGLE_FACTOR = 2.0;

const vec3 BACKGROUND_COLOR = vec3(0.1, 0.1, 0.2);

// NOTE: If SPOT_TEXTURE_EDGE_GRADIENT is defined, the edges of the shapes in the texture will be antialiased 

const float PI = 3.14159265;


// AABB intersection implementation
vec2 vec2_sort(vec2 v) {
    return vec2(min(v.x, v.y), max(v.x, v.y));
}

vec2 slab_nearfar(float a, float b, float rV) {
    return vec2_sort(rV * vec2(a, b));
}

vec2 intersect_box(vec3 P1, vec3 P2, vec3 rV, vec3 ray_O) {
    P1 -= ray_O;
    P2 -= ray_O;
    
    vec2 tx = slab_nearfar(P1.x, P2.x, rV.x);
    vec2 ty = slab_nearfar(P1.y, P2.y, rV.y);
    vec2 tz = slab_nearfar(P1.z, P2.z, rV.z);
    
    return vec2(max(tx.r, max(ty.r, tz.r)), min(tx.g, min(ty.g, tz.g)));
}

vec2 spot_ray_bound_intersection(vec3 ray_O, vec3 V) {
    return intersect_box(SPOT_BOUNDING_BOX_LOW, SPOT_BOUNDING_BOX_HIGH, 1./V, ray_O);
}

// Simple sphere tracing ray marcher
float ray_march_simple(vec3 ray_O, vec3 V, out int itr_count) {
    float t, t_max;
    
    {
        vec2 nearfar = spot_ray_bound_intersection(ray_O, V);
        
        t = max(nearfar.x, 0.);
        t_max = nearfar.y;
        
        if (t >= t_max) {
            itr_count = 0;
            return 1e20;
        }
    }
    
    // "min(0, iFrame)" is just to prevent loop unrolling by the compiler
    int itr_max = min(0, iFrame) + RAY_MARCH_SIMPLE_MAX_ITERATIONS;
    
    for (int itr=0; itr < itr_max; itr++) {
        if (t > t_max) {
            itr_count = itr;
            return 1e20;
        }
        
        float d = spot_sdf_and_gradient(ray_O + t*V).w;
        
        if (d < RAY_MARCH_SIMPLE_SDF_THRESHOLD
                || (d < RAY_MARCH_SIMPLE_SDF_THRESHOLD2 && itr == itr_max - 1)) {
            itr_count = itr;
            return t;
        }
        
        t += d;
    }
    
    itr_count = itr_max;
    
    return 1e20;
}

// Ray marcher that overshoots and then uses bisection to find intersection
float ray_march_bisect(vec3 ray_O, vec3 V, out int itr_count) {
    float t, t_max;
    
    {
        vec2 nearfar = spot_ray_bound_intersection(ray_O, V);
        
        t = max(nearfar.x, 0.);
        t_max = nearfar.y;
        
        if (t >= t_max) {
            itr_count = 0;
            return 1e20;
        }
    }
    
    float t_bound_near = 0.;
    float t_bound_far = 1e20;
    
    float f_near = 1e20;
    float f_far = -1e20;
    
#ifdef RAY_MARCH_BISECT_RAMP_UP_STEP
    float step_multipler = RAY_MARCH_BISECT_RAMP_UP_MULT;
#endif
    
    // "min(0, iFrame)" is just to prevent loop unrolling by the compiler
    int itr_max = min(0, iFrame) + RAY_MARCH_BISECT_MAX_ITERATIONS;
   
    for (int itr=0; itr < itr_max; itr++) {
        float d = spot_sdf_and_gradient(ray_O + t*V).w;
        
        if (d > 0.) {
            t_bound_near = t + d;
            f_near = d;
        } else {
            t_bound_far = t + d;
            f_far = d;
        }
        
        if (t_bound_far > 1e18) {
            if (abs(d) < RAY_MARCH_BISECT_SDF_THRESHOLD) {
                itr_count = itr;
                return t;
            }
            
            if (itr > RAY_MARCH_BISECT_INITIAL_STEP_COUNT) {
            
#ifndef RAY_MARCH_BISECT_RAMP_UP_STEP
                d = max(d, RAY_MARCH_BISECT_MIN_STEP);
#else
                d = max(d,  min(d*step_multipler, RAY_MARCH_BISECT_MIN_STEP));
                step_multipler *= RAY_MARCH_BISECT_RAMP_UP_MULT;
#endif
            }
            
            t += d;

            if (t > t_max) {
                itr_count = itr;
                return 1e20;
            }
        } else {
            t = 0.5*(t_bound_near + t_bound_far);
            
            if (t_bound_far - t_bound_near < RAY_MARCH_BISECT_DISTANCE_THRESHOLD
            
                    // Not necessary, but seems to help:
                    || (f_near - f_far) < RAY_MARCH_BISECT_DISTANCE_THRESHOLD
                        
                    ) {
                itr_count = itr;
                
                return t;
            }
        }
    }

    itr_count = itr_max;
    
    if (t_bound_far < 1e18) {
        return t = 0.5*(t_bound_near + t_bound_far);
    } else {
        return 1e20;
    }
}

// Note: this is a direct translation of the sRGB definition into branch-less GLSL (other people's code likely looks similar)
vec3 linear_to_srgb(vec3 C) {
    return mix(12.92*C, 1.055*pow(C, vec3(1./2.4)) - 0.055, step(0.0031308, C));
}

float rgb_to_luminance(vec3 c) {
    return dot(c, vec3(0.2126, 0.7152, 0.0722));
}

// Clip linear RGB colors, trying to preserve brightness and color
// (not sure if this counts as "tone mapping")
vec3 gamut_clip(vec3 rgb) {
    float target_luminance = clamp(rgb_to_luminance(rgb), 0., 1.);
    
    rgb = max(rgb, vec3(0));
    
    rgb /= max(max(max(rgb.r, rgb.g), rgb.b), 1e-12);
    
    float L = min(rgb_to_luminance(rgb), 0.9999);
    
    float a = target_luminance <= L ? target_luminance / L : (1. - target_luminance) / (1. - L);
        
    return a*rgb + (target_luminance - a*L);
}

mat3 make_camera_rotation_matrix(vec4 mouse4, vec3 resolution) {
    float xz, yz;
    
    if (FORCE_TURNTABLE_MODE || dot(abs(mouse4), vec4(1)) == 0.) {
        xz = PI * 2. * fract(TURNTABLE_SPEED1 * iTime);
        yz = -0.5*PI * TURNTABLE_DEPTH * sin(PI * 2. * fract(TURNTABLE_SPEED2 * iTime));
    } else {
        vec2 m = 2. * (mouse4.xy / resolution.xy - 0.5);
        
        xz = PI * m.x + PI;
        yz = 0.5*PI * clamp(m.y, -0.999, 0.999);
    }

    float sin_xz = sin(xz);
    float cos_xz = cos(xz);
    float cos_yz = cos(yz);
    float sin_yz = sin(yz);
    
    // (normalize is for precision loss only)
    return mat3(
        normalize(vec3(cos_xz, 0, -sin_xz)),
        normalize(vec3(-sin_yz*sin_xz, cos_yz, -sin_yz*cos_xz)),
        normalize(vec3(sin_xz*cos_yz, sin_yz, cos_xz*cos_yz))); 
}

// sun should really be 34 degrees above horizon for this lighting approximation to make sense
vec3 LIGHT_DIR = normalize(vec3(cos(PI/180.*34.)*cos(PI/4.), sin(PI/180.*34.), cos(PI/180.*34.)*sin(PI/4.)));

// Daylight lighting approximation for Lambertian surfaces
// Loosely based on the "Autumn Park" HDRI environment map from Poly Haven
vec3 lighting_approx(vec3 N) {
    float d = dot(N, LIGHT_DIR);
    
    vec3 c;
    
    c.b = 0.5013 + (0.288 - 0.1489*N.y)*N.y + (0.1818 + 0.1376*d)*d;
    c.g = -0.1509 + 0.8631*c.b + (0.02822*N.y - 0.1239)*N.y;
    c.r = -0.3396*c.b + 1.0746*c.g;
    
    return c + max(d, 0.) * vec3(1.968, 1.889, 1.523);
}

mat2x3 projection_Jacobian(float x, float y, float image_z) {
    float s = 1. / sqrt(x*x + y*y + image_z*image_z);
    float s3 = s*s*s;
    
    return mat2x3(s - s3*x*x, -s3*x*y, -s3*x*image_z,
                  -s3*x*y, s - s3*y*y, -s3*y*image_z);
}

void mainImage(out vec4 fragColor, in vec2 fragCoord) {
    fragCoord -= 0.5*iResolution.xy;

    mat3 rotation_matrix = make_camera_rotation_matrix(iMouse, iResolution);
    
    vec3 V = rotation_matrix * normalize(vec3(fragCoord, VIEW_ANGLE_FACTOR*iResolution.y));
    
    vec3 ray_O = rotation_matrix * (vec3(0.5, 0.5, -CAMERA_Z_DISTANCE) - vec3(0.5, 0.5, 0.25)) + vec3(0, CAMERA_SHIFT_Y, 0);

    mat2x3 J_proj = rotation_matrix * projection_Jacobian(fragCoord.x, fragCoord.y, VIEW_ANGLE_FACTOR*iResolution.y);
    
    vec3 color = BACKGROUND_COLOR;
    
    int itr_count;
    float t = ray_march(ray_O, V, itr_count);
    
    if (t < 1e18) {
        vec3 P = ray_O + t*V;
        
#ifdef SPOT_TEXTURE_EDGE_GRADIENT // antialias the texture

        vec4 f_grad;
    
        int color_index = spot_get_color_index(P, f_grad);
        
        vec3 N = normalize(spot_sdf_and_gradient(P).xyz);
        
        vec3 light = 0.5 * lighting_approx(N) + 0.1;

        color = gamut_clip(light * spot_color_index_to_rgb(color_index));
        
        if (f_grad.w > -1e18) {
            // Get combined Jacobian matrix (partial derivatives) for turning pixel coordinate into a camera ray,
            //      then intersecting that ray with the plane that locally approximates the surface.
            // This matrix allows computing the gradient of a function that defines the edge of the nearby shape
            //      (like an SDF, but does not give the distance to the edge, and the magnitude of the gradient is arbitrary).
            // The gradient will be used to estimate distance to edge, for anti-aliasing.
            J_proj = (outerProduct(-t / dot(V, N) * V, N) + mat3(t)) * J_proj;
            
            // At edges, where dot(V, N) > -1e-2, antialiasing is skipped
            //  (could skip a lot of calculations, but it probably wouldn't help performance because it only affects a small number of pixels)
            
            // Antialiasing is applied to clipped RGB values (but before conversion to sRGB)
            //      otherwise there would likely still be aliasing in brightly lit areas
            
            {
                // not using the built-in GLSL function "sign" because it seems to cause weird problems in Windows
                float d = dot(V, N) > -1e-2 ? (f_grad.w < 0. ? -1. : 1.) : f_grad.w / max(1e-12, length(f_grad.xyz * J_proj));
                
                if (d > -0.7) {
                    color = mix(color, 
                                      gamut_clip(light * spot_color_index_to_rgb(color_index == SPOT_COLOR_NOSTRILS ? SPOT_COLOR_NOSE : SPOT_COLOR_BASE)), 
                                      smoothstep(-0.7, 0.7, d));
                }
            }
            
            if (color_index == SPOT_COLOR_EYES) {
                f_grad = spot_get_eye_highlight_grad(P);
                
                float d = dot(V, N) > -1e-2 ? (f_grad.w < 0. ? -1. : 1.) : f_grad.w / max(1e-12, length(f_grad.xyz * J_proj));
                
                if (d < 0.7) {
                    color = mix(gamut_clip(light * spot_color_index_to_rgb(SPOT_COLOR_EYE_HIGHLIGHT)), 
                                      color,
                                      smoothstep(-0.7, 0.7, d));
                }
            }
        }
        
#else // spot_get_color_index does not return distance and gradient, so don't antialias

        int color_index = spot_get_color_index(P);
        
        vec3 N = normalize(spot_sdf_and_gradient(P).xyz);

        color = gamut_clip((0.5 * lighting_approx(N) + 0.1) * spot_color_index_to_rgb(color_index));

#endif

    }

#if SHOW_ITERATION_COUNT != 0
    if (itr_count >= 100) color = vec3(1,0,0);
    else if (itr_count >= 50) color = vec3(1,1,0);
    else if (itr_count >= 20) color = vec3(0,1,0);
    else if (itr_count >= 10) color = vec3(0,1,1);
    else if (itr_count >= 5) color = vec3(0,0,1);
#endif
    
    fragColor = vec4(linear_to_srgb(color), 1.0);
}
