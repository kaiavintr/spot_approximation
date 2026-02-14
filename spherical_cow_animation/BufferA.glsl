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
    
GitHub repository for this code (has some notes and image comparisons):
https://github.com/kaiavintr/spot_approximation

Please contact Kaia Vintr with questions regarding the code via:
- comment on this Shadertoy shader:
    https://www.shadertoy.com/view/t3tfD4
- GitHub issue
- Bluesky: https://bsky.app/profile/kaiavintr.bsky.social
- Mastodon: https://mastodon.social/@kaiavintr
    
*/

// See comments and configuration parameters in Common


// Ray marcher that overshoots and then uses bisection to find intersection
float ray_march(vec3 ray_O, vec3 V, float spherize, out int itr_count) {
    float t, t_max;
    
    {
        vec2 nearfar = spot_ray_bound_intersection(ray_O, V, spherize);
        
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
        float d = spot_sdf_and_gradient_spherize(ray_O + t*V, spherize).w;
        
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

void mainImage(out vec4 fragColor, in vec2 fragCoord) {
    fragCoord *= 4.;
    
    if (fragCoord.x > iResolution.x || fragCoord.y > iResolution.y) {
        fragColor = vec4(1e20);
        return;
    }

    fragCoord -= 0.5*iResolution.xy;

    mat3 rotation_matrix = make_camera_rotation_matrix(iMouse, iResolution, iTime);
    
    vec3 V = rotation_matrix * normalize(vec3(fragCoord, VIEW_ANGLE_FACTOR*iResolution.y));
    
    vec3 ray_O = rotation_matrix * (vec3(0.5, 0.5, -CAMERA_Z_DISTANCE) - vec3(0.5, 0.5, 0.25)) + vec3(0, CAMERA_SHIFT_Y, 0);
    
    float spherize; // value between 0 and 1 that controls morph position between the Spot approximation and a sphere

    {
        float k = 1./SPHERIZE_CYCLE_PERIOD * iTime;
        
        k = 2.*(k - floor(k));
        
        if (k > 1.) k = 2. - k;
        
        spherize = smoothstep(0.1, 0.9, k);
    }
    
    int itr_count;
    float t = ray_march(ray_O, V, spherize, itr_count);

#ifdef HIGH_QUALITY
    // downsampled texture mapping data is not used in high quality mode
    fragColor = vec4(vec3(0), t);
#else
    if (t < 1e18) {
        vec3 P0 = ray_O + t*V;
       
        float step = spherize / float(TEXTURE_DISTORT_STEP_COUNT);
        float sph = spherize;
        
        // min(0, iFrame) is to stop the compiler from unrolling the loop
        int itr_max = min(0, iFrame) + TEXTURE_DISTORT_STEP_COUNT + 2;
        
        for (int i=0; i < itr_max; i++) {
            vec4 f = spot_sdf_and_gradient_spherize(P0, max(0., sph));
            
            // one Newton's method iteration
            P0 -= f.xyz * f.w / dot(f.xyz, f.xyz);
            
            // allow sph to go negative once, so the spherize=0 pass is repeated
            if (sph == -1.) break;
            
            if (sph < 0.001) sph = -1.;
            else sph -= step;
        }
        
        fragColor = vec4(P0, t);
    } else {
        fragColor = vec4(vec3(0), t);
    }
#endif
}
