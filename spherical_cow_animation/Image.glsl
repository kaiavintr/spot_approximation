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


// Modified version of the bisecting ray marcher
float ray_march(vec3 ray_O, vec3 V, vec2 t_bound_outer, float spherize, out int itr_count) {
    t_bound_outer.x = max(0., t_bound_outer.x);

    float t, t_max;
    
    {
        vec2 nearfar = spot_ray_bound_intersection(ray_O, V, spherize);
        
        t = max(nearfar.x, t_bound_outer.x);
        t_max = min(nearfar.y, t_bound_outer.y);
        
        if (t >= t_max) {
            itr_count = 0;
            return 1e20;
        }
    }
    
    float t_bound_near = t_bound_outer.x;
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
            
#ifndef RAY_MARCH_BISECT_RAMP_UP_STEP
            d = max(d, RAY_MARCH_BISECT_MIN_STEP);
#else
            d = max(d,  min(d*step_multipler, RAY_MARCH_BISECT_MIN_STEP));
            step_multipler *= RAY_MARCH_BISECT_RAMP_UP_MULT;
#endif
            
            t += d;

            if (t > t_max) {
                itr_count = itr;
                return t_bound_outer.y;
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
        return t_bound_outer.y;
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
    bool is_edge = false;
    float t_min = 1e20;
    float t_max = -1e20;
    float t_interp = 1e20;
    
#ifndef HIGH_QUALITY
    vec3 p_interp = vec3(0);
    vec3 dp_dx = vec3(0);
    vec3 dp_dy = vec3(0);
    bool need_texture_steps = true;
    bool need_texture_steps2 = true;
#endif

    {
        vec2 xy = 0.25*fragCoord - 0.5;
        ivec2 ixy = ivec2(xy);

        // Get the 4 values from the downsampled data that are needed for linear
        //      interpolation
        
        vec4 v00 = texelFetch(iChannel0, ixy, 0);
        vec4 v10 = texelFetch(iChannel0, ixy + ivec2(1, 0), 0);
        vec4 v01 = texelFetch(iChannel0, ixy + ivec2(0, 1), 0);
        vec4 v11 = texelFetch(iChannel0, ixy + 1, 0);
        
        // Check if all values are present (ray did not miss the shape) or no
        //      values are present, and get bounds on intersection distance
        int n_count = 0;
        
        if (v00.w < 1e18) {
            t_min = min(t_min, v00.w);
            t_max = max(t_max, v00.w);
            n_count++;
        }
        
        if (v10.w < 1e18) {
            t_min = min(t_min, v10.w);
            t_max = max(t_max, v10.w);
            n_count++;
        }
        
        if (v01.w < 1e18) {
            t_min = min(t_min, v01.w);
            t_max = max(t_max, v01.w);
            n_count++;
        }
        
        if (v11.w < 1e18) {
            t_min = min(t_min, v11.w);
            t_max = max(t_max, v11.w);
            n_count++;
        }
        
        // If no downsampled values are present (ray missed for all neighboring
        //      downsampled pixels) then assume the ray misses in the full-res image.
        // This works for this particular model (very smooth/rounded) but it would
        //      likely not be safe for models with fine details.
        if (n_count == 0) {
            fragColor = vec4(linear_to_srgb(BACKGROUND_COLOR), 1);
            return;
        }
        
        // Treat as an "edge" region if one or more downsampled values is missing.
        // If it's an edge, we don't use the downsampled data except for bounds on the
        //      intersection distance, because we can't interpolate.
        is_edge = n_count != 4;
        
#ifndef HIGH_QUALITY
        // set flag indicating that we'll need to use expensive texture mapping iterations
        need_texture_steps = is_edge;
#endif
        
        if ( ! is_edge) {
            vec2 fxy = xy - floor(xy);
            
            // Perform linear interpolation on the texture location
            vec4 v_interp = mix(mix(v00, v10, fxy.x), mix(v01, v11, fxy.x), fxy.y);
            t_interp = v_interp.w;

#ifndef HIGH_QUALITY
            // Estimate partial derivatives for the texture location
            
            p_interp = v_interp.xyz;
            dp_dx = 0.25 * 0.5 * (v10.xyz + v11.xyz - v00.xyz - v01.xyz);
            dp_dy = 0.25 * 0.5 * (v01.xyz + v11.xyz - v00.xyz - v10.xyz);
            
            // Check if the neighborhood values look safe to use
            //      (partial derivatives are consistent and distance range is not too large)

            vec3 dp_dx1 = v10.xyz - v00.xyz;
            vec3 dp_dx2 = v11.xyz - v01.xyz;
            vec3 dp_dy1 = v01.xyz - v00.xyz;
            vec3 dp_dy2 = v11.xyz - v10.xyz;
            
            need_texture_steps = (distance(dp_dx1, dp_dx2) / max(length(dp_dx1), length(dp_dx2)) > 0.5) || (distance(dp_dy1, dp_dy2) / max(length(dp_dy1), length(dp_dy2)) > 0.5)
                || t_max - t_min > 0.1;

            need_texture_steps2 = (distance(dp_dx1, dp_dx2) / max(length(dp_dx1), length(dp_dx2)) > 0.1) || (distance(dp_dy1, dp_dy2) / max(length(dp_dy1), length(dp_dy2)) > 0.1)
                || t_max - t_min > 0.1;

    #ifdef LOW_QUALITY
            // In low quality mode, if neighborhood is bad but not too bad, try using
            //      a different ("blurrier") estimate of the partial derivatives
            if (need_texture_steps2 && ! need_texture_steps) {
                dp_dx = 0.25 * 0.5 * (length(v10.xyz - v00.xyz) > length(v11.xyz - v01.xyz) ? v10.xyz - v00.xyz : v11.xyz - v01.xyz);
                dp_dy = 0.25 * 0.5 * (length(v01.xyz - v00.xyz) > length(v11.xyz - v10.xyz) ? v01.xyz - v00.xyz : v11.xyz - v10.xyz);
                need_texture_steps2 = false;
            }
    #endif

#endif
        }
    }
    
    fragCoord -= 0.5*iResolution.xy;

    mat3 rotation_matrix = make_camera_rotation_matrix(iMouse, iResolution, iTime);
    
    vec3 V = rotation_matrix * normalize(vec3(fragCoord, VIEW_ANGLE_FACTOR*iResolution.y));
    
    vec3 ray_O = rotation_matrix * (vec3(0.5, 0.5, -CAMERA_Z_DISTANCE) - vec3(0.5, 0.5, 0.25)) + vec3(0, CAMERA_SHIFT_Y, 0);

#ifdef SPOT_TEXTURE_EDGE_GRADIENT
    mat2x3 J_proj = rotation_matrix * projection_Jacobian(fragCoord.x, fragCoord.y, VIEW_ANGLE_FACTOR*iResolution.y);
#endif
    
    float spherize; // value between 0 and 1 that controls morph position between the Spot approximation and a sphere

    {
        float k = 1./SPHERIZE_CYCLE_PERIOD * iTime;
        
        k = 2.*(k - floor(k));
        
        if (k > 1.) k = 2. - k;
        
        spherize = smoothstep(0.1, 0.9, k);
    }
    
    vec3 color = BACKGROUND_COLOR;
    
    vec2 t_bound = vec2(-1e20, 1e20);
    
    if ( ! is_edge) {
        // Use bound of low-res ray march distances in neighborhood to get a bound for the high-res distance
        // Again, this works because the shape is smooth, but it would not be safe for all shapes.
        
        float t_gap = max(0.002, 0.1*(t_max - t_min));
        t_bound = vec2(t_min - t_gap, t_max + t_gap);
        
        if ( ! is_edge && t_max - t_min < 0.02) {
            // linearly interpolated value allows using a tighter bound
            
            t_gap = max(0.001, 0.1*(t_max - t_min));
            
            vec2 t_bound2 = vec2(t_interp - t_gap, t_interp + t_gap);
            
            t_bound.x = max(t_bound.x, t_bound2.x);
            t_bound.y = max(t_bound.y, t_bound2.y);
        }
    }
    
    int itr_count;
    
    float t = ray_march(ray_O, V, t_bound, spherize, itr_count);
    
    if (t < 1e18) {
        vec3 P0 = ray_O + t*V;
        
#ifndef HIGH_QUALITY
        // Force using expensive texture mapping in problematic areas
        
        if ( ! need_texture_steps) {
            if (p_interp.y < -0.3 && abs(p_interp.x) < 0.12 && p_interp.z < 0.15 && p_interp.z > -0.01) {
                need_texture_steps = true;
            }
            
            if (p_interp.y < -0.3 && abs(p_interp.x) < 0.12 && p_interp.z < 0.9 && p_interp.z > 0.65) {
                need_texture_steps = true;
            }
        }
#endif
        
    #ifndef HIGH_QUALITY
        // If model is not spherized, the texture locatation doesn't need to be mapped, so disable the mapping steps.
        if (spherize <= SPHERIZE_LOW_THRESHOLD) need_texture_steps = need_texture_steps2 = false;
    #endif

#ifdef SPOT_TEXTURE_EDGE_GRADIENT // antialias the texture

        vec4 f_grad;
        int color_index;
        bool have_color = false;

    #ifndef HIGH_QUALITY
        if ( ! need_texture_steps && need_texture_steps2) {
            // Evaluate texture function so we can get the "SDF" gradients. Gradients are needed in some cases for testing
            //      if expensive mapping is needed.
            // I don't know if this a good idea, since spot_get_color_index might need evaluating again later (and it's expensive)
            // (also f_grad and color_index consume registers)
            color_index = spot_get_color_index(spherize <= SPHERIZE_LOW_THRESHOLD ? P0 : p_interp, f_grad);
            have_color = true;
        }
    #endif

        // Use a single variable-length loop for multiple calls to spot_sdf_and_gradient_spherize, to reduce compile time
        // First iteration computes normal (N)
        
        // Remaining iterations map three samples (close to P0) to less and less "spherized" surfaces until the undistorted surface is reached.
        // Each time, we perform one 3D Newton's method step, mapping the point along the gradient to where we estimate the surface is.
        // We do this twice for the final (undistorted) surface so we get a point very close to the surface (otherwise the texture function may not work well).
        // All of this is repeated three times because we need three samples so we can estimate the Jacobian for antialiasing.
        // The three samples are stored in P0, J_proj[0], and J_proj[1] to (potentially) save registers

        vec3 N;
       
        float step = spherize / float(TEXTURE_DISTORT_STEP_COUNT);
        float sph = spherize;
        
        // min(0, iFrame) is to stop the compiler from unrolling the loop
        int itr_max = min(0, iFrame) + (TEXTURE_DISTORT_STEP_COUNT + 2) * 3 + 1;
        
        // the "min" is to try to stop the compiler from being too clever and partially unrolling the loop
        int phase = min(-1, iFrame | 0xffff);
        
        for (int i=0; i < itr_max; i++) {
            #ifndef LOW_QUALITY
            vec3 P = phase <= 0 ? P0 : phase==1 ? J_proj[0] : J_proj[1];
            #else
            vec3 P = P0;
            #endif
            
            vec4 f = spot_sdf_and_gradient_spherize(P, max(0., sph));
            
            if (phase == -1) {
                N = normalize(f.xyz);
                
    #ifndef HIGH_QUALITY
                if ( ! need_texture_steps && need_texture_steps2) {
                    float d1 = dot(V, N) > -1e-2 ? (f_grad.w < 0. ? -1. : 1.) : f_grad.w / max(1e-12, length(f_grad.xyz * mat2x3(dp_dx, dp_dy)));
                    
                    if (abs(d1) < 2.) {
                        need_texture_steps = true;
                        have_color = false;
                    }
                }
                
                if ( ! need_texture_steps) {
                    break;
                }
    #else
                // If model is not spherized, we only needed to get the surface normal, so break out of the loop
                if (spherize <= SPHERIZE_LOW_THRESHOLD) {
                    break;
                }
    #endif

    #ifndef LOW_QUALITY
                // Get combined Jacobian matrix (partial derivatives) for turning pixel coordinate into a camera ray,
                //      then intersecting that ray with the plane that locally approximates the surface.
                // If we weren't distorting the shape, we would use this matrix directly to compute the gradient of the function
                ///     that defines the edge of each colored area (as function of pixel coordinates).
                // Because we are distorting the shapes in this version, we sample three points on the plane instead, and then use
                //      those points (after mapping) to get a new estimate of the Jacobian, which we will then use for gradient
                //      computation.
                // The gradient will be used to estimate distance to edge, for antialiasing.
                J_proj = (outerProduct(-t / dot(V, N) * V, N) + mat3(t)) * J_proj;
                
                // Sample three points around the pixel center on the plane
                // (equilateral triangle with arbitrarily chosen rotation)
                
                // Points are stored in P0, J_proj[0], and J_proj[1] to (potentially) save registers
                
                P = P0 + J_proj * (J_FINITE_DIFFERENCE_SCALE * vec2(-0.974370065, 0.22495105));
                
                vec3 P_tmp = P0 + J_proj * (J_FINITE_DIFFERENCE_SCALE * vec2(0.292371705, -0.956304756));

                P0 += J_proj * (J_FINITE_DIFFERENCE_SCALE * vec2(0.68199836, 0.731353702)); // first point
                
                J_proj[0] = P; // second point
                J_proj[1] = P_tmp; // third point
                
                sph -= step;
    #else
                phase = 2;
    #endif
            } else {
                // one Newton's method iteration
                P -= f.xyz * f.w / dot(f.xyz, f.xyz);
                
    #ifndef LOW_QUALITY
                if (phase == 0) {
                    P0 = P;
                } else if (phase == 1) {
                    J_proj[0] = P;
                } else {
                    J_proj[1] = P;
                }
    #else
                P0 = P;
    #endif
            }
            
            if (phase==2) {
                // allow sph to go negative once, so the spherize=0 pass is repeated
                if (sph <= -1.) break;
                
                if (sph <= 0.01) sph = -1.;
                else sph -= step;
                
    #ifndef LOW_QUALITY
                phase = 0;
    #endif
            } else {
                phase++;
            }
        }
        
        if (spherize > SPHERIZE_LOW_THRESHOLD) {

    #ifndef HIGH_QUALITY
            if ( ! need_texture_steps) {
                // Use texture location and partial derivatives from the low-res neighborhood
                P0 = p_interp;
                J_proj = mat2x3(dp_dx, dp_dy);
            }
            
            else 
        
    #endif //ifndef HIGH_QUALITY
        
            {
    #ifndef LOW_QUALITY
                // Get average (centroid) of the three texture location samples
                // (points are stored in P0, J_proj[0], and J_proj[1], to make things more confusing)
                vec3 tmp = 1./3. * (P0 + J_proj[0] + J_proj[1]);
                
                // Convert the three sample points back into an estimate of the Jacobian
                J_proj = mat2x3(J_proj[0] - P0, J_proj[1] - P0) * (1./J_FINITE_DIFFERENCE_SCALE * mat2x2(-0.64958004, 0.19491447, 0.14996737, -0.6375365));

                P0 = tmp;
    #endif
            }
        
        } else {
            // Model is not spherized -- compute the Jacobian in a straightforward way
            J_proj = (outerProduct(-t / dot(V, N) * V, N) + mat3(t)) * J_proj;
        }
        
        if ( ! have_color) {
            // evaluate texture using final location
            color_index = spot_get_color_index(P0, f_grad);
        }
        
        vec3 light = (0.5*lighting_approx(N) + 0.1);
        
        color = gamut_clip(light * spot_color_index_to_rgb(color_index));
        
        bool skip_aa = dot(V, N) > -1e-2;

    #ifdef LOW_QUALITY
        if (need_texture_steps) skip_aa = true;
    #endif
        
        if (f_grad.w != 0.) {
            {
                float d = skip_aa ? (f_grad.w < 0. ? -1. : 1.) : f_grad.w / max(1e-12, length(f_grad.xyz * J_proj));
                
                if (d > -0.7) {
                    color = mix(color, 
                                     gamut_clip(light * spot_color_index_to_rgb(color_index == SPOT_COLOR_NOSTRILS ? SPOT_COLOR_NOSE : SPOT_COLOR_BASE)), 
                                     smoothstep(-0.7, 0.7, d));
                }
            }
            
            if (color_index == SPOT_COLOR_EYES) {
                f_grad = spot_get_eye_highlight_grad(P0);
                
                float d = skip_aa ? (f_grad.w < 0. ? -1. : 1.) : f_grad.w / max(1e-12, length(f_grad.xyz * J_proj));
                
                if (d < 0.7) {
                    color = mix(gamut_clip(light * spot_color_index_to_rgb(SPOT_COLOR_EYE_HIGHLIGHT)), 
                                      color,
                                      smoothstep(-0.7, 0.7, d));
                }
            }
        }
        
#else // spot_get_color_index does not return distance and gradient, so don't antialias

        // Use a single variable-length loop for multiple calls to spot_sdf_and_gradient_spherize, to reduce compile time
        // First iteration computes normal (N)
        
        // Remaining iterations map three samples (close to P0) to less and less "spherized" surfaces until the undistorted surface is reached.
        // Each time, we perform one 3D Newton's method step, mapping the point along the gradient to where we estimate the surface is.
        // We do this twice for the final (undistorted) surface so we get a point very close to the surface (otherwise the texture function may not work well).
       
        float step = spherize / float(TEXTURE_DISTORT_STEP_COUNT);
        float sph = spherize;
        
        // min(0, iFrame) is to stop the compiler from unrolling the loop
        int itr_max = min(0, iFrame) + (TEXTURE_DISTORT_STEP_COUNT + 2) + 1;

        vec3 N;
        
        // the "min" is to try to stop the compiler from being too clever and partially unrolling the loop
        int phase = min(-1, iFrame | 0xffff);
        
        for (int i=0; i < itr_max; i++) {
            vec4 f = spot_sdf_and_gradient_spherize(P0, max(0., sph));
            
            if (phase == -1) {
                N = normalize(f.xyz);

    #ifndef HIGH_QUALITY
                if ( ! need_texture_steps) {
                    break;
                }
    #else
                if (spherize <= SPHERIZE_LOW_THRESHOLD) {
                    break;
                }
    #endif
            } else {
                // one Newton's method iteration
                P0 -= f.xyz * f.w / dot(f.xyz, f.xyz);
            }
            
            // allow sph to go negative once, so the spherize=0 pass is repeated
            if (sph == -1.) break;
            
            if (sph < 0.001) sph = -1.;
            else sph -= step;
            
            phase++;
        }

    #ifndef HIGH_QUALITY
        if ( ! need_texture_steps && spherize > SPHERIZE_LOW_THRESHOLD) {
            P0 = p_interp;
        }
    #endif

        int color_index = spot_get_color_index(P0);
        
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
