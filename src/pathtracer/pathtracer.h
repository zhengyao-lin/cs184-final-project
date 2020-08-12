#ifndef CGL_PATHTRACER_H
#define CGL_PATHTRACER_H

#include "CGL/timer.h"

#include "scene/bvh.h"
#include "pathtracer/sampler.h"
#include "pathtracer/intersection.h"

#include "application/renderer.h"

#include "scene/scene.h"
using CGL::SceneObjects::Scene;

#include "scene/environment_light.h"
using CGL::SceneObjects::EnvironmentLight;

using CGL::SceneObjects::BVHNode;
using CGL::SceneObjects::BVHAccel;

namespace CGL {

    class PathTracer {
    public:
        PathTracer();
        ~PathTracer();

        /**
         * Sets the pathtracer's frame size. If in a running state (VISUALIZE,
         * RENDERING, or DONE), transitions to READY b/c a changing window size
         * would invalidate the output. If in INIT and configuration is done,
         * transitions to READY.
         * \param width width of the frame
         * \param height height of the frame
         */
        void set_frame_size(size_t width, size_t height);

        void write_to_framebuffer(ImageBuffer& framebuffer, size_t x0, size_t y0, size_t x1, size_t y1);

        /**
         * If the pathtracer is in READY, delete all internal data, transition to INIT.
         */
        void clear();

        void autofocus(Vector2D loc) {
            vector<Spectrum> out = vector<Spectrum>();
            //out.push_back(Spectrum());
            /*for (int x = 0; x < out.size(); x ++) {
                std::cout<<out[x]<<std::endl;
            }*/
            /*for (int x = -10; x < 11; x ++) {
                for (int y = -10; y < 11; y++) {
                    sampleBuffer.update_pixel(out[(x+10)*21 + (y + 10)], x + 100, y + 100);
                }
            }*/
            double step;
            double dir = 1.0;
            double min;
            double max;
            double pos;
            double max_var = 0;
            double max_pos;
            double temp_var;
            double temp_p;
            double temp_x;
            if (camera->model == CameraModel::COMPOUND_LENS) {
                min = camera->lenses[camera->current_lens].infinity_focus;
                max = camera->lenses[camera->current_lens].near_focus*2.0;
                step = abs((min - max) / 25);
                dir = -1.0;
                pos = min;
                for (int x = 0; x < 26; x += 1) {
                    pos = min + x * step;
                    camera->lenses[camera->current_lens].sensor_depth = pos;
                    out = vector<Spectrum>();
                    cell_sample(loc, &out);
                    temp_var = calc_var(&out);
                    std::cout<<temp_var<<" "<<pos<<std::endl;
                    if (temp_var > max_var) {
                        max_var = temp_var;
                        max_pos = pos;
                        //temp_x = pos;
                        std::cout<<"max^^"<<std::endl;
                    }
                }
                std::cout<<"yoyoyo"<<std::endl;
                step = step / 5.0;
                min = max_pos - (step * 10.0);
                for (int x = 0; x < 21; x += 1) {
                    pos = min + x * step;
                    camera->lenses[camera->current_lens].sensor_depth = pos;
                    out = vector<Spectrum>();
                    cell_sample(loc, &out);
                    temp_var = calc_var(&out);
                    std::cout<<temp_var<<std::endl;
                    if (temp_var > max_var) {
                        max_var = temp_var;
                        max_pos = pos;
                        //temp_x = pos;
                        std::cout<<"max^^"<<std::endl;
                    }
                }
                camera->lenses[camera->current_lens].sensor_depth = max_pos;
            } else if (camera->model == CameraModel::THIN_LENS) {
                min = 0;
                max = MAXFLOAT;
                step = 0.2;

                for (int x = 1; x < 25; x += 1) {
                    temp_p = 1.0 / (x * 0.02);
                    camera->focalDistance = temp_p;
                    out = vector<Spectrum>();
                    cell_sample(loc, &out);
                    temp_var = calc_var(&out);
                    std::cout<<temp_var<<std::endl;
                    if (temp_var > max_var) {
                        max_var = temp_var;
                        max_pos = temp_p;
                        temp_x = x;
                        std::cout<<"max^^"<<std::endl;
                    }
                }
                std::cout<<"yoyoyo"<<std::endl;
                for (int x = -10; x < 10; x += 1) {
                    temp_p = 1.0 / ((temp_x + ((double) x / 10)) * 0.02);
                    camera->focalDistance = temp_p;
                    out = vector<Spectrum>();
                    cell_sample(loc, &out);
                    temp_var = calc_var(&out);
                    std::cout<<temp_var<<std::endl;
                    if (temp_var > max_var) {
                        max_var = temp_var;
                        max_pos = temp_p;
                        //temp_x = x;
                        std::cout<<"max^^"<<std::endl;
                    }
                }



                pos = max_pos;
                camera->focalDistance = pos;

            } else {
                return;
            }


            int dir_change = 0;
            double var;
            double last_var;
            //camera->focalDistance = pos;
            /*out = vector<Spectrum>();
            cell_sample(loc, &out);
            last_var = calc_var(&out);
            //out.clear();
            camera->focalDistance += dir * step;
            double last_pos;
            while (dir_change < 7) {
                out = vector<Spectrum>();
                cell_sample(loc, &out);
                var = calc_var(&out);
                std::cout<<var<<std::endl;
                if (var < last_var) {
                    step = step / 2.0;
                    dir *= -1.0;
                    dir_change += 1;
                    //hiiii
                }
                last_var = var;
                last_pos = camera->focalDistance;
                camera->focalDistance += (dir * step);
                if (camera->focalDistance > max || camera->focalDistance < min) {
                    dir = (int)round(abs(last_pos - camera->focalDistance) / (last_pos - camera->focalDistance));
                    std::cout<<dir<<std::endl;
                    camera->focalDistance = last_pos;
                }
            }*/

        }

        double calc_var(vector<Spectrum> *out) {
            double s1 = 0;
            double s2 = 0;
            double s;
            for (int x = 0; x < (*out).size(); x ++) {
                s = (*out)[x].r + (*out)[x].g + (*out)[x].b;
                //std::cout<<s<<std::endl;
                s1 += s;
                s2 += s * s;
            }
            return (double)(1.0 / ((*out).size() - 1.0)) * (s2 - ((s1 * s1)/((double)(*out).size())));
        }

        /**
         * Trace an ray in the scene.
         */
        Spectrum estimate_direct_lighting_hemisphere(const Ray& r, const SceneObjects::Intersection& isect);
        Spectrum estimate_direct_lighting_importance(const Ray& r, const SceneObjects::Intersection& isect);

        Spectrum est_radiance_global_illumination(const Ray& r);
        Spectrum zero_bounce_radiance(const Ray& r, const SceneObjects::Intersection& isect);
        Spectrum one_bounce_radiance(const Ray& r, const SceneObjects::Intersection& isect);
        
        Spectrum at_least_one_bounce_radiance(const Ray& r, const SceneObjects::Intersection& isect);
        Spectrum at_least_one_bounce_radiance(const Ray &r, const SceneObjects::Intersection &isect, size_t current_depth);

        void cell_sample(Vector2D loc, vector<Spectrum> *out);

        Spectrum debug_shading(const Vector3D& d) {
            return Vector3D(abs(d.r), abs(d.g), .0).unit();
        }

        Spectrum normal_shading(const Vector3D& n) {
            return n * .5 + Spectrum(.5);
        }

        /**
         * Trace a camera ray given by the pixel coordinate.
         */
        void raytrace_pixel(size_t x, size_t y);

        // Integrator sampling settings //

        size_t max_ray_depth; ///< maximum allowed ray depth (applies to all rays)
        size_t ns_aa;         ///< number of camera rays in one pixel (along one axis)
        size_t ns_area_light; ///< number samples per area light source
        size_t ns_diff;       ///< number of samples - diffuse surfaces
        size_t ns_glsy;       ///< number of samples - glossy surfaces
        size_t ns_refr;       ///< number of samples - refractive surfaces

        size_t samplesPerBatch;
        float maxTolerance;
        bool direct_hemisphere_sample; ///< true if sampling uniformly from hemisphere for direct lighting. Otherwise, light sample

        // Components //

        BVHAccel* bvh;                 ///< BVH accelerator aggregate
        EnvironmentLight* envLight;    ///< environment map
        Sampler2D* gridSampler;        ///< samples unit grid
        Sampler3D* hemisphereSampler;  ///< samples unit hemisphere
        HDRImageBuffer sampleBuffer;   ///< sample buffer
        Timer timer;                   ///< performance test timer

        std::vector<int> sampleCountBuffer;   ///< sample count buffer
        Spectrum temp_sample;

        Scene* scene;         ///< current scene
        Camera* camera;       ///< current camera

        // Tonemapping Controls //

        float tm_gamma;                           ///< gamma
        float tm_level;                           ///< exposure level
        float tm_key;                             ///< key value
        float tm_wht;                             ///< white point
    };

}  // namespace CGL

#endif  // CGL_RAYTRACER_H
