#include "pathtracer.h"

#include "scene/light.h"
#include "scene/sphere.h"
#include "scene/triangle.h"


using namespace CGL::SceneObjects;

namespace CGL {

PathTracer::PathTracer() {
  gridSampler = new UniformGridSampler2D();
  hemisphereSampler = new UniformHemisphereSampler3D();

  tm_gamma = 2.2f;
  tm_level = 1.0f;
  tm_key = 0.18;
  tm_wht = 5.0f;
}

PathTracer::~PathTracer() {
  delete gridSampler;
  delete hemisphereSampler;
}

void PathTracer::set_frame_size(size_t width, size_t height) {
  sampleBuffer.resize(width, height);
  sampleCountBuffer.resize(width * height);
}

void PathTracer::clear() {
  bvh = NULL;
  scene = NULL;
  camera = NULL;
  sampleBuffer.clear();
  sampleCountBuffer.clear();
  sampleBuffer.resize(0, 0);
  sampleCountBuffer.resize(0, 0);
}

void PathTracer::write_to_framebuffer(ImageBuffer &framebuffer, size_t x0,
                                      size_t y0, size_t x1, size_t y1) {
  sampleBuffer.toColor(framebuffer, x0, y0, x1, y1);
}

Spectrum
PathTracer::estimate_direct_lighting_hemisphere(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // For this function, sample uniformly in a hemisphere.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D &hit_p = r.o + r.d * isect.t;
  const Vector3D &w_out = w2o * (-r.d);

  // This is the same number of total samples as
  // estimate_direct_lighting_importance (outside of delta lights). We keep the
  // same number of samples for clarity of comparison.
  int num_samples = scene->lights.size() * ns_area_light;
  Spectrum L_out;

  // TODO (Part 3): Write your sampling loop here
  // TODO BEFORE YOU BEGIN
  // UPDATE `est_radiance_global_illumination` to return direct lighting instead of normal shading 

  Vector3D sampled_wi;
  Spectrum bsdf;
  Vector3D wi_world;
  Intersection isect_secondary;
  
  UniformHemisphereSampler3D sampler;

  for (int i = 0; i < num_samples; i++) {
    sampled_wi = sampler.get_sample();
    bsdf = isect.bsdf->f(w_out, -sampled_wi);

    sampled_wi.normalize();

    // get the incident ray direction in the world space
    wi_world = o2w * sampled_wi;
    wi_world.normalize();

    if (!bvh->intersect(Ray(hit_p, wi_world), &isect_secondary)) continue;

    L_out += bsdf * isect_secondary.bsdf->get_emission() * abs_cos_theta(sampled_wi);
  }

  return L_out * (2 * PI / (double)num_samples);
}

inline bool importance_sample_light(
  const BVHAccel *bvh,
  const SceneLight *light,
  const Vector3D &hit_p,
  const Matrix3x3 &w2o,
  const Vector3D &w_out,
  const Intersection &isect,
  Spectrum &out_spectrum
) {
  Vector3D sampled_wi_world, sampled_wi_object;
  float dist_to_light, pdf;
  Spectrum sampled_radiance, bsdf;

  sampled_radiance = light->sample_L(hit_p, &sampled_wi_world, &dist_to_light, &pdf);

  sampled_wi_object = w2o * sampled_wi_world;
  sampled_wi_object.normalize();

  // skip if the light is "behind" the surface
  if (sampled_wi_object.z < 0) return false;

  // sampled_wi_world should already be an unit vector
  // move the origin of the ray slightly towards the normal
  // so that we can avoid some numerical issue
  Ray test_ray(hit_p, sampled_wi_world);
  test_ray.max_t = dist_to_light;
  test_ray.min_t = EPS_D;

  if (bvh->has_intersection(test_ray)) return false;

  bsdf = isect.bsdf->f(w_out, sampled_wi_object);

  // pi here is to make sure the one-bounce operator K conserves energy
  out_spectrum = (bsdf * sampled_radiance) * (abs_cos_theta(sampled_wi_object) / pdf);

  return true;
}

Spectrum
PathTracer::estimate_direct_lighting_importance(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // To implement importance sampling, sample only from lights, not uniformly in
  // a hemisphere.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D &hit_p = r.o + r.d * isect.t;
  const Vector3D &w_out = w2o * (-r.d);
  Spectrum L_out;
  Spectrum sample;

  size_t samples = 0;

  for (SceneLight *light: scene->lights) {
    if (light->is_delta_light()) {
      if (importance_sample_light(bvh, light, hit_p, w2o, w_out, isect, sample)) {
        // only need to sample once for point light but need to weight different
        L_out += sample * ns_area_light;
      }
      samples += 1;
    } else {
      for (int i = 0; i < ns_area_light; i++) {
        if (importance_sample_light(bvh, light, hit_p, w2o, w_out, isect, sample)) {
          L_out += sample;
        }
      }
      samples += ns_area_light;
    }
  }

  return L_out * (1 / (double)samples);
}

Spectrum PathTracer::zero_bounce_radiance(const Ray &r,
                                          const Intersection &isect) {
  // TODO: Part 3, Task 2
  // Returns the light that results from no bounces of light
  return isect.bsdf->get_emission();
}

Spectrum PathTracer::one_bounce_radiance(const Ray &r,
                                         const Intersection &isect) {
  // TODO: Part 3, Task 3
  // Returns either the direct illumination by hemisphere or importance sampling
  // depending on `direct_hemisphere_sample`

  if (!direct_hemisphere_sample) {
    return estimate_direct_lighting_importance(r, isect);
  } else {
    return estimate_direct_lighting_hemisphere(r, isect);
  }
}

Spectrum PathTracer::at_least_one_bounce_radiance(const Ray &r,
                                                  const Intersection &isect,
                                                  size_t current_depth) {
  const double CONTINUATION_PDF = 0.6;

  Spectrum L_out;
  
  if (!isect.bsdf->is_delta())
    L_out += one_bounce_radiance(r, isect);

  // always recurse if we are on depth 0, otherwise recurse only if
  // continuation test is passed
  bool gi_enabled_and_depth_1 = max_ray_depth > 1 && current_depth == 1;
  bool recurse = gi_enabled_and_depth_1 ||
                 ((current_depth < max_ray_depth) && coin_flip(CONTINUATION_PDF));

  if (recurse) {
    Matrix3x3 o2w;
    make_coord_space(o2w, isect.n);

    const Vector3D &hit_p = r.o + r.d * isect.t;
    const Vector3D &w_out = o2w.T() * (-r.d);

    Vector3D sampled_wi_object;
    float pdf;
    Spectrum bsdf = isect.bsdf->sample_f(w_out, &sampled_wi_object, &pdf);
    sampled_wi_object.normalize();

    Vector3D sampled_wi_world = o2w * sampled_wi_object;
    sampled_wi_world.normalize();

    Ray next_ray(hit_p, sampled_wi_world);
    next_ray.min_t = EPS_F;

    Intersection isect_secondary;

    if (bvh->intersect(next_ray, &isect_secondary)) {
      Spectrum sampled_next_L =
        at_least_one_bounce_radiance(next_ray, isect_secondary, current_depth + 1);

      if (isect.bsdf->is_delta()) {
        L_out += zero_bounce_radiance(next_ray, isect_secondary);
      }

      L_out += bsdf * sampled_next_L
             * (abs_cos_theta(sampled_wi_object) / pdf / (gi_enabled_and_depth_1 ? 1 : CONTINUATION_PDF));
    }
  }

  return L_out;
}

Spectrum PathTracer::at_least_one_bounce_radiance(const Ray &r,
                                                  const Intersection &isect) {
  return at_least_one_bounce_radiance(r, isect, 1);
}

Spectrum PathTracer::est_radiance_global_illumination(const Ray &r) {
  Intersection isect;

  // You will extend this in assignment 3-2.
  // If no intersection occurs, we simply return black.
  // This changes if you implement hemispherical lighting for extra credit.

  if (!bvh->intersect(r, &isect))
    return Spectrum();

  // The following line of code returns a debug color depending
  // on whether ray intersection with triangles or spheres has
  // been implemented.

  // REMOVE THIS LINE when you are ready to begin Part 3.
  // L_out = (isect.t == INF_D) ? debug_shading(r.d) : normal_shading(isect.n);

  // TODO (Part 3): Return the direct illumination.

  // TODO (Part 4): Accumulate the "direct" and "indirect"
  // parts of global illumination into L_out rather than just direct

  if (max_ray_depth == 0) {
    return zero_bounce_radiance(r, isect);
  }

  return zero_bounce_radiance(r, isect) + at_least_one_bounce_radiance(r, isect);
}

void PathTracer::raytrace_pixel(size_t x, size_t y) {

  // TODO (Part 1.1):
  // Make a loop that generates num_samples camera rays and traces them
  // through the scene. Return the average Spectrum.
  // You should call est_radiance_global_illumination in this function.

  // TODO (Part 5):
  // Modify your implementation to include adaptive sampling.
  // Use the command line parameters "samplesPerBatch" and "maxTolerance"

  int num_samples = ns_aa;          // total samples to evaluate
  Vector2D origin = Vector2D(x, y); // bottom left corner of the pixel

  Spectrum sum;
  float sum_illum = 0, sum_sq_illum = 0;

  int i = 0;
  for (; i < num_samples;) {
    int actual_num_samples = 0;

    for (int j = 0; j < samplesPerBatch && i + j < num_samples; j++) {
      // pdf at each point is just 1 so no need to divide
      Vector2D sample = origin + gridSampler->get_sample();

      // normalize sample position
      Ray ray; double coeff;
      Spectrum sample_radiance;

      // keep trying until a ray successfully passes through
      // or the max nmuber of attempts is reached
      // TODO: remove hardcoded number of attempts;
      int attempts = 0;
      while (!camera->generate_ray(ray, coeff, sample.x / sampleBuffer.w, sample.y / sampleBuffer.h) &&
             ++attempts < 20);

      sample_radiance = est_radiance_global_illumination(ray) * coeff;
      sum += sample_radiance;
      
      float illum = sample_radiance.illum();
      sum_illum += illum;
      sum_sq_illum += illum * illum;

      actual_num_samples++;
    }

    i += actual_num_samples;

    // adaptive sampling
    float one_over_n = 1 / (float)i;
    float mean = sum_illum * one_over_n;
    float unbiased_variance = (1 / (float)(i - 1)) * (sum_sq_illum - sum_illum * sum_illum * one_over_n);
    float I = 1.96 * sqrt(unbiased_variance * one_over_n);

    if (I <= maxTolerance * mean) {
      // pixel converged
      break;
    }
  }
  Spectrum temp = sum * (1 / (double)i);
  sampleBuffer.update_pixel(temp, x, y);
  temp_sample = temp;
  sampleCountBuffer[x + y * sampleBuffer.w] = i;
}

void PathTracer::cell_sample(Vector2D loc, vector<Spectrum> *out) {
    size_t ns_area_light_temp = ns_area_light;
    size_t max_ray_depth_temp = max_ray_depth;
    size_t ns_aa_temp = ns_aa;
    ns_aa = 1024;
    max_ray_depth = 10;
    ns_area_light = 4;
    for (int x = -10; x < 11; x ++) {
        for (int y = -10; y < 11; y++) {
            raytrace_pixel(int(loc.x + x), int(loc.y + y));
            out->push_back(temp_sample);
        }
    }


    ns_aa = ns_aa_temp;
    ns_area_light = ns_area_light_temp;
    max_ray_depth = max_ray_depth_temp;

}


} // namespace CGL