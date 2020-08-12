/**
 * Lens abstractions
 * This file is mostly copied from https://github.com/CS184-sp16/asst4_lenssim
 */

#ifndef CGL_LENS_H
#define CGL_LENS_H

#include <string>
#include <vector>

#include "pathtracer/ray.h"

namespace CGL {

// abstraction for a surface in the lens system
struct LensElement {
public:
  // returns the coefficient that should be added to the sample
  // 0.0 if the ray does not pass through
  double pass_through(Ray &r, double &prev_ior, bool internal_reflection, double aperture_override, bool &is_refract) const;
  bool is_stop() const { return radius == 0; }

  double center; // the z coordinate of the center of the circle/sphere
  double radius;
  double ior;
  double aperture;
};

struct Lens {
  Lens(std::string filename) { parse_lens_file(filename); }

  void parse_lens_file(std::string filename);

  void set_focus_params();

  double trace_bidirectionally(bool initial_direction, Ray &r, std::vector<Vector3D> *trace, bool internal_reflection, double f_stop) const;
  double trace(Ray &r, std::vector<Vector3D> *trace = NULL, bool internal_reflection = false, double f_stop = 0.0) const;
  double trace_backwards(Ray &r, std::vector<Vector3D> *trace = NULL, bool internal_reflection = false, double f_stop = 0.0) const;

  float focus_depth(float d) const;

  Vector3D back_lens_sample() const;
  Vector3D front_lens_sample() const;

  mutable std::vector<LensElement> elts;
  mutable std::vector<LensElement> backward_elts;

  double back_elt, infinity_focus, near_focus, focal_length;
  double ap_radius, ap_original;
  size_t ap_i;
  double sensor_depth;

  int max_reflect_bounce = 100;
};
}

#endif
