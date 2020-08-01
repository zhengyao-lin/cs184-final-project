/**
 * Lens abstractions
 * This file is mostly copied from https://github.com/CS184-sp16/asst4_lenssim
 */

#ifndef CGL_LENS_H
#define CGL_LENS_H

#include <string>

#include "pathtracer/ray.h"

namespace CGL {

struct LensElement {
public:
  bool pass_through(Ray &r, double &prev_ior) const;
  double center;
  double radius;
  double ior;
  double aperture;
private:
  bool intersect(const Ray &r, Vector3D *hit_p) const;
  bool refract(Ray& r, const Vector3D& hit_p, const double& prev_ior) const;
};

struct Lens {
  Lens(std::string filename) { parse_lens_file(filename); }

  void parse_lens_file(std::string filename);

  void set_focus_params();

  bool trace(Ray &r, std::vector<Vector3D> *trace = NULL) const;
  bool trace_backwards(Ray &r, std::vector<Vector3D> *trace = NULL) const;

  float focus_depth(float d) const;

  Vector3D back_lens_sample() const;

  mutable std::vector<LensElement> elts;
  double back_elt, infinity_focus, near_focus, focal_length;
  double ap_radius, ap_original;
  size_t ap_i;
  double sensor_depth;
};
}

#endif
