/**
 * Lens abstractions
 * This file is mostly copied from https://github.com/CS184-sp16/asst4_lenssim
 */

#ifndef CGL_LENS_H
#define CGL_LENS_H

#include <string>
#include <vector>
#include <random>


#include "pathtracer/ray.h"

namespace CGL {

// abstraction for a surface in the lens system

    struct Dispersion {
    public:
        Dispersion(double a, double b, double c, double d, double e, double f, double g) {
            B1 = a;
            B2 = b;
            B3 = c;
            C1 = d;
            C2 = e;
            C3 = f;
            n = g;
        }
        double B1;
        double B2;
        double B3;
        double C1;
        double C2;
        double C3;
        double n;
    };

struct LensElement {
public:
  bool pass_through(Ray &r, double &prev_ior, double aperture_override) const;
  bool is_stop() const { return radius == 0; }

  double center; // the z coordinate of the center of the circle/sphere
  double radius;
  double ior;
  double aperture;
  int dis;
  double B1;
  double B2;
  double B3;
  double C1;
  double C2;
  double C3;
  //int m_n;
  //std::default_random_engine ran_x;

  void set_dis (Dispersion d) {
      B1 = d.B1;
      B2 = d.B2;
      B3 = d.B3;
      C1 = d.C1;
      C2 = d.C2;
      C3 = d.C3;
      ior = d.n;
  }
  //Dispersion dispersion;
};


struct Lens {
  Lens(std::string filename) { parse_lens_file(filename); }

  void parse_lens_file(std::string filename);

  void set_focus_params();

  bool trace(Ray &r, std::vector<Vector3D> *trace = NULL, double f_stop = 0.0) const;
  bool trace_backwards(Ray &r, std::vector<Vector3D> *trace = NULL, double f_stop = 0.0) const;

  float focus_depth(float d) const;

  Vector3D back_lens_sample() const;

  mutable std::vector<LensElement> elts;
  mutable std::vector<LensElement> backward_elts;

  double back_elt, infinity_focus, near_focus, focal_length;
  double ap_radius, ap_original;
  size_t ap_i;
  double sensor_depth;

  //std::vector<Dispersion> dis;



};
}

#endif
