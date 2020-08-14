#include <iostream>
#include <fstream>
#include <sstream>

#include "lens.h"
#include "scene/sphere.h"

namespace CGL {

using namespace std;
using namespace SceneObjects;
    
/****** LensElement functions ******/

// refract using the Snell's law
// assuming incoming and normal are unit vectors
// and assume that dot(incoming, normal) < 0
inline static bool refract(const Vector3D &incoming, const Vector3D &normal, double prev_ior, double ior, Vector3D &outgoing) {
  // regularize normal to point in the opposite direction of the incoming ray
  double cos_theta_i = fabs(dot(incoming, normal));

  // https://cs184.eecs.berkeley.edu/sp18/lecture/materials/slide_018
  double k = prev_ior / ior;
  double sin_theta_o_2 = k * k * (1 - cos_theta_i * cos_theta_i);

  // total internal reflection
  if (sin_theta_o_2 > 1) {
    return false;
  }

  // https://graphics.stanford.edu/courses/cs148-10-summer/docs/2006--degreve--reflection_refraction.pdf
  outgoing = k * incoming + (k * cos_theta_i - sqrt(1 - sin_theta_o_2)) * normal;
  outgoing.normalize();

  return true;
}

inline static void reflect(const Vector3D &incoming, const Vector3D &normal, Vector3D &outgoing) {
  outgoing = incoming - 2 * dot(incoming, normal) * normal;
}

double LensElement::pass_through(Ray &r, double &prev_ior, bool internal_reflection, double aperture_override, bool &is_refract) const {
  // Part 1 Task 1: Implement this. It takes r and passes it through this lens element.
  
  is_refract = true;

  // only perform the aperture test
  // if the element is a stop
  if (is_stop()) {
    double t = (center - r.o.z) / r.d.z;
    if (t < 0) return false;
    Vector3D p_intersect = r.at_time(t);
    double actual_aperture = aperture_override != 0 ? aperture_override : aperture;

    if (p_intersect.x * p_intersect.x + p_intersect.y * p_intersect.y > actual_aperture * actual_aperture * 0.25) {
      return 0.0;
    }

#define T(j, n) (p_intersect.x * cos(2 * PI * (j) / (n)) + p_intersect.y * sin(2 * PI * (j) / (n)))

    // max_j [x cos(2 pi j/n) + y sin(2 pi j/n)]
    // max()

    // pentagon
    // if (max({ T(1.0, 5.0), T(2.0, 5.0), T(3.0, 5.0), T(4.0, 5.0), T(5.0, 5.0) }) > actual_aperture * 0.5) {
    //   return 0.0;
    // }

    // hexagon
    // if (max({ T(1.0, 6.0), T(2.0, 6.0), T(3.0, 6.0), T(4.0, 6.0), T(5.0, 6.0), T(6.0, 6.0) }) > actual_aperture * 0.5) {
    //   return 0.0;
    // }

    return 1.0;
  }

  // intersect with the surface (sphere) defined by (center, radius)
  double t1, t2;
  double t;
  Vector3D p_center(0, 0, center);

  if (!Sphere::intersect(radius, p_center, r, t1, t2)) {
    return false;
  }

  // TODO this is a bit messy
  // the surface bends "against" the direction of the ray
  if (r.d.z * radius < 0) {
    // check t2 (the further-away one first)
    if (t2 >= 0) {
      t = t2;
    } else if (t1 >= 0) {
      t = t1;
    } else return false;
  } else {
    if (t1 >= 0) {
      t = t1;
    } else if (t2 >= 0) {
      t = t2;
    } else return false;
  }

  Vector3D p_intersect = r.at_time(t);

  // distance to the z-axis is greater than (aperture / 2)
  if (4 * (p_intersect.x * p_intersect.x + p_intersect.y * p_intersect.y) > aperture * aperture) {
    return false;
  }

  Vector3D normal = p_intersect - p_center;
  normal.normalize();

  // regularize normal to point in the opposite direction of the incoming ray
  if (dot(normal, r.d) > 0) {
    normal = -normal;
  }

  r.o = p_intersect;

  if (internal_reflection) {
    // reflection enabled, there are three possibilities:
    // - total internal reflection
    // - refraction
    // - reflection (weighted by the Schlick's reflection coefficient)
    Vector3D outgoing;

    if (!refract(r.d, normal, prev_ior, ior, outgoing)) {
      // printf("total internal reflection!\n");
      // total internal reflection
      reflect(r.d, normal, outgoing);
      r.d = outgoing;
      is_refract = false;
      return 1.0 / fabs(dot(outgoing, normal));
    } else {
      // Schlick's approximation
      // https://en.wikipedia.org/wiki/Schlick's_approximation
      float r0 = (prev_ior - ior) / (prev_ior + ior);
      float r1 = r0 * r0;
      float r2 = 1 - fabs(dot(r.d, normal));
      float r3 = r2 * r2;
      float R = r1 + (1 - r1) * r3 * r3 * r2;

      if (coin_flip_with_seed(R, r.d.x * 100 + r.d.y * 10000 + r.d.z * 1000000)) {
      // if (coin_flip(R)) {
        // printf("not total reflection! %lf\n", R);
        // reflection
        reflect(r.d, normal, outgoing);
        r.d = outgoing;
        is_refract = false;
        return R / fabs(dot(outgoing, normal));
        // return 1.0;
      } else {
        // refraction
        r.d = outgoing;
        return (1 - R) / fabs(dot(outgoing, normal));
        // return 1.0;
      }
    }
  } else {
    Vector3D outgoing;
    if (!refract(r.d, normal, prev_ior, ior, outgoing)) return 0.0;
    r.d = outgoing;
    return 1.0;
  }
}

/****** Lens functions ******/

void Lens::parse_lens_file(std::string filename) {

  ifstream infile(filename);
  string line;
  double z_coord = 0;
  double z_ap = 0;
  backward_elts.clear();
  elts.clear();
  bool first = true;
  while (getline(infile, line)) {
    if (first) {
      cout << "[Lens] Loading lens file " << line << endl;
      first = false;
    }
    if (line[0] == '#')
      continue;
    stringstream ss(line);
    LensElement lens;
    double offset;
    ss >> lens.radius >> offset >> lens.ior >> lens.aperture;
    lens.center = z_coord;
    if (!lens.radius) {
      z_ap = z_coord;
    }
    z_coord += offset;
    backward_elts.push_back(lens);
  }
  for (int i = backward_elts.size() - 1; i >= 0; --i) {
    backward_elts[i].center = (backward_elts[i].center - z_ap) + backward_elts[i].radius;
    if (!backward_elts[i].ior) backward_elts[i].ior = 1;
    LensElement l = backward_elts[i];
    if (i) l.ior = backward_elts[i-1].ior;
    else l.ior = 1;
    if (!l.ior) l.ior = 1;
    elts.push_back(l);
    if (!l.radius)
      ap_i = elts.size()-1;
    // cout << "Lens element edge first " << (l.center - l.radius) << " " 
    //   << l.radius << " " << l.center << " " << l.ior << " " << l.aperture << endl;
  }
  double c = elts.front().center, r = elts.front().radius, a = elts.front().aperture * .5;
  back_elt = c - (r>0?1:-1) * sqrt(r*r-a*a);
  ap_radius = ap_original = elts[ap_i].aperture;

  // Get infinity and close focus depths, also get focal length.
  set_focus_params();
  // Focus at infinity to start.
  sensor_depth = infinity_focus;
       
}

void Lens::set_focus_params() {

  // Part 1 Task 2: Implement this. 
  // After this function is called, the three variables
  // infinity_focus, near_focus, and focal_length
  // should be set correctly.

  // TODO: the y coordinate is hard-coded now
  double height = ap_original / 2 / 5;

  // estimate the infinity focus
  Ray r0(Vector3D(0, height, -1000000), Vector3D(0, 0, 1));
  double p = trace_backwards(r0, NULL, false);
  assert(p != 0.0 && "failed to estimate the infinity focus");
  assert(r0.d.x > -EPS_D && r0.d.x < EPS_D && "the ray should not deviate from the y-z plane");
  double t0 = (0 - r0.o.y) / r0.d.y;
  assert(t0 > 0 && "the ray should not diverge");
  infinity_focus = r0.o.z + t0 * r0.d.z;

  // estimate the focal length by tracing back from the infinity focus
  // to meet the parallel incoming ray
  focal_length = height / -r0.d.y * r0.d.z;
  assert(focal_length > 0 && "negative focal length");

  // estimate the near focus
  // see https://cs184.eecs.berkeley.edu/sp16/article/19 for explanation
  Vector3D o1(0, 0, -5 * focal_length);
  Vector3D d1 = Vector3D(0, height, 0) - o1;
  d1.normalize();
  Ray r1(o1, d1);
  p = trace_backwards(r1, NULL, false);
  assert(p != 0 && "failed to estimate the near focus");
  assert(r1.d.x > -EPS_D && r1.d.x < EPS_D && "the ray should not deviate from the y-z plane");
  double t1 = (0 - r1.o.y) / r1.d.y;
  assert(t1 > 0 && "the ray should not diverge");
  near_focus = r1.o.z + t1 * r1.d.z;

  cout << "[Lens] Infinity focus depth is " << infinity_focus << endl;
  cout << "[Lens] Close focus depth is " << near_focus << endl;
  cout << "[Lens] True focal length is " << focal_length << endl;
}

double Lens::trace_bidirectionally(bool initial_direction /* true for forward */,
                                   Ray &r, std::vector<Vector3D> *trace, bool internal_reflection, double f_stop) const {
  double current_ior = 1; // air
  r.d.normalize();

  double p = 1;

  double aperture_override = f_stop != 0 ? f_stop * infinity_focus : 0;

  bool is_forward = initial_direction;
  // index in the forward list (complement to get the same surface in the backward list)
  int forward_idx = initial_direction ? 0 : elts.size() - 1;

  int bounces = 0;

  assert(backward_elts.size() == elts.size());

  while (bounces < elts.size() + max_reflect_bounce && forward_idx >= 0 && forward_idx < elts.size()) {
    bool is_refract;

    if (is_forward) {
      // trace forward
      double coeff = elts[forward_idx].pass_through(r, current_ior, internal_reflection, aperture_override, is_refract);
      if (coeff == 0.0) return 0.0;
      p *= coeff;

      if (is_refract) {
        // keep going in the same direction
        current_ior = elts[forward_idx].ior;
        forward_idx++;
      } else {
        // reverse course
        is_forward = false;
        forward_idx--;
        // no change in current_ior
      }
    } else {
      double coeff = backward_elts[elts.size() - forward_idx - 1].pass_through(r, current_ior, internal_reflection, aperture_override, is_refract);
      // printf("backward tracing %lf %lf %d %d %ld\n", p, coeff, forward_idx, bounces, elts.size() + max_reflect_bounce);
      if (coeff == 0.0) return 0.0;
      p *= coeff;

      if (is_refract) {
        // keep going in the same direction
        current_ior = backward_elts[elts.size() - forward_idx - 1].ior;
        forward_idx--; // reduce forward_idx instead of increasing
      } else {
        // reverse course
        is_forward = true;
        forward_idx++;
        // no change in current_ior
      }
    }

    if (trace) trace->push_back(r.o);

    bounces++;
  }

  // successfullly traces through the original direction
  if ((initial_direction && forward_idx == elts.size()) ||
      (!initial_direction && forward_idx == -1)) {
    return p;
  }

  // failed to trace through the original direction or
  // the number of bounces exceeded
  return 0.0;
}

double Lens::trace(Ray &r, std::vector<Vector3D> *trace, bool internal_reflection, double f_stop) const {
  return trace_bidirectionally(true, r, trace, internal_reflection, f_stop);
}

double Lens::trace_backwards(Ray &r, std::vector<Vector3D> *trace, bool internal_reflection, double f_stop) const {
  return trace_bidirectionally(false, r, trace, internal_reflection, f_stop);
}

float Lens::focus_depth(float d) const {

  // Part 1 Task 2: Implement this. Should find the conjugate of a ray
  // starting from the sensor at depth d.

  return 0;
}

Vector3D Lens::back_lens_sample() const {

  // Part 1 Task 2: Implement this. Should return a point randomly sampled
  // on the back element of the lens (the element closest to the sensor)

  LensElement &closest = elts[0];

  UniformGridSampler2D sampler;
  Vector2D sample = sampler.get_sample();
  double r = sample.x;
  double theta = sample.y * 2.0 * PI;

  return Vector3D(
    r * closest.radius * cos(theta),
    r * closest.radius * sin(theta),
    closest.center - closest.radius
  );
}

Vector3D Lens::front_lens_sample(double &area) const {
  LensElement &closest = backward_elts[0];

  double lens_radius = closest.aperture / 2;

  UniformGridSampler2D sampler;
  Vector2D sample = sampler.get_sample();
  double r = sample.x;
  double theta = sample.y * 2.0 * PI;

  area = PI * lens_radius * lens_radius;

  return Vector3D(
    r * lens_radius * cos(theta),
    r * lens_radius * sin(theta),
    closest.center - closest.radius
  );
}

}
