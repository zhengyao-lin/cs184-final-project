#include <iostream>
#include <fstream>
#include <sstream>

#include "lens.h"
#include "scene/sphere.h"

namespace CGL {

using namespace std;
using namespace SceneObjects;
    
/****** LensElement functions ******/

bool LensElement::pass_through(Ray &r, double &prev_ior) const {
  // Part 1 Task 1: Implement this. It takes r and passes it through this lens element.
  
  // only perform the aperture test
  // if the element is a stop
  if (is_stop()) {
    double t = (center - r.o.z) / r.d.z;
    if (t < 0) return false;
    Vector3D p_intersect = r.at_time(t);
    if (4 * p_intersect.x * p_intersect.x + p_intersect.y * p_intersect.y > aperture * aperture) {
      return false;
    }
    return true;
  }

  // intersect with the surface (sphere) defined by (center, radius)
  double t;
  Vector3D p_center(0, 0, center);

  r.min_t = 0;
  r.max_t = INF_D;

  if (!Sphere::intersect(radius, p_center, r, t)) {
    return false;
  }

  Vector3D p_intersect = r.at_time(t);

  // distance to the z-axis is greater than (aperture / 2)
  if (4 * (p_intersect.x * p_intersect.x + p_intersect.y * p_intersect.y) > aperture * aperture) {
    return false;
  }

  // refract using the Snell's law
  Vector3D normal = p_intersect - p_center;
  normal.normalize();

  // regularize normal to point in the opposite direction of the incoming ray
  if (dot(normal, r.d) > 0) {
    normal = -normal;
  }

  // assuming r.d is normalized
  double cos_theta_i = fabs(dot(r.d, normal));

  // https://cs184.eecs.berkeley.edu/sp18/lecture/materials/slide_018
  double k = prev_ior / ior;
  double sin_theta_o_2 = k * k * (1 - cos_theta_i * cos_theta_i);

  // total internal reflection
  if (sin_theta_o_2 > 1) {
    return false;
  }

  // https://graphics.stanford.edu/courses/cs148-10-summer/docs/2006--degreve--reflection_refraction.pdf
  r.d = k * r.d + (k * cos_theta_i - (1 - sin_theta_o_2)) * normal;
  r.d.normalize();
  
  r.o = p_intersect;

  return true;
}

/****** Lens functions ******/

void Lens::parse_lens_file(std::string filename) {

  ifstream infile(filename);
  string line;
  double z_coord = 0;
  double z_ap;
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

  cout << "[Lens] Infinity focus depth is " << infinity_focus << endl;
  cout << "[Lens] Close focus depth is " << near_focus << endl;
  cout << "[Lens] True focal length is " << focal_length << endl;
}




bool Lens::trace(Ray &r, std::vector<Vector3D> *trace) const {
  double current_ior = 1; // air

  for (auto &elem: elts) {
    if (!elem.pass_through(r, current_ior)) return false;
    current_ior = elem.ior;
    trace->push_back(r.o);
  }

  return true;
}

bool Lens::trace_backwards(Ray &r, std::vector<Vector3D> *trace) const {
  double current_ior = 1; // air

  for (auto &elem: backward_elts) {
    if (!elem.pass_through(r, current_ior)) return false;
    current_ior = elem.ior;
    trace->push_back(r.o);
  }

  return true;
}

float Lens::focus_depth(float d) const {

  // Part 1 Task 2: Implement this. Should find the conjugate of a ray
  // starting from the sensor at depth d.

  return 0;
}

Vector3D Lens::back_lens_sample() const {

  // Part 1 Task 2: Implement this. Should return a point randomly sampled
  // on the back element of the lens (the element closest to the sensor)

  return Vector3D();

}

}
