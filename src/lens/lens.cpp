#include <iostream>
#include <fstream>
#include <sstream>

#include "lens.h"
#include "scene/sphere.h"

namespace CGL {

using namespace std;
using namespace SceneObjects;
    
/****** LensElement functions ******/

bool LensElement::pass_through(Ray &r, double &prev_ior, double aperture_override) const {
  // Part 1 Task 1: Implement this. It takes r and passes it through this lens element.
  
  // only perform the aperture test
  // if the element is a stop
  if (is_stop()) {
    double t = (center - r.o.z) / r.d.z;
    if (t < 0) return false;
    Vector3D p_intersect = r.at_time(t);
    double actual_aperture = aperture_override != 0 ? aperture_override : aperture;
    if (4 * (p_intersect.x * p_intersect.x + p_intersect.y * p_intersect.y) > actual_aperture * actual_aperture) {
      return false;
    }
    return true;
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
  double real_ior = 1;
  //cout<<dis<<endl;
  double wavelength = 0.5876;
  if(dis != 0) {
      std::default_random_engine ran_x;
      unsigned seed1 = std::chrono::system_clock::now().time_since_epoch().count();
      ran_x.seed(seed1);
      std::normal_distribution<double> distributionR(0.6, 0.025);
      std::normal_distribution<double> distributionG(0.55, 0.025);
      std::normal_distribution<double> distributionB(0.45, 0.015);
      //cout<<r.color<<endl;
      switch (r.color) {
          case 1:
              wavelength = distributionR(ran_x);
              //wavelength = 0.6;
              //cout<<wavelength<<endl;
              break;
          case 2:
              wavelength = distributionG(ran_x);
              //wavelength = 0.55;
              break;
          case 3:
              wavelength = distributionB(ran_x);
              //wavelength = 0.45;
              break;
      }

      //double wavelength = 0;
      double w2 = wavelength * wavelength;
      //cout<<w2<<endl;
      real_ior = sqrt(1 + ((B1 * w2) / (w2 - C1)) + ((B2 * w2) / (w2 - C2)) + ((B3 * w2) / (w2 - C3)));

      /*if(ior == 1) {
          cout << "#real: " << real_ior << "ior: " << ior << endl;
      }*/
  } else {
      real_ior = ior;
  }
        //real_ior = ior;
  //cout<<real_ior<<dis<<endl;
  double k = prev_ior / real_ior;
  double sin_theta_o_2 = k * k * (1 - cos_theta_i * cos_theta_i);

  // total internal reflection
  if (sin_theta_o_2 > 1) {
    return false;
  }

  Vector3D old_direction = r.d;

  // https://graphics.stanford.edu/courses/cs148-10-summer/docs/2006--degreve--reflection_refraction.pdf
  r.d = k * r.d + (k * cos_theta_i - sqrt(1 - sin_theta_o_2)) * normal;
  r.d.normalize();
  
  r.o = p_intersect;

  prev_ior = real_ior;
  r.wavelength = wavelength;

  // printf("refract from (%lf, %lf, %lf) to (%lf, %lf, %lf)\n", old_direction.x, old_direction.y, old_direction.z, r.d.x, r.d.y, r.d.z);
  return true;

}

/****** Lens functions ******/

void Lens::parse_lens_file(std::string filename) {

    Dispersion BK7 = Dispersion(1.03961212, 0.231792344, 1.01046945, 0.00600069867, 0.0200179144, 103.560653, 1.5168);
    Dispersion SCHOTT_MP = Dispersion(1.3182408, 0.0244, 1.08915181, 0.00879, 0.0609, 110, 1.5424);
    Dispersion SCHOTT_SF = Dispersion(1.55912923, 0.284246288, 0.968842926, 0.0121481001, 0.0534549042, 112.174809, 1.7174);
    Dispersion OHARA_NBM = Dispersion(1.37023101, 0.177665568, 1.30515471, 0.00871920342, 0.0405725552, 112.703058, 1.6134);
    Dispersion SCHOTT_SK = Dispersion(1.28189012, 0.257738258, 0.96818604, 0.0072719164, 0.0242823527, 110.377773, 1.6074);
    Dispersion OHARA_NPH = Dispersion(1.75156623, 0.364006304, 2.47874141, 0.0135004681, 0.0668245147, 170.756006, 1.8081);
    Dispersion SF11 = Dispersion(1.73759695, 0.313747346, 1.89878101, 0.013188707, 0.0623068142, 155.23629, 1.7847);

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
    //double temp;
    lens.dis = 0;
    ss >> lens.radius >> offset >> lens.ior >> lens.aperture >> lens.dis;
    //cout<<temp<<endl;
    lens.center = z_coord;
    if (!lens.radius) {
      z_ap = z_coord;
    }
    z_coord += offset;
      switch (lens.dis) {
          case 0:
              break;
          case 1:
              lens.set_dis(BK7);
              break;
          case 2:
              lens.set_dis(SCHOTT_MP);
              break;
          case 3:
              lens.set_dis(SCHOTT_SF);
              break;
          case 4:
              lens.set_dis(OHARA_NBM);
              break;
          case 5:
              lens.set_dis(SCHOTT_SK);
              break;
          case 6:
              lens.set_dis(OHARA_NPH);
              break;
          case 7:
              lens.set_dis(SF11);
              break;

      }
      //lens.m_n =

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
  //sensor_depth = infinity_focus;
        sensor_depth = 46.2;
       
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
  assert(trace_backwards(r0, NULL) && "failed to estimate the infinity focus");
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

  assert(trace_backwards(r1, NULL) && "failed to estimate the near focus");
  assert(r1.d.x > -EPS_D && r1.d.x < EPS_D && "the ray should not deviate from the y-z plane");
  double t1 = (0 - r1.o.y) / r1.d.y;
  assert(t1 > 0 && "the ray should not diverge");
  near_focus = r1.o.z + t1 * r1.d.z;

  cout << "[Lens] Infinity focus depth is " << infinity_focus << endl;
  cout << "[Lens] Close focus depth is " << near_focus << endl;
  cout << "[Lens] True focal length is " << focal_length << endl;


}

bool Lens::trace(Ray &r, std::vector<Vector3D> *trace, double f_stop) const {
  double current_ior = 1; // air
  r.d.normalize();

  for (auto &elem: elts) {
    if (!elem.pass_through(r, current_ior, f_stop != 0 ? f_stop * infinity_focus : 0)) return false;
    //current_ior = elem.ior;
    if (trace) trace->push_back(r.o);
  }
  return true;
}

bool Lens::trace_backwards(Ray &r, std::vector<Vector3D> *trace, double f_stop) const {
  double current_ior = 1; // air
  r.d.normalize();

  for (auto &elem: backward_elts) {
    // printf("%lf => %lf (%lf %lf)\n", current_ior, elem.ior, elem.center, elem.radius);
    if (!elem.pass_through(r, current_ior, f_stop != 0 ? f_stop * infinity_focus : 0)) return false;
    current_ior = elem.ior;
    if (trace) trace->push_back(r.o);
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

}
