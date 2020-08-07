#include "sphere.h"

#include <cmath>

#include "pathtracer/bsdf.h"
#include "util/sphere_drawing.h"

namespace CGL {
namespace SceneObjects {

bool Sphere::test(const Ray &r, double &t1, double &t2) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.

  return true;
}

bool Sphere::intersect(
  double radius,
  const Vector3D &center,
  const Ray &r,
  double &t1,
  double &t2
) {
  Vector3D o_to_center = r.o - center;

  double a = dot(r.d, r.d);
  double b = 2 * dot(o_to_center, r.d);
  double c = dot(o_to_center, o_to_center) - radius * radius;

  double delta = b * b - 4 * a * c;
  if (delta < 0) return false;

  t1 = (-b - sqrt(delta)) / (2 * a);
  t2 = (-b + sqrt(delta)) / (2 * a);
  
  return true;
}

bool Sphere::has_intersection(const Ray &ray) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.

  double t1, t2;
  if (!intersect(r, o, ray, t1, t2)) return false;

  if (t1 >= ray.min_t && t1 <= ray.max_t) {
    ray.max_t = t1;
    return true;
  }

  if (t2 >= ray.min_t && t2 <= ray.max_t) {
    ray.max_t = t2;
    return true;
  }

  return false;
}

bool Sphere::intersect(const Ray &ray, Intersection *isect) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.

  double t1, t2;
  double t;
  if (!intersect(r, o, ray, t1, t2)) return false;

  if (t1 >= ray.min_t && t1 <= ray.max_t) {
    t = t1;
  } else if (t2 >= ray.min_t && t2 <= ray.max_t) {
    t = t2;
  } else return false;

  Vector3D intersection_pt = ray.at_time(t);
  Vector3D normal = intersection_pt - o;
  normal.normalize();

  ray.max_t = t;
  isect->t = t;
  isect->n = normal;
  isect->primitive = this;
  isect->bsdf = get_bsdf();

  return true;
}

void Sphere::draw(const Color &c, float alpha) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color &c, float alpha) const {
  // Misc::draw_sphere_opengl(o, r, c);
}

} // namespace SceneObjects
} // namespace CGL
