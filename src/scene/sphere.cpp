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

inline bool ray_sphere_intersection(
  double radius,
  const Vector3D &center,
  const Ray &r,
  double &t
) {
  Vector3D o_to_center = r.o - center;

  double a = dot(r.d, r.d);
  double b = 2 * dot(o_to_center, r.d);
  double c = dot(o_to_center, o_to_center) - radius * radius;

  double delta = b * b - 4 * a * c;
  if (delta < 0) return false;

  t = (-b - sqrt(delta)) / (2 * a);
  if (t >= r.min_t && t <= r.max_t) return true;

  t = (-b + sqrt(delta)) / (2 * a);
  if (t >= r.min_t && t <= r.max_t) return true;

  return false;
}

bool Sphere::has_intersection(const Ray &ray) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.

  double t;
  if (!ray_sphere_intersection(r, o, ray, t)) return false;
  ray.max_t = t;
  return true;
}

bool Sphere::intersect(const Ray &ray, Intersection *isect) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.

  double t;
  if (!ray_sphere_intersection(r, o, ray, t)) return false;

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
