#include "bbox.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>

namespace CGL {

inline double max3(double x, double y, double z) {
  if (x > y && x > z) return x;
  if (y > x && y > z) return y;
  return z;
}

inline double min3(double x, double y, double z) {
  if (x < y && x < z) return x;
  if (y < x && y < z) return y;
  return z;
}

bool BBox::intersect(const Ray& r, double& t_min, double& t_max) const {

  // TODO (Part 2.2):
  // Implement ray - bounding box intersection test
  // If the ray intersected the bouding box within the range given by
  // t0, t1, update t0 and t1 with the new intersection times.

  double t_min_x, t_max_x;
  double t_min_y, t_max_y;
  double t_min_z, t_max_z;

  const Vector3D &min_o_delta = min - r.o;
  const Vector3D &max_o_delta = max - r.o;

  t_min = r.min_t;
  t_max = r.max_t;

  // t range in the x axis
  double one_over_x = 1 / r.d.x;
  if (r.d.x >= 0) {
    t_min_x = min_o_delta.x * one_over_x;
    t_max_x = max_o_delta.x * one_over_x;
  } else {
    t_min_x = max_o_delta.x * one_over_x;
    t_max_x = min_o_delta.x * one_over_x;
  }

  // intersect [t_min, t_max] with [t_min_x, t_max_x]
  t_min = t_min_x < t_min ? t_min : t_min_x;
  t_max = t_max_x < t_max ? t_max_x : t_max;
  if (t_max < t_min) return false;

  // t range in the y axis
  double one_over_y = 1 / r.d.y;
  if (r.d.y >= 0) {
    t_min_y = min_o_delta.y * one_over_y;
    t_max_y = max_o_delta.y * one_over_y;
  } else {
    t_min_y = max_o_delta.y * one_over_y;
    t_max_y = min_o_delta.y * one_over_y;
  }

  // intersect [t_min, t_max] with [t_min_y, t_max_y]
  t_min = t_min_y < t_min ? t_min : t_min_y;
  t_max = t_max_y < t_max ? t_max_y : t_max;
  if (t_max < t_min) return false;

  // t range in the z axis
  double one_over_z = 1 / r.d.z;
  if (r.d.z >= 0) {
    t_min_z = min_o_delta.z * one_over_z;
    t_max_z = max_o_delta.z * one_over_z;
  } else {
    t_min_z = max_o_delta.z * one_over_z;
    t_max_z = min_o_delta.z * one_over_z;
  }

  // intersect [t_min, t_max] with [t_min_z, t_max_z]
  t_min = t_min_z < t_min ? t_min : t_min_z;
  t_max = t_max_z < t_max ? t_max_z : t_max;
  if (t_max < t_min) return false;
  
  return true;
}

void BBox::draw(Color c, float alpha) const {

  glColor4f(c.r, c.g, c.b, alpha);

  // top
  glBegin(GL_LINE_STRIP);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(max.x, max.y, max.z);
  glEnd();

  // bottom
  glBegin(GL_LINE_STRIP);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, min.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glEnd();

  // side
  glBegin(GL_LINES);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(min.x, min.y, max.z);
  glEnd();

}

std::ostream& operator<<(std::ostream& os, const BBox& b) {
  return os << "BBOX(" << b.min << ", " << b.max << ")";
}

} // namespace CGL
