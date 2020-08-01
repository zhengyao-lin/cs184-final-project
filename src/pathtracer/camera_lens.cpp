#include "camera.h"

#include <iostream>
#include <sstream>
#include <fstream>

#include "CGL/misc.h"
#include "CGL/vector2D.h"
#include "CGL/vector3D.h"

using std::cout;
using std::endl;
using std::max;
using std::min;
using std::ifstream;
using std::ofstream;

namespace CGL {

using Collada::CameraInfo;

Ray Camera::generate_ray_for_thin_lens(double x, double y, double rndR, double rndTheta) const {
  // Part 2, Task 4:
  // compute position and direction of ray from the input sensor sample coordinate.
  // Note: use rndR and rndTheta to uniformly sample a unit disk.

  // compute the chef ray
  double h_fov_radian = hFov * PI / 180;
  double v_fov_radian = vFov * PI / 180;

  double w_half = tan(h_fov_radian / 2);
  double h_half = tan(v_fov_radian / 2);

  Vector3D sensor_in_camera(
    w_half - x * w_half * 2,
    h_half - y * h_half * 2,
    1
  );

  // intersect with the focal plane
  Vector3D focal_point = sensor_in_camera * -focalDistance;

  // get the sample point on the lens
  double sample_x = lensRadius * sqrt(rndR) * cos(rndTheta);
  double sample_y = lensRadius * sqrt(rndR) * sin(rndTheta);

  Vector3D position_on_lens = Vector3D(sample_x, sample_y, 0);

  // actual sampled ray direction
  Vector3D sample_ray_direction = focal_point - position_on_lens;

  // transform that to the world space
  Vector3D world_sensor_direction = c2w * sample_ray_direction;
  world_sensor_direction.normalize();

  Ray ray(pos + position_on_lens, world_sensor_direction);
  ray.min_t = nClip;
  ray.max_t = fClip;

  return ray;
}


} // namespace CGL
