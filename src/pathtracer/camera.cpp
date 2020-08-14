#include "camera.h"

#include <iostream>
#include <sstream>
#include <fstream>
#include <filesystem>
#include <cassert>

#include "CGL/misc.h"
#include "CGL/vector3D.h"

using std::cout;
using std::endl;
using std::max;
using std::min;
using std::ifstream;
using std::ofstream;

// TODO: hard-coded right now
#define LENSES_DIR "lenses"

namespace CGL {

using Collada::CameraInfo;

Camera::Camera() {
  // load all lenses
  for (const auto &entry : std::filesystem::directory_iterator(LENSES_DIR)) {
    lenses.emplace_back(entry.path());
  }
}

/**
 * Sets the field of view to match screen screenW/H.
 * NOTE: data and screenW/H will almost certainly disagree about the aspect
 *       ratio. screenW/H are treated as the source of truth, and the field
 *       of view is expanded along whichever dimension is too narrow.
 * NOTE2: info.hFov and info.vFov are expected to be in DEGREES.
 */
void Camera::configure(const CameraInfo& info, size_t screenW, size_t screenH) {
  this->screenW = screenW;
  this->screenH = screenH;
  nClip = info.nClip;
  fClip = info.fClip;
  hFov = info.hFov;
  vFov = info.vFov;

  double ar1 = tan(radians(hFov) / 2) / tan(radians(vFov) / 2);
  ar = static_cast<double>(screenW) / screenH;
  if (ar1 < ar) {
    // hFov is too small
    hFov = 2 * degrees(atan(tan(radians(vFov) / 2) * ar));
  } else if (ar1 > ar) {
    // vFov is too small
    vFov = 2 * degrees(atan(tan(radians(hFov) / 2) / ar));
  }
  screenDist = ((double) screenH) / (2.0 * tan(radians(vFov) / 2));
}

/**
 * This function places the camera at the target position and sets the arguments.
 * Phi and theta are in RADIANS.
 */
void Camera::place(const Vector3D& targetPos, const double phi,
                   const double theta, const double r, const double minR,
                   const double maxR) {
  double r_ = min(max(r, minR), maxR);
  double phi_ = (sin(phi) == 0) ? (phi + EPS_F) : phi;
  this->targetPos = targetPos;
  this->phi = phi_;
  this->theta = theta;
  this->r = r_;
  this->minR = minR;
  this->maxR = maxR;
  compute_position();
}

/**
 * Copies just placement data from the other camera.
 */
void Camera::copy_placement(const Camera& other) {
  pos = other.pos;
  targetPos = other.targetPos;
  phi = other.phi;
  theta = other.theta;
  minR = other.minR;
  maxR = other.maxR;
  c2w = other.c2w;
}

/**
 * Updates the screen size to be the specified size, keeping screenDist
 * constant.
 */
void Camera::set_screen_size(const size_t screenW, const size_t screenH) {
  this->screenW = screenW;
  this->screenH = screenH;
  ar = 1.0 * screenW / screenH;
  hFov = 2 * degrees(atan(((double) screenW) / (2 * screenDist)));
  vFov = 2 * degrees(atan(((double) screenH) / (2 * screenDist)));
}

/**
 * Translates the camera such that a value at distance d directly in front of
 * the camera moves by (dx, dy). Note that dx and dy are in screen coordinates,
 * while d is in world-space coordinates (like pos/dir/up).
 */
void Camera::move_by(const double dx, const double dy, const double d) {
  const double scaleFactor = d / screenDist;
  const Vector3D& displacement =
    c2w[0] * (dx * scaleFactor) + c2w[1] * (dy * scaleFactor);
  pos += displacement;
  targetPos += displacement;
}

/**
 * Move the specified amount along the view axis.
 */
void Camera::move_forward(const double dist) {
  double newR = max(r - dist, minR), maxR;
  pos = targetPos + ((pos - targetPos) * (newR / r));
  r = newR;
}

/**
 * Rotate by the specified amount around the target.
 */
void Camera::rotate_by(const double dPhi, const double dTheta) {
  phi = clamp(phi + dPhi, 0.0, (double) PI);
  theta += dTheta;
  compute_position();
}

/**
 * Computes the camera position, basis vectors, and the view matrix
 */
void Camera::compute_position() {
  double sinPhi = sin(phi);
  if (sinPhi == 0) {
    phi += EPS_F;
    sinPhi = sin(phi);
  }
  const Vector3D dirToCamera(r * sinPhi * sin(theta),
                             r * cos(phi),
                             r * sinPhi * cos(theta));
  pos = targetPos + dirToCamera;
  Vector3D upVec(0, sinPhi > 0 ? 1 : -1, 0);
  Vector3D screenXDir = cross(upVec, dirToCamera);
  screenXDir.normalize();
  Vector3D screenYDir = cross(dirToCamera, screenXDir);
  screenYDir.normalize();

  c2w[0] = screenXDir;
  c2w[1] = screenYDir;
  c2w[2] = dirToCamera.unit();   // camera's view direction is the
                                 // opposite of of dirToCamera, so
                                 // directly using dirToCamera as
                                 // column 2 of the matrix takes [0 0 -1]
                                 // to the world space view direction
}

/**
 * Stores the camera settings into a file
 */
void Camera::dump_settings(string filename) {
  ofstream file(filename);
  file << hFov << " " << vFov << " " << ar << " " << nClip << " " << fClip << endl;
  for (int i = 0; i < 3; ++i)
    file << pos[i] << " ";
  for (int i = 0; i < 3; ++i)
    file << targetPos[i] << " ";
  file << endl;
  file << phi << " " << theta << " " << r << " " << minR << " " << maxR << endl;
  for (int i = 0; i < 9; ++i)
    file << c2w(i/3, i%3) << " ";
  file << endl;
  file << screenW << " " << screenH << " " << screenDist << endl;
  cout << "[Camera] Dumped settings to " << filename << endl;
}

/**
 * Loads the camera settings from a file
 */
void Camera::load_settings(string filename) {
  ifstream file(filename);

  file >> hFov >> vFov >> ar >> nClip >> fClip;
  for (int i = 0; i < 3; ++i)
    file >> pos[i];
  for (int i = 0; i < 3; ++i)
    file >> targetPos[i];
  file >> phi >> theta >> r >> minR >> maxR;
  for (int i = 0; i < 9; ++i)
    file >> c2w(i/3, i%3);
  file >> screenW >> screenH >> screenDist;
  cout << "[Camera] Loaded settings from " << filename << endl;
}

/**
 * Returns a world-space ray from the camera that corresponds to a
 * ray exiting the camera that deposits light on the sensor plane,
 * positioned in normalized image space given by (x,y).  x and y are
 * provided in the normalized coordinate space of the image / output
 * device.  For example (0.5, 0.5) corresponds to the middle of the screen.
 *
 * \param x x-coordinate of the pixel in normalized image space
 * \param y y-coordinate of the pixel in normalized image space
 */
bool Camera::generate_ray(Ray &ray, double &coeff, double x, double y) const {
  switch (model) {
    case CameraModel::PINHOLE:
      return generate_ray_for_pinhole(ray, coeff, x, y);
    case CameraModel::THIN_LENS:
      return generate_ray_for_thin_lens(ray, coeff, x, y);
    case CameraModel::COMPOUND_LENS:
      return generate_ray_for_compound_lens(ray, coeff, x, y);
  }
  assert(false && "unexpected camera model");
}

// generate a ray for the pinhole model
bool Camera::generate_ray_for_pinhole(Ray &ray, double &coeff, double x, double y) const {
  // TODO (Part 1.2):
  // compute position of the input sensor sample coordinate on the
  // canonical sensor plane one unit away from the pinhole.
  // Note: hFov and vFov are in degrees.
  
  // get the position at the sensor in camera space
  double h_fov_radian = hFov * PI / 180;
  double v_fov_radian = vFov * PI / 180;

  double w_half = tan(h_fov_radian / 2);
  double h_half = tan(v_fov_radian / 2);

  Vector3D sensor_in_camera(
    -w_half + x * w_half * 2,
    -h_half + y * h_half * 2,
    -1
  );

  // transform that to the world space
  Vector3D world_sensor_direction = c2w * sensor_in_camera;
  world_sensor_direction.normalize();

  ray = Ray(pos, world_sensor_direction);
  ray.min_t = nClip;
  ray.max_t = fClip;
  coeff = 1;
  return true;
}

bool Camera::generate_ray_for_thin_lens(Ray &ray, double &coeff, double x, double y) const {
  // Part 2, Task 4:
  // compute position and direction of ray from the input sensor sample coordinate.
  // Note: use rndR and rndTheta to uniformly sample a unit disk.

  Vector2D lens_sample = gridSampler.get_sample();
  double rndR = lens_sample.x;
  double rndTheta = lens_sample.y * 2.0 * PI;

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

  ray = Ray(pos + position_on_lens, world_sensor_direction);
  ray.min_t = nClip;
  ray.max_t = fClip;
  coeff = 1;
  return true;
}

bool Camera::generate_ray_for_compound_lens(Ray &ray, double &coeff, double x, double y) const {
  const Lens *lens = get_current_lens();
  assert(lens && "no current lens");

  // sample a point on the rearest lens
  Vector3D sample = lens->back_lens_sample();

  // compute the chef ray
  double h_fov_radian = hFov * PI / 180;
  double v_fov_radian = vFov * PI / 180;

  double w_half = tan(h_fov_radian / 2) * lens->sensor_depth;
  double h_half = tan(v_fov_radian / 2) * lens->sensor_depth;

  // generate a ray from x, y on the sensor plane (at sensor_depth)
  Vector3D sensor_position(
    w_half - x * w_half * 2,
    h_half - y * h_half * 2,
    lens->sensor_depth
  );

  Vector3D direction = sample - sensor_position;
  direction.normalize();

  Ray sensor_ray(sensor_position, direction);

  // (cos theta)^4 for an unbiased estimate of the radiance
  coeff = direction.z;
  coeff *= coeff;
  coeff *= coeff;

  // trace the sensor ray through the compound lens
  double p = lens->trace(sensor_ray, NULL, false, lensRadius /* using as f-stop here */);
  if (p == 0.0) {
    return false;
  }

  coeff /= p;

  Vector3D world_direction = c2w * sensor_ray.d;
  world_direction.normalize();

  ray = Ray(pos + c2w * sensor_ray.o * 0.01 /* from mm to meters, but also scale up a bit to shrink the depth of field */,
            world_direction);
  ray.min_t = nClip;
  ray.max_t = fClip;

  // printf("sampled ray: (%lf, %lf, %lf) -> (%lf, %lf, %lf)\n",
  //   ray.o.x, ray.o.y, ray.o.z,
  //   ray.d.x, ray.d.y, ray.d.z
  // );

  return true;
}

} // namespace CGL
