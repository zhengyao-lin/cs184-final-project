#include "bsdf.h"

#include <algorithm>
#include <iostream>
#include <utility>


using std::max;
using std::min;
using std::swap;

namespace CGL {

// Mirror BSDF //

Spectrum MirrorBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum MirrorBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  // TODO:
  // Implement MirrorBSDF
  *pdf = 1;
  reflect(wo, wi);
  return reflectance / abs_cos_theta(*wi);
}

// Microfacet BSDF //

double MicrofacetBSDF::G(const Vector3D& wo, const Vector3D& wi) {
  return 1.0 / (1.0 + Lambda(wi) + Lambda(wo));
}

double MicrofacetBSDF::D(const Vector3D& h) {
  // TODO: proj3-2, part 3
  // Compute Beckmann normal distribution function (NDF) here.
  // You will need the roughness alpha.
  // return std::pow(cos_theta(h), 100.0);;

  double cos_theta_h = h.z;
  double sin_theta_h = sqrt(1 - cos_theta_h * cos_theta_h);
  double tan_theta_h = sin_theta_h / cos_theta_h;
  double alpha_2 = alpha * alpha;
  double cos_theta_h_2 = cos_theta_h * cos_theta_h;

  return exp(-(tan_theta_h * tan_theta_h) / alpha_2)
         / (PI * alpha_2 * cos_theta_h_2 * cos_theta_h_2);
}

inline double fresnel_channel(double eta, double k, double cos_theta_i) {
  double r0 = eta * eta + k * k;
  double cos_theta_i_2 = cos_theta_i * cos_theta_i;
  double rs = (r0 - 2 * eta * cos_theta_i + cos_theta_i_2)
            / (r0 + 2 * eta * cos_theta_i + cos_theta_i_2);
  double rp = (r0 * cos_theta_i_2 - 2 * eta * cos_theta_i + 1)
            / (r0 * cos_theta_i_2 + 2 * eta * cos_theta_i + 1);
  return (rs + rp) * 0.5;
}

Spectrum MicrofacetBSDF::F(const Vector3D& wi) {
  // TODO: proj3-2, part 3
  // Compute Fresnel term for reflection on dielectric-conductor interface.
  // You will need both eta and etaK, both of which are Spectrum.

  double cos_theta_i = cos_theta(wi);

  return Spectrum(
    fresnel_channel(eta.x, k.x, cos_theta_i),
    fresnel_channel(eta.y, k.y, cos_theta_i),
    fresnel_channel(eta.z, k.z, cos_theta_i)
  );
}

Spectrum MicrofacetBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  // TODO: proj3-2, part 3
  // Implement microfacet model here.

  if (wo.z <= 0 || wi.z <= 0) return Spectrum();

  Vector3D h = wo +  wi;
  h.normalize();
  return F(wi) * G(wo, wi) * D(h) / (4 * wo.z * wi.z); // normal = (0, 0, 1)
}

Spectrum MicrofacetBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  // TODO: proj3-2, part 3
  // *Importance* sample Beckmann normal distribution function (NDF) here.
  // Note: You should fill in the sampled direction *wi and the corresponding *pdf,
  //       and return the sampled BRDF value.
  // *wi = cosineHemisphereSampler.get_sample(pdf); //placeholder
  // return MicrofacetBSDF::f(wo, *wi);

  const Vector2D& r = sampler.get_sample();

  double alpha_2 = alpha * alpha;

  double theta_h = atan(sqrt(-(alpha_2 * log(1 - r.x))));
  double phi_h = 2 * PI * r.y;

  double sin_theta_h = sin(theta_h);
  double cos_theta_h = cos(theta_h);
  double tan_theta_h = tan(theta_h);

  Vector3D h(
    sin_theta_h * cos(phi_h),
    sin_theta_h * sin(phi_h),
    cos_theta_h
  );
  h.normalize();

  // reflect wo wrt h
  *wi = -wo + 2 * dot(wo, h) * h;

  if (wi->z <= 0) return Spectrum();

  double p_theta_h = exp(-tan_theta_h * tan_theta_h / alpha_2) * (2 * sin_theta_h)
                   / (alpha_2 * cos_theta_h * cos_theta_h * cos_theta_h);
  double p_phi_h = 0.5 / PI;

  double p_wi = p_theta_h * p_phi_h / (sin_theta_h * 4 * dot(*wi, h));

  *pdf = p_wi;

  return MicrofacetBSDF::f(wo, *wi);
}

// Refraction BSDF //

Spectrum RefractionBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum RefractionBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  // TODO:
  // Implement RefractionBSDF
  return Spectrum();
}

// Glass BSDF //

Spectrum GlassBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum GlassBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  // TODO:
  // Compute Fresnel coefficient and use it as the probability of reflection
  // - Fundamentals of Computer Graphics page 305

  if (!refract(wo, wi, ior)) {
    reflect(wo, wi);
    *pdf = 1;
    return reflectance / abs_cos_theta(*wi);
  }

  // Schlick's approximation
  // https://en.wikipedia.org/wiki/Schlick's_approximation
  float r0 = (1 - ior) / (1 + ior);
  float r1 = r0 * r0;
  float r2 = 1 - abs_cos_theta(wo);
  float r3 = r2 * r2;
  float r = r1 + (1 - r1) * r3 * r3 * r2;

  if (coin_flip(r)) {
    reflect(wo, wi);
    *pdf = r;
    return r * reflectance / abs_cos_theta(*wi);
  } else {
    *pdf = 1 - r;

    float eta_i_over_eta_o;
    if (wo.z > 0) {
      // entering from vacuum
      eta_i_over_eta_o = ior;
    } else {
      // exiting from vacuum
      eta_i_over_eta_o = 1 / ior;
    }

    return (1 - r) * transmittance / abs_cos_theta(*wi) / (eta_i_over_eta_o * eta_i_over_eta_o);
  }
}

void BSDF::reflect(const Vector3D& wo, Vector3D* wi) {
  // TODO:
  // Implement reflection of wo about normal (0,0,1) and store result in wi.
  *wi = Vector3D(-wo.x, -wo.y, 2 * wo.z - wo.z);
}

bool BSDF::refract(const Vector3D& wo, Vector3D* wi, float ior) {
  // TODO:
  // Use Snell's Law to refract wo surface and store result ray in wi.
  // Return false if refraction does not occur due to total internal reflection
  // and true otherwise. When dot(wo,n) is positive, then wo corresponds to a
  // ray entering the surface through vacuum.

  float eta_i_over_eta_o;
  
  if (wo.z > 0) {
    // entering from vacuum
    eta_i_over_eta_o = 1 / ior;
  } else {
    // exiting from vacuum
    eta_i_over_eta_o = ior;
  }

  double cos_theta_o = cos_theta(wo);
  double k = eta_i_over_eta_o * eta_i_over_eta_o * (1 - cos_theta_o * cos_theta_o);

  if (1 < k) {
    return false;
  }

  // because wo.x = sin theta * cos phi
  // and wo.y = sin theta * sin phi
  *wi = Vector3D(
    -wo.x * eta_i_over_eta_o,
    -wo.y * eta_i_over_eta_o,
    wo.z > 0 ? -sqrt(1 - k) : sqrt(1 - k)
  );

  return true;
}

} // namespace CGL
