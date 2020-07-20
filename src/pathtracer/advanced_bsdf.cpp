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
  return std::pow(cos_theta(h), 100.0);;
}

Spectrum MicrofacetBSDF::F(const Vector3D& wi) {
  // TODO: proj3-2, part 3
  // Compute Fresnel term for reflection on dielectric-conductor interface.
  // You will need both eta and etaK, both of which are Spectrum.
  return Spectrum();
}

Spectrum MicrofacetBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  // TODO: proj3-2, part 3
  // Implement microfacet model here.
  return Spectrum();
}

Spectrum MicrofacetBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  // TODO: proj3-2, part 3
  // *Importance* sample Beckmann normal distribution function (NDF) here.
  // Note: You should fill in the sampled direction *wi and the corresponding *pdf,
  //       and return the sampled BRDF value.
  *wi = cosineHemisphereSampler.get_sample(pdf); //placeholder
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
