#include "bvh.h"

#include "CGL/CGL.h"
#include "triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CGL {
namespace SceneObjects {

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {

  primitives = std::vector<Primitive *>(_primitives);
  root = construct_bvh(primitives.begin(), primitives.end(), max_leaf_size);
}

BVHAccel::~BVHAccel() {
  if (root)
    delete root;
  primitives.clear();
}

BBox BVHAccel::get_bbox() const { return root->bb; }

void BVHAccel::draw(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->draw(c, alpha);
    }
  } else {
    draw(node->l, c, alpha);
    draw(node->r, c, alpha);
  }
}

void BVHAccel::drawOutline(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->drawOutline(c, alpha);
    }
  } else {
    drawOutline(node->l, c, alpha);
    drawOutline(node->r, c, alpha);
  }
}

BVHNode *BVHAccel::construct_bvh(std::vector<Primitive *>::iterator start,
                                 std::vector<Primitive *>::iterator end,
                                 size_t max_leaf_size) {

  // TODO (Part 2.1):
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.

  BBox bbox;

  std::vector<double> x_values;
  std::vector<double> y_values;
  std::vector<double> z_values;

  for (auto p = start; p != end; p++) {
    BBox bb = (*p)->get_bbox();
    bbox.expand(bb);

    Vector3D centroid = bb.centroid();
    x_values.push_back(centroid.x);
    y_values.push_back(centroid.y);
    z_values.push_back(centroid.z);
  }

  int count = std::distance(start, end);

  if (count <= 1 || count <= max_leaf_size) {
    BVHNode *node = new BVHNode(bbox);
    node->start = start;
    node->end = end;
    return node;
  }

  double x_length = bbox.max.x - bbox.min.x;
  double y_length = bbox.max.y - bbox.min.y;
  double z_length = bbox.max.z - bbox.min.z;

  // figure out with axis to split by finding the median centroid
  if (x_length > y_length && x_length > z_length) {
    sort(start, end, [](Primitive *a, Primitive *b) {
      return a->get_bbox().centroid().x < b->get_bbox().centroid().x;
    });

  } else if (y_length > x_length && y_length > z_length) {
    sort(start, end, [](Primitive *a, Primitive *b) {
      return a->get_bbox().centroid().y < b->get_bbox().centroid().y;
    });
  
  } else {
    sort(start, end, [](Primitive *a, Primitive *b) {
      return a->get_bbox().centroid().z < b->get_bbox().centroid().z;
    });
  }

  // if count >= 2, left and right are guaranteed to be both non-empty
  BVHNode *node = new BVHNode(bbox);
  node->l = construct_bvh(start, start + (count + 1) / 2, max_leaf_size);
  node->r = construct_bvh(start + (count + 1) / 2, end, max_leaf_size);

  return node;
}

bool BVHAccel::has_intersection(const Ray &ray, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
  // Take note that this function has a short-circuit that the
  // Intersection version cannot, since it returns as soon as it finds
  // a hit, it doesn't actually have to find the closest hit.

  double t0, t1;

  if (!node->bb.intersect(ray, t0, t1)) return false;

  // this is a cheaper substitute for node->isLeaf()
  // but is only true in our current impl of split
  if (node->l == NULL) {
    for (auto p = node->start; p != node->end; p++) {
      if ((*p)->has_intersection(ray)) {
        total_isects += std::distance(p, node->start);
        return true;
      }
    }
    return false;
  }

  return has_intersection(ray, node->l) || has_intersection(ray, node->r);
}

bool BVHAccel::intersect(const Ray &ray, Intersection *i, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.

  double t0, t1;

  if (!node->bb.intersect(ray, t0, t1)) return false;

  if (node->l == NULL) {
    bool hit = false;
    for (auto p = node->start; p != node->end; p++) {
      hit = (*p)->intersect(ray, i) | hit;
    }
    total_isects += std::distance(node->end, node->start);
    return hit;
  }

  // no short-circuiting (please)
  return intersect(ray, i, node->l) | intersect(ray, i, node->r);
}

} // namespace SceneObjects
} // namespace CGL
