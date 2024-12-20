/**
 * @file BVH.cpp
 * @brief Simple implementation of the Bounding Volume Hierarchy (BVH)
 */

#include "BVH.h"
#include <iostream>
#include <numeric>

BVH::BVH(Eigen::MatrixX3d &vertices, Eigen::MatrixX3i &faces, double slack) {
  std::cout << "Building BVH for " << faces.rows() << " triangles..."
            << std::endl;
  m_indices.resize(faces.rows());
  std::fill(m_indices.begin(), m_indices.end(), 0);
  std::iota(m_indices.begin(), m_indices.end(), 0);

  std::vector<std::tuple<BVHNode *, int, bool, Eigen::Index, Eigen::Index>>
      queue;
  queue.emplace(queue.begin(), nullptr, 0, false, 0, faces.rows());

  int max_depth = 0;
  int min_width = faces.rows();

  while (!queue.empty()) {
    auto [parent, depth, isLeft, start, length] = queue.back();
    queue.pop_back();

    max_depth = std::max(max_depth, depth);
    min_width = std::min(min_width, (int)length);

    AABB bbox(slack);
    for (size_t i = start; i < start + length; i++) {
      Eigen::Index j = m_indices[i];

      Eigen::Vector3d v0 = vertices.row(faces(j, 0)).transpose();
      Eigen::Vector3d v1 = vertices.row(faces(j, 1)).transpose();
      Eigen::Vector3d v2 = vertices.row(faces(j, 2)).transpose();
      bbox.expand(v0);
      bbox.expand(v1);
      bbox.expand(v2);
    }
    auto *node = new BVHNode(bbox, start, length);

    if (parent == nullptr) {
      m_root = node;
    } else {
      if (isLeft) {
        parent->setLeft(node);
      } else {
        parent->setRight(node);
      }
    }

    // Heuristic to tune depth of the tree vs. width of the leafs
    if (depth >= length)
      continue;

    Eigen::Index axis = bbox.longestAxis();

    std::sort(m_indices.begin() + start, m_indices.begin() + start + length,
              [&faces, &vertices, &axis](Eigen::Index a, Eigen::Index b) {
                Eigen::RowVector3i vertsA = faces.row(a);
                Eigen::RowVector3i vertsB = faces.row(b);

                double midpointA =
                    (vertices(vertsA(0), axis) + vertices(vertsA(1), axis) +
                     vertices(vertsA(2), axis)) /
                    3;

                double midpointB =
                    (vertices(vertsB(0), axis) + vertices(vertsB(1), axis) +
                     vertices(vertsB(2), axis)) /
                    3;

                return midpointA < midpointB;
              });

    Eigen::Index middle = length / 2;

    queue.emplace(queue.begin(), node, depth + 1, true, start, middle);
    queue.emplace(queue.begin(), node, depth + 1, false, start + middle,
                  length - middle);
  }

  std::cout << "Done. Max Depth: " << max_depth << ", Min Width: " << min_width
            << std::endl;
}

BVH::~BVH() {
  std::vector<BVHNode *> queue;
  queue.emplace(queue.begin(), m_root);

  while (!queue.empty()) {
    BVHNode *node = queue.back();
    queue.pop_back();

    if (node->hasLeft())
      queue.emplace(queue.begin(), node->getLeft());

    if (node->hasRight())
      queue.emplace(queue.begin(), node->getRight());

    delete node;
  }
}

void BVH::query(const Eigen::Vector3d &q,
                std::vector<Eigen::Index> &triangles) const {
  std::vector<BVHNode *> queue;
  queue.emplace(queue.begin(), m_root);

  while (!queue.empty()) {
    BVHNode *node = queue.back();
    queue.pop_back();

    if (!node->query(q))
      continue;

    if (node->hasLeft())
      queue.emplace(queue.begin(), node->getLeft());

    if (node->hasRight())
      queue.emplace(queue.begin(), node->getRight());

    if (node->isLeaf()) {
      triangles.insert(triangles.end(), m_indices.begin() + node->getStart(),
                       m_indices.begin() + node->getStart() +
                           node->getLength());
    }
  }
}