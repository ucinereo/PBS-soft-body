#pragma once

#include "aabb.h"
#include <Eigen/Core>
#include <vector>

class BVH {
  int MAX_DEPTH = 14;
  int CUTOFF = 10;

  struct BVHNode {
    AABB m_bbox;
    BVHNode *m_left = nullptr;
    BVHNode *m_right = nullptr;
    Eigen::Index m_start;
    Eigen::Index m_length;

    BVHNode(AABB bbox, Eigen::Index start, Eigen::Index length)
        : m_bbox(bbox), m_start(start), m_length(length) {}

    bool isLeaf() const { return m_left == nullptr && m_right == nullptr; }

    bool hasLeft() const { return m_left != nullptr; }
    BVHNode *getLeft() const { return m_left; }
    void setLeft(BVHNode *left) { m_left = left; }

    bool hasRight() const { return m_right != nullptr; }
    BVHNode *getRight() const { return m_right; }
    void setRight(BVHNode *right) { m_right = right; }

    Eigen::Index getStart() const { return m_start; }

    Eigen::Index getLength() const { return m_length; }

    bool query(const Eigen::Vector3d &q) { return m_bbox.query(q); }
  };

public:
  BVH(Eigen::MatrixX3d &vertices, Eigen::MatrixX3i &faces, double slack);

  void query(const Eigen::Vector3d &q,
             std::vector<Eigen::Index> &triangles) const;

private:
  BVHNode *m_root;
  std::vector<Eigen::Index> m_indices;
};