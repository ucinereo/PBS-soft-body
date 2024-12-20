/**
 * @file BVH.h
 * @brief Defines the Bounding Volume Hierarchy (BVH) interface which is used to
 * accelerate vertex<->triangle collision queries
 */
#pragma once

#include "AABB.h"
#include <Eigen/Core>
#include <vector>

/**
 * @class BVH
 * @brief Simple implementation of a BVH for the static objects in the scene.
 * Can be used to accelerate collision detection.
 */
class BVH {
  /**
   * @struct BVH
   * @brief A simple data structure to hold information about the nodes in the
   * BVH tree
   */
  struct BVHNode {
    /**
     * @brief Create a new BVHNode.
     * @param bbox An axis-aligned bounding box that encloses the triangles
     * inside this BVHNode
     * @param start The starting index in the triangle list
     * @param length The number of triangles inside this BVHNode
     */
    BVHNode(AABB bbox, Eigen::Index start, Eigen::Index length)
        : m_bbox(bbox), m_start(start), m_length(length) {}

    /**
     * @returns true if this BVHNode is a leaf, otherwise false.
     */
    bool isLeaf() const { return m_left == nullptr && m_right == nullptr; }

    /**
     * @returns true if this BVHNode has a left child, otherwise false.
     */
    bool hasLeft() const { return m_left != nullptr; }

    /**
     * @returns The left child of this node
     */
    BVHNode *getLeft() const { return m_left; }

    /**
     * @brief Update the left child of this node
     * @param left A pointer to the corresponding BVHNode
     */
    void setLeft(BVHNode *left) { m_left = left; }

    /**
     * @returns true if this BVHNode has a right child, otherwise false.
     */
    bool hasRight() const { return m_right != nullptr; }

    /**
     * @returns The right child of this node
     */
    BVHNode *getRight() const { return m_right; }

    /**
     * @brief Update the right child of this node
     * @param right A pointer to the corresponding BVHNode
     */
    void setRight(BVHNode *right) { m_right = right; }

    /**
     * @returns The starting index in the triangle list
     */
    Eigen::Index getStart() const { return m_start; }

    /**
     * @returns The number of triangles inside this BVHNode
     */
    Eigen::Index getLength() const { return m_length; }

    /**
     * @brief Fast check if a query point lies within the bounding box of this
     * BVHNode
     * @param q The query point
     * @returns true if the query point is inside the bounds, otherwise false;
     */
    bool query(const Eigen::Vector3d &q) { return m_bbox.query(q); }

  private:
    AABB m_bbox; /// The axis-aligned bounding box of the triangles within this
                 /// BVHNode
    BVHNode *m_left = nullptr;  /// Pointer to the left child in the tree
    BVHNode *m_right = nullptr; /// Pointer to the right child in the tree
    Eigen::Index m_start;       /// The starting index in the triangle list
    Eigen::Index m_length;      /// The number of triangles inside this BVHNode
  };

public:
  /**
   * @brief Create a new BVH from a given set of triangles.
   * @param vertices A set of vertices
   * @param faces A set of indices denoting the triangle relationships between
   * the vertices
   * @parm slack A slack value to avoid missing a collision, this grows the
   * AABBs of the BVH nodes
   */
  BVH(Eigen::MatrixX3d &vertices, Eigen::MatrixX3i &faces, double slack);

  /**
   * @brief Destructor of the BVH, deletes all the nodes in the BVH tree
   */
  ~BVH();

  /**
   * @brief Collect all the triangles that are possible candidates for a
   * collision with the query point
   * @param q The query point
   * @param triangles The collision candidates
   */
  void query(const Eigen::Vector3d &q,
             std::vector<Eigen::Index> &triangles) const;

private:
  BVHNode *m_root; /// Pointer to the root node of the BVH tree
  std::vector<Eigen::Index>
      m_indices; /// The set of indices pointing to all the triangles
};