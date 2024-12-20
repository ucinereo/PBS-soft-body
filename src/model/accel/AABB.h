/**
 * @file AABB.h
 * @brief Defines and implements a simple axis-aligned bounding box (AABB)
 * utility class
 */
#pragma once

#include <Eigen/Core>

/**
 * @class AABB
 * @brief Utility class that implements an axis-aligned bounding box
 */
class AABB {
public:
  /**
   * @brief Initialize an empty AABB.
   * @param slack A slack value that virtually expands the bounds
   */
  AABB(double slack) : m_slack(slack) {
    m_min.setConstant(std::numeric_limits<double>::infinity());
    m_max.setConstant(-std::numeric_limits<double>::infinity());
  }

  /**
   * @brief Add a new point to the bounding box
   * @param v The point to be included
   */
  void expand(Eigen::Vector3d &v) {
    m_min = m_min.cwiseMin(v);
    m_max = m_max.cwiseMax(v);
  }

  /**
   * @brief Return the longest axis of the bounding box. This axis is used to
   * decide how to split in the BVH construction.
   * @returns The longest axis, 0: x-axis, 1: y-axis, 2: z-axis
   */
  Eigen::Index longestAxis() const {
    Eigen::Index axis;
    (m_max - m_min).cwiseAbs().maxCoeff(&axis);

    return axis;
  }

  /**
   * @brief Fast inclusion check given a point with some predefined amount of
   * slack that grows the bounds a bit. This will result in a few false
   * positives, but will avoid missing vertex<->triangle collisions.
   * @param q The query point
   * @returns true if the query point lies inside the bounding box, otherwise
   * false.
   */
  bool query(const Eigen::Vector3d &q) const {
    return (m_min.array() - m_slack <= q.array()).all() &&
           (q.array() <= m_max.array() + m_slack).all();
  }

private:
  Eigen::Vector3d m_min,
      m_max;      /// Vectors defining the extents of the bounding box
  double m_slack; /// Slack value by which to virtually grow the bounds
};