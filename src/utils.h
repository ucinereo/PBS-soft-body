/**
 * @file utils.h
 * @brief Collection of utility function which are not directly connected to
 * other classes and methods.
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

inline bool vertexIntersectsTriangle(const Eigen::Vector3d &qp,
                                     const Eigen::Vector3d &q,
                                     const Eigen::Vector3d &x1,
                                     const Eigen::Vector3d &x2,
                                     const Eigen::Vector3d &x3, double slack,
                                     double &penetrationDepth) {
  Eigen::Vector3d e12 = x2 - x1, e13 = x3 - x1;
  Eigen::Vector3d n = e12.cross(e13).normalized();
  double d = n.dot(q - x1);
  if (n.dot(qp - x1) < -slack || d > 1e-3) {
    return false;
  }
  Eigen::Vector3d dq = qp - q;

  // Solve n.dot(qp + t * dq - x1) = 0 for t
  double t = n.dot(x1 - qp) / n.dot(dq);

  Eigen::Vector3d p = qp + t * dq;
  penetrationDepth = -d;

  double A123 = e12.cross(e13).norm() / 2;
  double A12p = (p - x1).cross(p - x2).norm() / 2;
  double A13p = (p - x1).cross(p - x3).norm() / 2;
  double A23p = (p - x2).cross(p - x3).norm() / 2;

  double u = A12p / A123;
  double v = A13p / A123;
  double w = A23p / A123;

  return abs(1 - u - v - w) < 1e-12;
}