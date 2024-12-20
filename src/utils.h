/**
 * @file utils.h
 * @brief Collection of utility function which are not directly connected to
 * other classes and methods.
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

/**
 * @brief Detect vertex-triangle intersections
 * @param qp Previous position of the vertex
 * @param q Current position of the vertex
 * @param x1 First vertex of the triangle
 * @param x2 Second vertex of the triangle
 * @param x3 Third vertex of the triangle
 * @param slack threshold up to which to discard the collision, useful for
 * closed meshes where we want to ignore intersections with triangles on the
 * backside, set to infinity for objects like the floor where we want the
 * vertices to stay strictly in the half-space given by the plane
 * @param penetrationDepth the penetration depth along the normal will be
 * written back to this variable for later use
 */
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