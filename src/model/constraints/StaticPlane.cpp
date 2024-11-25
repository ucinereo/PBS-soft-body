/**
 * @file StaticPlane.cpp
 * @brief Definitions of the Static Plane Constraint
 */

#include "StaticPlane.h"

double StaticPlaneConstraint::operator()(const Eigen::MatrixX3d &x) const {
  Eigen::Vector3d u = x.row(m_indices[0]).transpose();
  return m_normal.dot(u) - m_restDistance;
}

Eigen::MatrixX3d StaticPlaneConstraint::grad(const Eigen::MatrixX3d &x) const {
  Eigen::MatrixX3d dC(1, 3);
  dC << m_normal.transpose();

  return dC;
}