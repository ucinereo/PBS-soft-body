/**
 * @file Distance.cpp
 * @brief Definitions of the Distance Constraint
 */

#include "Distance.h"

DistanceConstraint::DistanceConstraint(Eigen::MatrixX3d &x0,
                                       Eigen::Index indexU, Eigen::Index indexV,
                                       double inverseStiffness)
    : Constraint({indexU, indexV}, inverseStiffness) {
  Eigen::Vector3d u = x0.row(m_indices[0]).transpose();
  Eigen::Vector3d v = x0.row(m_indices[1]).transpose();
  m_distance = (u - v).norm();
}

double DistanceConstraint::operator()(const Eigen::MatrixX3d &x) const {
  Eigen::Vector3d u = x.row(m_indices[0]).transpose();
  Eigen::Vector3d v = x.row(m_indices[1]).transpose();
  return (u - v).norm() - m_distance;
}

Eigen::MatrixX3d DistanceConstraint::grad(const Eigen::MatrixX3d &x) const {
  Eigen::MatrixX3d dC(2, 3);

  Eigen::Vector3d u = x.row(m_indices[0]).transpose();
  Eigen::Vector3d v = x.row(m_indices[1]).transpose();

  Eigen::Vector3d n = (u - v).normalized();

  dC << n.transpose(), -n.transpose();

  return dC;
}