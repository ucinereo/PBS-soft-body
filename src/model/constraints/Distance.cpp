/**
 * @file Distance.cpp
 * @brief Definitions of the Distance Constraint
 */

#include "../Constraint.h"

DistanceConstraint::DistanceConstraint(double compliance, Eigen::MatrixX3d &X0,
                                       Eigen::Index index1, Eigen::Index index2)
    : Constraint(compliance, {index1, index2}) {
  Eigen::Vector3d x1 = X0.row(m_indices[0]).transpose();
  Eigen::Vector3d x2 = X0.row(m_indices[1]).transpose();

  m_distance = (x1 - x2).norm();
}

void DistanceConstraint::solve(ConstraintQueryRecord &cRec) const {
  Eigen::Vector3d x1 = cRec.X.row(m_indices[0]).transpose();
  Eigen::Vector3d x2 = cRec.X.row(m_indices[1]).transpose();

  double C = (x1 - x2).norm() - m_distance;

  Eigen::MatrixX3d dCdx(2, 3);
  Eigen::Vector3d n = (x1 - x2).normalized();
  dCdx << n.transpose(), -n.transpose();

  cRec.updateValGrad(C, dCdx);
}