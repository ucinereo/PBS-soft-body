/**
 * @file Distance.cpp
 * @brief Definitions of the Distance Constraint
 */

#include "../Constraint.h"

DistanceConstraint::DistanceConstraint(double compliance, Eigen::MatrixX3d &x0,
                                       Eigen::Index indexU, Eigen::Index indexV)
    : Constraint(compliance, {indexU, indexV}) {
  Eigen::Vector3d u = x0.row(m_indices[0]).transpose();
  Eigen::Vector3d v = x0.row(m_indices[1]).transpose();
  
  m_distance = (u - v).norm();
}

void DistanceConstraint::operator()(ConstraintQueryRecord &cRec) const {
  Eigen::Vector3d u = cRec.x.row(m_indices[0]).transpose();
  Eigen::Vector3d v = cRec.x.row(m_indices[1]).transpose();

  double C = (u - v).norm() - m_distance;

  Eigen::MatrixX3d dCdx(2, 3);
  Eigen::Vector3d n = (u - v).normalized();
  dCdx << n.transpose(), -n.transpose();

  cRec.updateValGrad(C, dCdx);
}