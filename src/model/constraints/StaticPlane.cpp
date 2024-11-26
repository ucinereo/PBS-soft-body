/**
 * @file StaticPlane.cpp
 * @brief Definitions of the Static Plane Constraint
 */

#include "../Constraint.h"

void StaticPlaneConstraint::operator()(ConstraintQueryRecord &cRec) const {
  Eigen::Vector3d u = cRec.x.row(m_indices[0]).transpose();

  double C = m_normal.dot(u) - m_restDistance;

  Eigen::MatrixX3d dCdx(1, 3);
  dCdx << m_normal.transpose();

  cRec.updateValGrad(C, dCdx);
}