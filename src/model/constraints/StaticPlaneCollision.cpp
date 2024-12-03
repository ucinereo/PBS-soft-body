/**
 * @file StaticPlaneCollision.cpp
 * @brief Definitions of the Static Plane Collision Constraint
 */

#include "../Constraint.h"
#include <iostream>

void StaticPlaneCollisionConstraint::solve(ConstraintQueryRecord &cRec) const {
  Eigen::Vector3d x1 = cRec.X.row(m_indices[0]).transpose();

  double C = m_normal.dot(x1) - m_restDistance;

  Eigen::MatrixX3d dCdx(1, 3);
  dCdx << m_normal.transpose();

  if (C > 0) {
    C *= 0;
    dCdx *= 0;
  }

  cRec.updateValGrad(C, dCdx);
}