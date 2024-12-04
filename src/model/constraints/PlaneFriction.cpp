/**
 * @file PlaneFriction.cpp
 * @brief Definitions of the Static Plane Constraint
 */

#include "../Constraint.h"
#include <iostream>

void PlaneFrictionConstraint::solve(ConstraintQueryRecord &cRec) const {
  Eigen::Vector3d x1p = cRec.Xp.row(m_indices[0]).transpose();
  Eigen::Vector3d x1 = cRec.X.row(m_indices[0]).transpose();

  Eigen::Vector3d deltaX = x1 - x1p;
  Eigen::Vector3d deltaXPerp = deltaX - m_normal * m_normal.dot(deltaX);

  double C = deltaXPerp.norm();
  Eigen::MatrixX3d dCdx(1, 3);

  Eigen::Vector3d t0 = x1 - x1p;
  double t1 = 1 / (t0 - t0.dot(m_normal) * m_normal).norm();
  Eigen::Vector3d t2 = t0 - m_normal.dot(t0) * m_normal;
  dCdx.row(0) = (t1 * t2 - t1 * m_normal.dot(t2) * m_normal).transpose();

  double penetrationDepth = m_normal.dot(x1p);

  // Up to here, Static Friction
  if (C > m_staticMu * penetrationDepth) {
    // Kinetic Friction
    double factor = std::min(m_kineticMu * penetrationDepth / C, 1.0);
    C -= (1 - factor) * C;
    dCdx -= (1 - factor) * dCdx;
  }

  cRec.updateValGrad(C, dCdx);
}