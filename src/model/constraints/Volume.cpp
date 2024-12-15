/**
 * @file Distance.cpp
 * @brief Definitions of the Distance Constraint
 */

#include "../Constraint.h"
#include <Eigen/Geometry>
#include <iostream>

ShellVolumeConstraint::ShellVolumeConstraint(
    double compliance, Eigen::MatrixX3d &x0, Eigen::MatrixX3i triangles,
    Eigen::Index start, Eigen::Index length, double pressure)
    : Constraint(compliance, start, length), m_triangles(triangles),
      m_pressure(pressure) {
  m_init_volume = calculateVolume(x0);
}

void ShellVolumeConstraint::solve(ConstraintQueryRecord &cRec) const {
  double currentVolume = calculateVolume(cRec.X);

  double C = currentVolume - m_pressure * m_init_volume;

  Eigen::Index n = cRec.Xp.rows();
  Eigen::MatrixX3d dCdx(n, 3);
  dCdx.setZero();

  if (abs(C) > 1e-5) {
    for (Eigen::Index i = 0; i < m_triangles.rows(); i++) {
      Eigen::Index i1 = m_triangles(i, 0);
      Eigen::Index i2 = m_triangles(i, 1);
      Eigen::Index i3 = m_triangles(i, 2);
      Eigen::Vector3d p1 = cRec.X.row(i1).transpose();
      Eigen::Vector3d p2 = cRec.X.row(i2).transpose();
      Eigen::Vector3d p3 = cRec.X.row(i3).transpose();

      dCdx.row(i1) += 1.0 / 6.0 * (p2.cross(p3)).transpose();
      dCdx.row(i2) += 1.0 / 6.0 * (p3.cross(p1)).transpose();
      dCdx.row(i3) += 1.0 / 6.0 * (p1.cross(p2)).transpose();
    }
  }
  cRec.updateValGrad(C, dCdx);
};

double ShellVolumeConstraint::calculateVolume(const Eigen::MatrixX3d &x) const {
  double volume = 0.0;
  for (Eigen::Index i = 0; i < m_triangles.rows(); i++) {
    Eigen::Index i1 = m_triangles(i, 0);
    Eigen::Index i2 = m_triangles(i, 1);
    Eigen::Index i3 = m_triangles(i, 2);
    Eigen::Vector3d p1 = x.row(i1).transpose();
    Eigen::Vector3d p2 = x.row(i2).transpose();
    Eigen::Vector3d p3 = x.row(i3).transpose();

    volume += 1.0 / 6.0 * (p1.cross(p2)).dot(p3);
  }
  return volume;
}

TetVolumeConstraint::TetVolumeConstraint(double compliance,
                                         Eigen::MatrixX3d &x0, double pressure,
                                         Eigen::Index p0, Eigen::Index p1,
                                         Eigen::Index p2, Eigen::Index p3)
    : Constraint(compliance, {p0, p1, p2, p3}), m_pressure(pressure) {
  m_init_volume = calculateVolume(x0);
}

double TetVolumeConstraint::calculateVolume(const Eigen::MatrixX3d &x) const {
  Eigen::Vector3d p0 = x.row(m_indices[0]).transpose();
  Eigen::Vector3d p1 = x.row(m_indices[1]).transpose();
  Eigen::Vector3d p2 = x.row(m_indices[2]).transpose();
  Eigen::Vector3d p3 = x.row(m_indices[3]).transpose();

  double volume = 1.0 / 6.0 * ((p1 - p0).cross(p2 - p0)).dot(p3 - p0);
  return volume;
}

void TetVolumeConstraint::solve(ConstraintQueryRecord &cRec) const {
  double currentVolume = calculateVolume(cRec.X);

  double C = currentVolume - m_pressure * m_init_volume;

  Eigen::MatrixX3d dCdx(4, 3);
  dCdx.setZero();

  if (abs(C) > 1e-5) {
    Eigen::Vector3d p0 = cRec.X.row(m_indices[0]).transpose();
    Eigen::Vector3d p1 = cRec.X.row(m_indices[1]).transpose();
    Eigen::Vector3d p2 = cRec.X.row(m_indices[2]).transpose();
    Eigen::Vector3d p3 = cRec.X.row(m_indices[3]).transpose();

    dCdx.row(0) = 1.0 / 6.0 * ((p1 - p2).cross(p3 - p2)).transpose();
    dCdx.row(1) = 1.0 / 6.0 * ((p2 - p0).cross(p3 - p0)).transpose();
    dCdx.row(2) = 1.0 / 6.0 * ((p0 - p1).cross(p3 - p1)).transpose();
    dCdx.row(3) = 1.0 / 6.0 * ((p1 - p0).cross(p2 - p0)).transpose();
  }
  cRec.updateValGrad(C, dCdx);
};