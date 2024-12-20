#pragma once

#include <Eigen/Core>

class AABB {
public:
  AABB(double slack) : m_slack(slack) {
    m_min.setConstant(std::numeric_limits<double>::infinity());
    m_max.setConstant(-std::numeric_limits<double>::infinity());
  }

  void expand(Eigen::Vector3d &v) {
    m_min = m_min.cwiseMin(v);
    m_max = m_max.cwiseMax(v);
  }

  Eigen::Index longestAxis() const {
    Eigen::Index axis;
    (m_max - m_min).cwiseAbs().maxCoeff(&axis);

    return axis;
  }

  bool query(const Eigen::Vector3d &q) const {
    return (m_min.array() - m_slack <= q.array()).all() &&
           (q.array() <= m_max.array() + m_slack).all();
  }

private:
  Eigen::Vector3d m_min, m_max;
  double m_slack;
};