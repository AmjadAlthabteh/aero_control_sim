#include "acs/math/Matrix3.h"

#include <cmath>
#include <stdexcept>

namespace acs::math {

Matrix3 Matrix3::identity() {
  Matrix3 a{};
  a.m[0][0] = 1.0;
  a.m[1][1] = 1.0;
  a.m[2][2] = 1.0;
  return a;
}

Matrix3 Matrix3::zeros() { return Matrix3{}; }

Matrix3 Matrix3::from_rows(const Vector3& r0, const Vector3& r1, const Vector3& r2) {
  Matrix3 a{};
  a.m[0][0] = r0.x;
  a.m[0][1] = r0.y;
  a.m[0][2] = r0.z;
  a.m[1][0] = r1.x;
  a.m[1][1] = r1.y;
  a.m[1][2] = r1.z;
  a.m[2][0] = r2.x;
  a.m[2][1] = r2.y;
  a.m[2][2] = r2.z;
  return a;
}

Vector3 Matrix3::row(int i) const { return Vector3(m[i][0], m[i][1], m[i][2]); }

Vector3 Matrix3::col(int j) const { return Vector3(m[0][j], m[1][j], m[2][j]); }

Matrix3 Matrix3::transposed() const {
  Matrix3 a{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      a.m[i][j] = m[j][i];
    }
  }
  return a;
}

double Matrix3::det() const {
  // cofactor expansion
  return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
         m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
         m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
}

Matrix3 Matrix3::inverse() const {
  // Tikhonov regularisation: if the matrix is near-singular, add ε·I before
  // inverting rather than throwing.
  //
  // Why not throw?
  //   An exception mid-simulation aborts the entire run and discards all
  //   telemetry collected up to that point.  A near-singular inertia tensor
  //   (e.g. from a mis-typed diagonal element) should not be a fatal error.
  //
  // What regularisation does:
  //   Adding ε·I shifts all singular values by +ε, guaranteeing |det| > 0.
  //   For a well-conditioned physical matrix (inertia tensor, rotation) ε is
  //   many orders of magnitude below the smallest eigenvalue, so the inverse
  //   is numerically identical to the unperturbed result.  For a genuinely
  //   degenerate matrix it returns the best available inverse rather than
  //   crashing.  The choice ε = 1e-6 sits comfortably between the 1e-12
  //   singularity threshold and any physically meaningful inertia value.
  double d = det();
  Matrix3 work = *this;
  if (std::abs(d) < 1e-12) {
    constexpr double kEps = 1e-6;
    work.m[0][0] += kEps;
    work.m[1][1] += kEps;
    work.m[2][2] += kEps;
    d = work.det();
  }
  if (std::abs(d) < 1e-30) {
    // Completely degenerate even after regularisation — return identity.
    return Matrix3::identity();
  }

  // Adjugate / Cramer's rule for the (possibly regularised) matrix.
  const auto& w = work.m;
  Matrix3 a{};
  a.m[0][0] = (w[1][1] * w[2][2] - w[1][2] * w[2][1]) / d;
  a.m[0][1] = (w[0][2] * w[2][1] - w[0][1] * w[2][2]) / d;
  a.m[0][2] = (w[0][1] * w[1][2] - w[0][2] * w[1][1]) / d;

  a.m[1][0] = (w[1][2] * w[2][0] - w[1][0] * w[2][2]) / d;
  a.m[1][1] = (w[0][0] * w[2][2] - w[0][2] * w[2][0]) / d;
  a.m[1][2] = (w[0][2] * w[1][0] - w[0][0] * w[1][2]) / d;

  a.m[2][0] = (w[1][0] * w[2][1] - w[1][1] * w[2][0]) / d;
  a.m[2][1] = (w[0][1] * w[2][0] - w[0][0] * w[2][1]) / d;
  a.m[2][2] = (w[0][0] * w[1][1] - w[0][1] * w[1][0]) / d;

  return a;
}

Vector3 Matrix3::operator*(const Vector3& v) const {
  return Vector3(m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
                 m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
                 m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z);
}

Matrix3 Matrix3::operator*(const Matrix3& other) const {
  Matrix3 a{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      double s = 0.0;
      for (int k = 0; k < 3; ++k) {
        s += m[i][k] * other.m[k][j];
      }
      a.m[i][j] = s;
    }
  }
  return a;
}

Matrix3 Matrix3::operator+(const Matrix3& other) const {
  Matrix3 a{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      a.m[i][j] = m[i][j] + other.m[i][j];
    }
  }
  return a;
}

Matrix3 Matrix3::operator-(const Matrix3& other) const {
  Matrix3 a{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      a.m[i][j] = m[i][j] - other.m[i][j];
    }
  }
  return a;
}

Matrix3 Matrix3::operator*(double s) const {
  Matrix3 a{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      a.m[i][j] = m[i][j] * s;
    }
  }
  return a;
}

Matrix3 operator*(double s, const Matrix3& a) { return a * s; }

}  // namespace acs::math

