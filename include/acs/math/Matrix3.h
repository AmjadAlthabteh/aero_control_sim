#pragma once

#include "acs/math/Vector3.h"

namespace acs::math {

class Matrix3 {
public:
  // row-major storage
  double m[3][3]{};

  Matrix3() = default;
  static Matrix3 identity();
  static Matrix3 zeros();
  static Matrix3 from_rows(const Vector3& r0, const Vector3& r1, const Vector3& r2);

  Vector3 row(int i) const;
  Vector3 col(int j) const;

  Matrix3 transposed() const;
  double det() const;
  Matrix3 inverse() const;

  Vector3 operator*(const Vector3& v) const;
  Matrix3 operator*(const Matrix3& other) const;
  Matrix3 operator+(const Matrix3& other) const;
  Matrix3 operator-(const Matrix3& other) const;
  Matrix3 operator*(double s) const;
};

Matrix3 operator*(double s, const Matrix3& a);

}  // namespace acs::math

