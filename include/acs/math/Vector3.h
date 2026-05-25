#pragma once

#include <cstddef>
#include <string>

namespace acs::math {

class Vector3 {
public:
  double x{};
  double y{};
  double z{};

  Vector3() = default;
  Vector3(double x_in, double y_in, double z_in);

  double norm() const;
  double norm_squared() const;
  Vector3 normalized() const;

  static double dot(const Vector3& a, const Vector3& b);
  static Vector3 cross(const Vector3& a, const Vector3& b);

  Vector3 operator+(const Vector3& other) const;
  Vector3 operator-(const Vector3& other) const;
  Vector3 operator-() const;
  Vector3 operator*(double s) const;
  Vector3 operator/(double s) const;

  Vector3& operator+=(const Vector3& other);
  Vector3& operator-=(const Vector3& other);
  Vector3& operator*=(double s);
  Vector3& operator/=(double s);

  double& operator[](std::size_t idx);
  double operator[](std::size_t idx) const;

  std::string to_string(int precision = 6) const;
};

Vector3 operator*(double s, const Vector3& v);

}  // namespace acs::math

