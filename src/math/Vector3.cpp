#include "acs/math/Vector3.h"

#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace acs::math {

Vector3::Vector3(double x_in, double y_in, double z_in) : x(x_in), y(y_in), z(z_in) {}

double Vector3::norm_squared() const { return x * x + y * y + z * z; }

double Vector3::norm() const { return std::sqrt(norm_squared()); }

Vector3 Vector3::normalized() const {
  const double n = norm();
  if (n <= 0.0) {
    return Vector3{};
  }
  return (*this) / n;
}

double Vector3::dot(const Vector3& a, const Vector3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

Vector3 Vector3::cross(const Vector3& a, const Vector3& b) {
  return Vector3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

Vector3 Vector3::operator+(const Vector3& other) const { return Vector3(x + other.x, y + other.y, z + other.z); }

Vector3 Vector3::operator-(const Vector3& other) const { return Vector3(x - other.x, y - other.y, z - other.z); }

Vector3 Vector3::operator-() const { return Vector3(-x, -y, -z); }

Vector3 Vector3::operator*(double s) const { return Vector3(x * s, y * s, z * s); }

Vector3 Vector3::operator/(double s) const { return Vector3(x / s, y / s, z / s); }

Vector3& Vector3::operator+=(const Vector3& other) {
  x += other.x;
  y += other.y;
  z += other.z;
  return *this;
}

Vector3& Vector3::operator-=(const Vector3& other) {
  x -= other.x;
  y -= other.y;
  z -= other.z;
  return *this;
}

Vector3& Vector3::operator*=(double s) {
  x *= s;
  y *= s;
  z *= s;
  return *this;
}

Vector3& Vector3::operator/=(double s) {
  x /= s;
  y /= s;
  z /= s;
  return *this;
}

double& Vector3::operator[](std::size_t idx) {
  if (idx == 0) return x;
  if (idx == 1) return y;
  if (idx == 2) return z;
  throw std::out_of_range("Vector3 index");
}

double Vector3::operator[](std::size_t idx) const {
  if (idx == 0) return x;
  if (idx == 1) return y;
  if (idx == 2) return z;
  throw std::out_of_range("Vector3 index");
}

std::string Vector3::to_string(int precision) const {
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(precision) << "[" << x << ", " << y << ", " << z << "]";
  return ss.str();
}

Vector3 operator*(double s, const Vector3& v) { return v * s; }

}  // namespace acs::math

