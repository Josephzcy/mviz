/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_TRANSFORM_RIGID_TRANSFORM_H_
#define CARTOGRAPHER_TRANSFORM_RIGID_TRANSFORM_H_

#include <cmath>
#include <iostream>
#include <string>

#include "Eigen/Core"
#include "Eigen/Geometry"

namespace module {
namespace sensor_fusion {
namespace common {

template <typename FloatType>
class Rigid2 {
 public:
  using Vector = Eigen::Matrix<FloatType, 2, 1>;
  using Rotation2D = Eigen::Rotation2D<FloatType>;

  Rigid2() : translation_(Vector::Zero()), rotation_(Rotation2D::Identity()) {}
  Rigid2(const Vector& translation, const Rotation2D& rotation) : translation_(translation), rotation_(rotation) {}
  Rigid2(const Vector& translation, const double rotation) : translation_(translation), rotation_(rotation) {}

  static Rigid2 Rotation(const double rotation) { return Rigid2(Vector::Zero(), rotation); }

  static Rigid2 Rotation(const Rotation2D& rotation) { return Rigid2(Vector::Zero(), rotation); }

  static Rigid2 Translation(const Vector& vector) { return Rigid2(vector, Rotation2D::Identity()); }

  static Rigid2<FloatType> Identity() { return Rigid2<FloatType>(); }

  template <typename OtherType>
  Rigid2<OtherType> cast() const {
    return Rigid2<OtherType>(translation_.template cast<OtherType>(), rotation_.template cast<OtherType>());
  }

  const Vector& translation() const { return translation_; }

  Rotation2D rotation() const { return rotation_; }

  Rigid2 inverse() const {
    const Rotation2D rotation = rotation_.inverse();
    const Vector translation = -(rotation * translation_);
    return Rigid2(translation, rotation);
  }

 private:
  Vector translation_;
  Rotation2D rotation_;
};

template <typename FloatType>
Rigid2<FloatType> operator*(const Rigid2<FloatType>& lhs, const Rigid2<FloatType>& rhs) {
  return Rigid2<FloatType>(lhs.rotation() * rhs.translation() + lhs.translation(), lhs.rotation() * rhs.rotation());
}

template <typename FloatType>
typename Rigid2<FloatType>::Vector operator*(const Rigid2<FloatType>& rigid,
                                             const typename Rigid2<FloatType>::Vector& point) {
  return rigid.rotation() * point + rigid.translation();
}

using Rigid2d = Rigid2<double>;
using Rigid2f = Rigid2<float>;

template <typename FloatType>
class Rigid3 {
 public:
  using Vector = Eigen::Matrix<FloatType, 3, 1>;
  using Quaternion = Eigen::Quaternion<FloatType>;
  using AngleAxis = Eigen::AngleAxis<FloatType>;

  Rigid3() : translation_(Vector::Zero()), rotation_(Quaternion::Identity()) {}
  Rigid3(const Vector& translation, const Quaternion& rotation) : translation_(translation), rotation_(rotation) {}
  Rigid3(const Vector& translation, const AngleAxis& rotation) : translation_(translation), rotation_(rotation) {}

  static Rigid3 Rotation(const AngleAxis& angle_axis) { return Rigid3(Vector::Zero(), Quaternion(angle_axis)); }

  static Rigid3 Rotation(const Quaternion& rotation) { return Rigid3(Vector::Zero(), rotation); }

  static Rigid3 Translation(const Vector& vector) { return Rigid3(vector, Quaternion::Identity()); }

  static Rigid3 FromArrays(const std::array<FloatType, 4>& rotation, const std::array<FloatType, 3>& translation) {
    return Rigid3(Eigen::Map<const Vector>(translation.data()),
                  Eigen::Quaternion<FloatType>(rotation[0], rotation[1], rotation[2], rotation[3]));
  }

  static Rigid3<FloatType> Identity() { return Rigid3<FloatType>(); }

  template <typename OtherType>
  Rigid3<OtherType> cast() const {
    return Rigid3<OtherType>(translation_.template cast<OtherType>(), rotation_.template cast<OtherType>());
  }

  const Vector& translation() const { return translation_; }
  const Quaternion& rotation() const { return rotation_; }

  Rigid3 inverse() const {
    const Quaternion rotation = rotation_.conjugate();
    const Vector translation = -(rotation * translation_);
    return Rigid3(translation, rotation);
  }

  bool IsValid() const {
    return !std::isnan(translation_.x()) && !std::isnan(translation_.y()) && !std::isnan(translation_.z()) &&
           std::abs(FloatType(1) - rotation_.norm()) < FloatType(1e-3);
  }

 private:
  Vector translation_;
  Quaternion rotation_;
};

template <typename FloatType>
Rigid3<FloatType> operator*(const Rigid3<FloatType>& lhs, const Rigid3<FloatType>& rhs) {
  return Rigid3<FloatType>(lhs.rotation() * rhs.translation() + lhs.translation(),
                           (lhs.rotation() * rhs.rotation()).normalized());
}

template <typename FloatType>
typename Rigid3<FloatType>::Vector operator*(const Rigid3<FloatType>& rigid,
                                             const typename Rigid3<FloatType>::Vector& point) {
  return rigid.rotation() * point + rigid.translation();
}

using Rigid3d = Rigid3<double>;
using Rigid3f = Rigid3<float>;

// Converts (roll, pitch, yaw) to a unit length quaternion. Based on the URDF
Eigen::Quaterniond GetQuaternion(const double roll, const double pitch, const double yaw);

template <typename T>
typename common::Rigid3<T>::Vector GetRollPitchYaw(const Eigen::Quaternion<T>& rotation) {
  // roll (x-axis rotation)
  double sinr_cosp = +2.0 * (rotation.w() * rotation.x() + rotation.y() * rotation.z());
  double cosr_cosp = +1.0 - 2.0 * (rotation.x() * rotation.x() + rotation.y() * rotation.y());
  double roll = atan2(sinr_cosp, cosr_cosp);
  // pitch (y-axis rotation)
  double sinp = +2.0 * (rotation.w() * rotation.y() - rotation.z() * rotation.x());
  double pitch = 0.0;
  if (fabs(sinp) >= 1)
    pitch = copysign(M_PI / 2, sinp);  // use 90 degrees if out of range
  else
    pitch = asin(sinp);
  // yaw (z-axis rotation)
  double siny_cosp = +2.0 * (rotation.w() * rotation.z() + rotation.x() * rotation.y());
  double cosy_cosp = +1.0 - 2.0 * (rotation.y() * rotation.y() + rotation.z() * rotation.z());
  double yaw = atan2(siny_cosp, cosy_cosp);
  return typename common::Rigid3<T>::Vector(roll, pitch, yaw);
}

// Returns the yaw component in radians of the given 3D 'rotation'. Assuming
// 'rotation' is composed of three rotations around X, then Y, then Z, returns
// the angle of the Z rotation.
template <typename T>
T GetYaw(const Eigen::Quaternion<T>& rotation) {
  const Eigen::Matrix<T, 3, 1> direction = rotation * Eigen::Matrix<T, 3, 1>::UnitX();
  return std::atan2(direction.y(), direction.x());
}

// Projects 'transform' onto the XY plane.
template <typename T>
Rigid2<T> Project2D(const Rigid3<T>& transform) {
  return Rigid2<T>(transform.translation().template head<2>(), GetYaw(transform));
}

// Embeds 'transform' into 3D space in the XY plane.
template <typename T>
Rigid3<T> Embed3D(const Rigid2<T>& transform) {
  return Rigid3<T>({transform.translation().x(), transform.translation().y(), T(0)},
                   Eigen::AngleAxis<T>(transform.rotation().angle(), Eigen::Matrix<T, 3, 1>::UnitZ()));
}

}  // namespace common
}  // namespace sensor_fusion
}  // namespace module
#endif  // CARTOGRAPHER_TRANSFORM_RIGID_TRANSFORM_H_
