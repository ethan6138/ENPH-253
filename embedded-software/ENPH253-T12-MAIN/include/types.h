#pragma once

#include <type_traits>

// shared enums + types
template <typename T> constexpr auto operator+(T a) noexcept {
  return static_cast<std::underlying_type_t<T>>(a);
}

struct Vector3D {
  float x{};
  float y{};
  float z{};
};

namespace ArmParameters {

enum class ArmPreset {
  RockPickup,
  SolarPanel,
  HabitatPiece,
  DropOff,
  Home,
  DriveHome,
  end
};

struct ArmAngles {
  float alpha{90};
  float beta{};
  float gamma{};
};

} // namespace ArmParameters

namespace Imu {
struct IMUAngles {
  float roll{};  // roll
  float pitch{}; // pitch
};
struct IMUAccel {
  float x{};
  float y{};
  float z{};
};
} // namespace Imu

namespace Pose {
enum class ResetPoint { Start, RampBottom, RampTop, end };
enum class Position {
  ROCK1,
  ROCK2,
  ROCK3,
  ROCK4,
  ROCK5,
  ROCK6,
  SOLARPANEL,
  END
};

struct PoseState {
  // from the top left corner
  float x{};   // cm, +x is right on dash
  float y{};   // cm, +y is down on dash
  float yaw{}; // deg, measured from +y axis.
};
} // namespace Pose

namespace Wheels {
enum class WheelSide { LEFT, RIGHT, end };

struct RpmTargets {
  float rpmLeft{};
  float rpmRight{};
};

} // namespace Wheels

namespace K230 {
  enum class AlignTargetClass { ROCK, TELETUBBY, LOOP };
}