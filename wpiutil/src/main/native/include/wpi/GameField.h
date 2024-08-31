// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/geometry/Transform2d.h"
#include "frc/geometry/Translation2d.h"
#include "frc/geometry/Rectangle2d.h"
#include "units/length.h"
#include "units/math.h"

#include <optional>
#include <string_view>

namespace wpi {
  /**
   * Represents the field of play for robotics competitions
   */
  class GameField {
    public:
      constexpr bool Contains(const frc::Translation2d& point) const {
        return m_shape.Contains(point);
      }

      constexpr frc::Translation2d FlipX(const frc::Translation2d& point) const {
        units::meters_t x = m_shape.XWidth() - point.X();
        return frc::Translation2d(x, point.Y());
      }

      constexpr frc::Translation2d FlipY(const frc::Translation2d& point) const {
        units::meters_t y = m_shape.YWidth() - point.Y();
        return frc::Translation2d(point.X(), y);
      }

      /**
       * Gets a zone by name.
       * 
       * @param name The name of the zone to get.
       * @return An optional containing the zone, or an empty one if none is found
       */
      std::optional<Rectangle2d> getZone(std::string_view name) const;

    private:
      frc::Rectangle2d m_shape;
      std::unordered_map<std::string, frc::Rectangle2d> m_zones;
      std::unordered_map<std::string, PointOfInterest> m_pois;
  };

  struct PointOfInterest {
    std::string_view name;
    frc::Translation2d position;
  }
}
