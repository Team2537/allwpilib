// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "GameField.h"

std::optional<frc::Rectangle2d> wpi::GameField::getZone(std::string_view name) const {
  auto it = m_zones.find(name);
  if(it == m_zones.end()){
    return std::nullopt;
  }

  return std::make_optional(*it);
}