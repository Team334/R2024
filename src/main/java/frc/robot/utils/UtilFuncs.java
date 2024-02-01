/* Copyright (C) 2024 Team 334. All Rights Reserved.*/
package frc.robot.utils;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Any utility functions are here. */
public final class UtilFuncs {
  /**
   * Control a motor controller with voltage by converting a voltage output into
   * percent output with a scale factor.
   *
   * @param volts
   *            The voltage output.
   * @return The percent output to set the controller to.
   */
  public static double FromVolts(double volts) {
    return volts / 12.0;
  }

  /**
   * The alliance for the match and DOES NOT CHANGE (same from the start). 
   */
  public static DriverStation.Alliance GetAlliance() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get();
    }

    return null;
  }
}
