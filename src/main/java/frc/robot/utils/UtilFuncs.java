/*                                  Team 334                                  */
/* Copyright (c) 2024 Team 334. All Rights Reserved.                          */

package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

/** Any utility functions are here. */
public final class UtilFuncs {
  /**
   * Control a motor controller with voltage by converting a voltage output into percent output with
   * a scale factor.
   *
   * @param volts The voltage output.
   * @return The percent output to set the controller to.
   */
  public static double FromVolts(double volts) {
    return volts / 12.0;
  }

  /**
   * The current alliance for the match from DS. If no value is successfully retrieved, null is returned.
   */
  public static DriverStation.Alliance GetCurrentAlliance() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get();
    }

    return null;
  }
}
