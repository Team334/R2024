/*                                  Team 334                                  */
/* Copyright (c) 2024 Team 334. All Rights Reserved.                          */

package frc.robot.utils;

import edu.wpi.first.wpilibj.RobotController;

/** Any utility functions are here. */
public final class UtilFuncs {
  /**
   * Applies deadband to a certain value.
   *
   * @param val - The value to deadband.
   * @param deadband - The deadband to apply.
   * @return The new value with deadband applied.
   */
  public static double ApplyDeadband(double val, double deadband) {
    if (Math.abs(val) > deadband) {
      return val;
    }

    return 0;
  }

  /**
   * Control a motor controller with voltage by converting a voltage output into percent output with a scale factor.
   * 
   * @param volts The voltage output.
   * @return The percent output to set the controller to.
   */
  public static double FromVolts(double volts) {
    return volts / 12.0;
  }
}
