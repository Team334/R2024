/*                                  Team 334                                  */
/* Copyright (c) 2024 Team 334. All Rights Reserved.                          */

package frc.robot.utils;

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
}
