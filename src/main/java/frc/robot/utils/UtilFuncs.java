/* Copyright (C) 2024 Team 334. All Rights Reserved.*/
package frc.robot.utils;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FieldConstants;
import frc.robot.utils.helpers.AllianceHelper;

/** Any utility functions are here. */
public final class UtilFuncs {
  private static DoubleSupplier _distanceSupplier;

  /**
   * Supply the ShootFast function with a distance supplier.
   * 
   * @param distance Distance supplier - chassis distance from speaker shot point.
   */
  public static void ShootFast(DoubleSupplier distance) {
    _distanceSupplier = distance;
  }

  /**
   * Whether to shoot fast based on distance of bot from speaker shot point.
   */
  public static boolean ShootFast() {
    if (_distanceSupplier.getAsDouble() > FieldConstants.SHOOTER_SLOW_THRESHOLD) return true;
    
    return false;
  }

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
   * See if value is within range of another value.
   * 
   * @param val The value to check.
   * @param desired The desired value.
   * @param range The range.
   * 
   * @return If in range.
   */
  public static boolean InRange(double val, double desired, double range) {
    if (Math.abs(desired - val) <= range) return true;
    return false;
  }

  /**
   * The alliance for the match (shortcut for AllianceHelper).
   */
  public static Alliance GetAlliance() {
    return AllianceHelper.getInstance().getAlliance();
  }
}
