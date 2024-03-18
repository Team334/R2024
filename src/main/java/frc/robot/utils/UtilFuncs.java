/* Copyright (C) 2024 Team 334. All Rights Reserved.*/
package frc.robot.utils;

import java.util.function.DoubleSupplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FieldConstants;
import frc.robot.utils.helpers.AllianceHelper;

/** Any utility functions are here. */
public final class UtilFuncs {
  private static DoubleSupplier _shotDistanceSupplier;
  private static DoubleSupplier _distanceSupplier;
  
  private static AprilTagFieldLayout _field;

  /**
   * Loads the AprilTag field.
   */
  public static void LoadField() {
    _field = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
    _field.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
  }

  /**
   * Gets the necessary speaker pose for auto-aim.
   * 
   * @return 3d point to aim at.
   */
  public static Pose3d GetSpeakerPose() {
    Pose3d pose;

    if (GetAlliance() == Alliance.Red) {
      pose =  _field.getTagPose(FieldConstants.SPEAKER_TAG_RED).get();
    } else {
      pose = _field.getTagPose(FieldConstants.SPEAKER_TAG_BLUE).get();
    }

    return new Pose3d(
      new Translation3d(pose.getX(), pose.getY(), pose.getZ() + FieldConstants.SPEAKER_TAG_OFFSET),
      new Rotation3d()
    );
  }

  public static void ShotDistance(DoubleSupplier distance) {
    _shotDistanceSupplier = distance;
  }

  public static double ShotDistance() {
    return _shotDistanceSupplier.getAsDouble();
  }

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
