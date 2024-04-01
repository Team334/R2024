/* Copyright (C) 2024 Team 334. All Rights Reserved.*/
package frc.robot.utils;

import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import frc.robot.utils.helpers.AllianceHelper;

/** Any utility functions are here. */
public final class UtilFuncs {
  private static AprilTagFieldLayout _field;
  private static Supplier<Translation2d> _shotVectorSupplier;

  private static Pose3d SPEAKER_RED_POSE;
  private static Pose3d SPEAKER_BLUE_POSE;

  /**
   * Converts limelight pose data into wpilib "Pose2d".
   * 
   * @see Pose2d
   * 
   * @param botpose The "wpi_blue" botpose from the limelight.
   */
  public static Pose2d ToPose(double[] botpose) {
    return new Pose2d(
      botpose[0],
      botpose[1],
      Rotation2d.fromDegrees(botpose[5])
    );
  }
  
  /**
   * Loads the AprilTag field.
   */
  public static void LoadField() {
    try {
      _field = AprilTagFieldLayout.loadFromResource("/edu/wpi/first/apriltag/2024-crescendo.json");
    } catch (Exception e) {
      throw new Error(e.getMessage());
    }

    _field.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

    SPEAKER_RED_POSE = _field.getTagPose(FieldConstants.SPEAKER_TAG_RED).get();
    SPEAKER_BLUE_POSE = _field.getTagPose(FieldConstants.SPEAKER_TAG_BLUE).get();
  }

  /**
   * Gets the necessary speaker pose for auto-aim.
   * 
   * @return 3d point to aim at.
   */
  public static Pose3d GetSpeakerPose() {
    Pose3d pose;

    if (GetAlliance() == Alliance.Red) {
      pose =  SPEAKER_RED_POSE;
    } else {
      pose = SPEAKER_BLUE_POSE;
    }

    return new Pose3d(
      new Translation3d(pose.getX(), pose.getY(), pose.getZ() + FieldConstants.SPEAKER_TAG_OFFSET),
      new Rotation3d()
    );
  }

  /**
   * Supply UtilFuncs with a shot vector supplier.
   * 
   * @param shotVectorSupplier A supplier that returns the vector from the chassis to the shot point.
   */
  public static void ShotVector(Supplier<Translation2d> shotVectorSupplier) {
    _shotVectorSupplier = shotVectorSupplier;
  }

  /**
   * The vector between the chassis pose and the specific shot point.
   */
  public static Translation2d ShotVector() {
    return _shotVectorSupplier.get();
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
