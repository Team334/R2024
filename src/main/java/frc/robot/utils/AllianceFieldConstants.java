/* Copyright (C) 2024 Team 334. All Rights Reserved.*/
package frc.robot.utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

/**
 * Dynamic field constants that at the start of the match are set based on the
 * alliance color.
 */
public class AllianceFieldConstants {
  /** The speaker's 3d pose. */
  public final Pose3d SPEAKER_POSE;

  /** Creates a new AllianceFieldConstants. */
  public AllianceFieldConstants() {
    Alliance alliance = UtilFuncs.GetAlliance();

    if (alliance == Alliance.Blue) {
      SPEAKER_POSE = Constants.FieldConstants.SPEAKER_POSE_BLUE;
    } else {
      SPEAKER_POSE = Constants.FieldConstants.SPEAKER_POSE_RED;
    }
  }
}
