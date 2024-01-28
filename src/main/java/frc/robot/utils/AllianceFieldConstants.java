/*                                  Team 334                                  */
/* Copyright (c) 2024 Team 334. All Rights Reserved.                          */

package frc.robot.utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

/** Dynamic field constants that at the start of the match are set based on the alliance color. */
public class AllianceFieldConstants {
    /**
     * The field layout.
     * @see AprilTagFieldLayout
     */
    public final AprilTagFieldLayout APRILTAG_LAYOUT = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    /** The speaker's 3d pose. */
    public final Pose3d SPEAKER_POSE;

    /** Creates a new AllianceFieldConstants. */
    public AllianceFieldConstants() {
        APRILTAG_LAYOUT.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

        Alliance alliance = UtilFuncs.GetCurrentAlliance();

        if (alliance == Alliance.Blue) {
            SPEAKER_POSE = Constants.FieldConstants.SPEAKER_POSE_BLUE;
        } else {
            SPEAKER_POSE = Constants.FieldConstants.SPEAKER_POSE_RED;
        }
    }
}
