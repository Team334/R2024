package frc.robot.utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

/** Dynamic field constants that at the start of the match are set based on the alliance color. */
public class AllianceFieldConstants {
    public final AprilTagFieldLayout APRILTAG_LAYOUT = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    /** The speaker's tag ID. */
    public final int SPEAKER_TAG;

    /** The speaker's 2d pose. */
    public final Pose2d SPEAKER_POSE;

    /** Creates a new AllianceFieldConstants. */
    public AllianceFieldConstants() {
        APRILTAG_LAYOUT.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

        Alliance alliance = UtilFuncs.GetCurrentAlliance();

        if (alliance == Alliance.Blue) {
            SPEAKER_TAG = Constants.FieldConstants.SPEAKER_TAG_BLUE;
            SPEAKER_POSE = Constants.FieldConstants.SPEAKER_POSE_BLUE;
        } else {
            SPEAKER_TAG = Constants.FieldConstants.SPEAKER_TAG_RED;
            SPEAKER_POSE = Constants.FieldConstants.SPEAKER_POSE_RED;
        }
    }
}
