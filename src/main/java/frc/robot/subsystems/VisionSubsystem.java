/* Copyright (C) 2024 Team 334. All Rights Reserved.*/
package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Limelights;
import frc.robot.utils.helpers.LimelightHelper;
import frc.robot.utils.helpers.LimelightHelper.PoseEstimate;

/**
 * @author Lucas Ou
 * @author Alex Reyes
 * @author Peter Gutkovich
 */
public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {}

  @Override
  public void periodic() {
    Optional<double[]> noteAngles = getNoteAngles();

    SmartDashboard.putBoolean("SEES NOTE(S)", getNoteAngles().isPresent());

    if (noteAngles.isPresent()) {
      SmartDashboard.putNumber("NOTE TX", noteAngles.get()[0]);
      SmartDashboard.putNumber("NOTE TY", noteAngles.get()[1]);
    }
  }

  /** Return a boolean for whether a tag is seen by main cam. */
  public boolean isApriltagVisible() {
    return LimelightHelper.getTV(Limelights.MAIN);
  }

  /** Return a boolean for whether a note is seen by intake cam. */
  public boolean isNoteVisible() {
    return LimelightHelper.getTV(Limelights.INTAKE);
  }

  /**
   * Returns the NT "botpose_wpiblue" as an array from the limelight if tags are visible. 
   * 
   * @return NT "botpose_wpiblue" limelight topic. This uses megatag 2.
   * 
   * @see Optional
   */
  public Optional<PoseEstimate> getBotposeBlue() {
    if (!isApriltagVisible()) return Optional.empty();

    PoseEstimate botpose_blue = LimelightHelper.getBotPoseEstimate_wpiBlue_MegaTag2(Limelights.MAIN);

    return Optional.of(botpose_blue);
  }

  /**
   * Returns the angles between the intake limelight and the note closest to it.
   * 
   * @return {xAngle, yAngle}
   */
  public Optional<double[]> getNoteAngles() {
    if (!isNoteVisible()) return Optional.empty();

    double[] angles = {
      LimelightHelper.getTX(Limelights.INTAKE),
      LimelightHelper.getTY(Limelights.INTAKE)
    };

    return Optional.of(angles);
  }
}
