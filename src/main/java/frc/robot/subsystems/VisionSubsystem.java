/* Copyright (C) 2024 Team 334. All Rights Reserved.*/
package frc.robot.subsystems;

import java.util.Optional;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.helpers.LimelightHelper;

/**
 * @author Lucas Ou
 * @author Alex Reyes
 * @author Peter Gutkovich
 */
public class VisionSubsystem extends SubsystemBase {
  private final LimelightHelper _main = new LimelightHelper("limelight-main");
  private final LimelightHelper _intake = new LimelightHelper("limelight-intake");

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {}

  @Override
  public void periodic() {
    Optional<double[]> noteAngles = getNoteAngles();

    if (noteAngles.isPresent()) {
      SmartDashboard.putNumber("NOTE TX", noteAngles.get()[0]);
      SmartDashboard.putNumber("NOTE TY", noteAngles.get()[1]);
    }
  }

  /**
   * Returns the latency from the last time data was sent from the main limelight. This
   * should be used in the pose estimator.
   */
  public double getLatency() {
    double tl = _main.getEntry("tl").getDouble(0);
    double cl = _main.getEntry("cl").getDouble(0);

    return Timer.getFPGATimestamp() - (tl / 1000.0) - (cl / 1000.0);
  }

  /** Return a boolean for whether a tag is seen by main cam. */
  public boolean isApriltagVisible() {
    double tv = _main.getEntry("tv").getDouble(0);

    if (tv == 0) {
      return false;
    }

    if (tv == 1) {
      return true;
    }

    return false;
  }

  /** Return a boolean for whether a note is seen by intake cam. */
  public boolean isNoteVisible() {
    double tv = _intake.getEntry("tv").getDouble(0);

    if (tv == 0) {
      return false;
    }

    if (tv == 1) {
      return true;
    }

    return false;
  }

  /**
   * Returns the NT "botpose_wpiblue" as an array from the limelight if tags are visible. 
   * 
   * @return NT "botpose_wpiblue" limelight topic.
   * 
   * @see Optional
   */
  public Optional<double[]> getBotposeBlue() {
    if (!isApriltagVisible()) return Optional.empty();

    NetworkTableEntry botpose_entry = _main.getEntry("botpose_wpiblue");
    if (!botpose_entry.exists()) return Optional.empty();

    double[] botpose_array = botpose_entry.getDoubleArray(new double[11]);

    return Optional.of(botpose_array);
  }

  /**
   * Returns the angles between the intake limelight and the note closest to it.
   * 
   * @return {xAngle, yAngle}
   */
  public Optional<double[]> getNoteAngles() {
    if (!isNoteVisible()) return Optional.empty();

    ArrayNode targets = _intake.getNeuralTargets();
    JsonNode target = targets.get(0);

    if (target == null) return Optional.empty();

    double[] angles = {
      target.get("tx").asDouble(0),
      target.get("ty").asDouble(0)
    };

    return Optional.of(angles);
  }
}
