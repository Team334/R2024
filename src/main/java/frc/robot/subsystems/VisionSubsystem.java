/* Copyright (C) 2024 Team 334. All Rights Reserved.*/
package frc.robot.subsystems;

import java.util.Optional;

import javax.swing.text.html.Option;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.utils.helpers.LimelightHelper;

/**
 * @author Lucas Ou
 * @author Alex Reyes
 * @author Peter Gutkovich
 */
public class VisionSubsystem extends SubsystemBase {
  private final LimelightHelper _limelight = LimelightHelper.getInstance();

  private final MedianFilter _xFilter = new MedianFilter(20);
  private final MedianFilter _yFilter = new MedianFilter(20);
  private final MedianFilter _yawFilter = new MedianFilter(20);

  // private double[] _botpose = new double[6];

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
  }

  @Override
  public void periodic() {
  }

  /**
   * Returns the latency from the last time data was sent from the limelight. This
   * should be used in the pose estimator.
   */
  public double getLatency() {
    double tl = _limelight.getEntry("tl").getDouble(0);
    double cl = _limelight.getEntry("cl").getDouble(0);

    return Timer.getFPGATimestamp() - (tl / 1000.0) - (cl / 1000.0);
  }

  /**
   * Returns the "wpiblue" botpose of the robot from the limelight.
   *
   * @return An Optional of Pose2d which is necessary if no valid (at least 2 tags, good distance) data is found from
   * the limelight.
   * 
   * @see Optional
   */
  public Optional<Pose2d> getBotpose() {
    // return Optional.empty();

    if (!isApriltagVisible()) return Optional.empty();

    NetworkTableEntry botpose_entry = _limelight.getEntry("botpose_wpiblue");

    if (!botpose_entry.exists()) return Optional.empty();

    double[] botpose_array = botpose_entry.getDoubleArray(new double[11]);

    int tags = (int) botpose_array[7];
    double distance = botpose_array[9];

    if (tags < 2) return Optional.empty();
    if (distance > FieldConstants.TAG_DISTANCE_THRESHOLD) return Optional.empty();

    double botposeX = _xFilter.calculate(botpose_array[0]); // to get rid of the weird origin outlier
    double botposeY = _yFilter.calculate(botpose_array[1]); // to get rid of the weird origin outlier
    double botposeYaw = _yawFilter.calculate(botpose_array[5]); // to get rid of weird 0 heading outlier
    Rotation2d botposeRotation = Rotation2d.fromDegrees(botposeYaw);

    Pose2d botpose2D = new Pose2d(botposeX, botposeY, botposeRotation);

    return Optional.of(botpose2D);
  }

  /**
   * Tells whether limelight data is valid or not. For the data to be valid, the limelight 
   * must be looking at at least one apriltag, and the apriltag must be in the specific distance range.
   */
  public boolean isValid() {
    if (!isApriltagVisible()) return false;

    JsonNode tags = _limelight.getTags();
    
    for (JsonNode tag : tags) {
      double distance = ((ArrayNode) tag.get("t6t_cs")).get(2).asDouble();
      if (distance <= FieldConstants.TAG_DISTANCE_THRESHOLD) {
        return true;
      }
    }
    
    return false;
  }

  /** Return a boolean for whether a tag is seen. */
  public boolean isApriltagVisible() {
    double tv = _limelight.getEntry("tv").getDouble(0);

    if (tv == 0) {
      return false;
    }

    if (tv == 1) {
      return true;
    }

    return false;
  }

  /**
   * Return a boolean for whether the desired tag is seen.
   *
   * @param ID
   *            The id of the desired tag.
   */
  public boolean isApriltagVisible(int ID) {
    if (!isApriltagVisible())
      return false;

    return _limelight.getTag(ID) != null;
  }

  /**
   * Return tx and ty angle offsets from a desired tag.
   *
   * @param ID
   *            The id of the desired tag.
   * @return A double array [tx, ty]. Null is returned if no tags are visible at
   *         all.
   */
  public double[] tagAngleOffsets(int ID) {
    if (!isApriltagVisible(ID))
      return null;

    JsonNode tag = _limelight.getTag(ID);

    double tx = tag.get("tx").asDouble();
    double ty = tag.get("ty").asDouble();

    double[] angles = {tx, ty};

    return angles;
  }
}
