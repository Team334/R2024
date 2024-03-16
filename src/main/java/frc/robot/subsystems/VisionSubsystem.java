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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.utils.UtilFuncs;
import frc.robot.utils.helpers.LimelightHelper;

/**
 * @author Lucas Ou
 * @author Alex Reyes
 * @author Peter Gutkovich
 */
public class VisionSubsystem extends SubsystemBase {
  private final LimelightHelper _limelight = LimelightHelper.getInstance();

  private final MedianFilter _xFilter = new MedianFilter(20); // TODO: change?
  private final MedianFilter _yFilter = new MedianFilter(20);
  private final MedianFilter _yawFilter = new MedianFilter(20);

  private boolean _shouldResetPose = true;

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
   * Returns the NT "botpose_wpiblue" as an array from the limelight if it is valid (good distance). 
   * 
   * @return NT "botpose_wpiblue" limelight topic.
   * 
   * @see Optional
   */
  public Optional<double[]> getValidNTEntry() {
    if (!isApriltagVisible()) return Optional.empty();

    NetworkTableEntry botpose_entry = _limelight.getEntry("botpose_wpiblue");
    if (!botpose_entry.exists()) return Optional.empty();

    double[] botpose_array = botpose_entry.getDoubleArray(new double[11]);

    double distance = botpose_array[9];
    if (distance > FieldConstants.TAG_DISTANCE_THRESHOLD) return Optional.empty();

    return Optional.of(botpose_array);
  }

  /**
   * If the limelight is in perfect condition with the apriltags to reset the robot's pose.
   * 
   * @return The pose to reset to (USE GYRO FOR HEADING).
   */
  public Optional<Pose2d> resetPose() {
    Optional<double[]> botpose = getValidNTEntry();

    if (botpose.isEmpty()) { _shouldResetPose = true; return Optional.empty(); }
    // if (botpose.get()[7] < 2) return Optional.empty(); // tag count?

    int centerTag = UtilFuncs.GetAlliance() == Alliance.Red ? FieldConstants.SPEAKER_TAG_RED : FieldConstants.SPEAKER_TAG_BLUE;
    int offsetTag = UtilFuncs.GetAlliance() == Alliance.Red ? FieldConstants.SPEAKER_TAG_RED_OFF : FieldConstants.SPEAKER_TAG_BLUE_OFF; 

    if (isApriltagVisible(centerTag) || isApriltagVisible(offsetTag)) {
      if (!_shouldResetPose) return Optional.empty();
      _shouldResetPose = false;
      return getBotpose();
    }

    _shouldResetPose = true;

    return Optional.empty();
  }

  /**
   * Returns the "wpiblue" Pose2d of the robot from the limelight.
   *
   * @return An Optional of Pose2d which is necessary if no valid (at least 2 tags, good distance) data is found from
   * the limelight.
   * 
   * @see Optional
   */
  public Optional<Pose2d> getBotpose() {
    Optional<double[]> botpose = getValidNTEntry();
    if (botpose.isEmpty()) return Optional.empty();

    double[] botpose_array = botpose.get();

    double botposeX = _xFilter.calculate(botpose_array[0]); // to get rid of the weird origin outlier
    double botposeY = _yFilter.calculate(botpose_array[1]); // to get rid of the weird origin outlier
    double botposeYaw = _yawFilter.calculate(botpose_array[5]); // to get rid of weird 0 heading outlier
    Rotation2d botposeRotation = Rotation2d.fromDegrees(botposeYaw);

    Pose2d botpose2D = new Pose2d(botposeX, botposeY, botposeRotation);

    return Optional.of(botpose2D);
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
