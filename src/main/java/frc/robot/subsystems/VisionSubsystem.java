/*                                  Team 334                                  */
/* Copyright (c) 2024 Team 334. All Rights Reserved.                          */

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

/**
 * @author Lucas Ou
 * @author Alex Reyes
 */
public class VisionSubsystem extends SubsystemBase {
  private final NetworkTableInstance _inst = NetworkTableInstance.getDefault();
  private final NetworkTable _limelight = _inst.getTable("limelight");

  private final ObjectMapper _objectMapper = new ObjectMapper();

  // private double[] _botpose = new double[6];

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // SmartDashboard.putNumber("retrieved botpose", getBotpose().getTranslation().getX());

    System.out.println(isApriltagVisible(6));

    // _field.setRobotPose(getBotpose());

    // SmartDashboard.putData("Limelight Field", _field);
  }

  public Optional<Pose2d> get_botpose() {
    NetworkTableEntry botpose_entry = _limelight.getEntry("botpose_wpiblue");

    if (!botpose_entry.exists()) {
      return Optional.empty();
    } else {
      double[] botpose_array = botpose_entry.getDoubleArray(new double[6]);

      double botposeX = botpose_array[0];
      double botposeY = botpose_array[1];
      double botposeYaw = Math.toRadians(botpose_array[5]);
      Rotation2d botposeRotation = new Rotation2d(botposeYaw);

      Pose2d botPose2D = new Pose2d(botposeX, botposeY, botposeRotation);

      return Optional.of(botPose2D);
    }
  }

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

  public boolean isApriltagVisible(int ID) {
    if (!isApriltagVisible()) return false;

    String jsonString = _limelight.getEntry("json").getString("");

    JsonNode tags;

    try { tags = _objectMapper.readTree(jsonString).get("Results").get("Fiducial"); }
    catch (Exception e) { throw new Error("IDKK"); }

    for (JsonNode tag : tags) {
      if (tag.get("fID").asInt() == ID) {
        return true;
      }
    }

    return false;
  }

  public double[] tagAngleOffsets() {
    double tx = _limelight.getEntry("tx").getDouble(0);
    double ty = _limelight.getEntry("ty").getDouble(0);

    double[] angles = {tx, ty};

    return angles;
  }

  public double shooterAngleToSpeaker() {
    return tagAngleOffsets()[0];
  }
}
