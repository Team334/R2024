/*                                  Team 334                                  */
/* Copyright (c) 2024 Team 334. All Rights Reserved.                          */

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * @author Lucas Ou
 * @author Alex Reyes
 */
public class VisionSubsystem extends SubsystemBase {
  private final NetworkTableInstance _inst = NetworkTableInstance.getDefault();
  private final NetworkTable _limelight = _inst.getTable("limelight");

  private double[] _botpose = new double[6];

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // SmartDashboard.putNumber("retrieved botpose", getBotpose().getTranslation().getX());

    // _field.setRobotPose(getBotpose());

    // SmartDashboard.putData("Limelight Field", _field);
  }

  public Pose2d get_botpose() {
    _botpose = _limelight.getEntry("botpose_wpiblue").getDoubleArray(_botpose);

    double botposeX = _botpose[0];
    double botposeY = _botpose[1];
    double botposeYaw = Math.toRadians(_botpose[5]);

    Rotation2d botposeRotation = new Rotation2d(botposeYaw);
    Pose2d botPose2D = new Pose2d(botposeX, botposeY, botposeRotation);
    // System.out.println(botposeRotation);

    return botPose2D;
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
}
