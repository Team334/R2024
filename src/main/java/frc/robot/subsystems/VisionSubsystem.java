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
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable limelight = inst.getTable("limelight");

  private double[] botpose = new double[6];

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // SmartDashboard.putNumber("retrieved botpose", getBotpose().getTranslation().getX());

    // _field.setRobotPose(getBotpose());

    // SmartDashboard.putData("Limelight Field", _field);
  }

  public Pose2d getBotpose() {

    botpose = limelight.getEntry("botpose_wpiblue").getDoubleArray(botpose);

    double botposeX = botpose[0];
    double botposeY = botpose[1];
    double botposeYaw = Math.toRadians(botpose[5]);

    Rotation2d botposeRotation = new Rotation2d(botposeYaw);
    Pose2d botPose2D = new Pose2d(botposeX, botposeY, botposeRotation);
    // System.out.println(botposeRotation);

    return botPose2D;
  }

  public boolean isApriltagVisible() {
    double tv = limelight.getEntry("tv").getDouble(0);
    if (tv == 0) {
      return false;
    }
    if (tv == 1) {
      return true;
    }
    return false;
  }
}
