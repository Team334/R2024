// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/**
 * @author Lucas Ou
 * @author Alex Reyes
 */
public class VisionSubsystem extends SubsystemBase {
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable limelight = inst.getTable("limelight");

  private final Field2d _field = new Field2d();

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double[] botpose = new double[6];

    Optional<Alliance> alliance = DriverStation.getAlliance();

    if (alliance.isPresent()) {
      if (alliance.get() == Alliance.Red) {
        botpose = limelight.getEntry("botpose_wpired").getDoubleArray(new double[6]);
      }
      if (alliance.get() == Alliance.Blue) {
        botpose = limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
      }
    }

    SmartDashboard.putNumber("retrieved botpose", botpose[0]);
    
    _field.setRobotPose(getBotPose(botpose));

    SmartDashboard.putData(_field);
  }

  public Pose2d getBotPose(double[] botposeArray) {
    double botposeX = botposeArray[0];
    double botposeY = botposeArray[1];
    double botposeYaw = Math.toRadians(botposeArray[5]);

    Rotation2d botposeRotation = new Rotation2d(botposeYaw);
    Pose2d botpose = new Pose2d(botposeX, botposeY, botposeRotation);
    // System.out.println(botposeRotation);

    return(botpose);
  }
}
