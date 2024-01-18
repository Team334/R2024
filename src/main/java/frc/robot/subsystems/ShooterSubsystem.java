// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems; 

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {
  private CANSparkMax _leftMotor = new CANSparkMax(0, MotorType.kBrushless); 
  private CANSparkMax _rightMotor = new CANSparkMax(1, MotorType.kBrushless); 

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void spinMotor(){
    _leftMotor.set(-1);
    _rightMotor.set(1);
  }

  public void stopMotors(){
    _leftMotor.set(0);
    _rightMotor.set(0);
  }
}