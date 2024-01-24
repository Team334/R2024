/*                                  Team 334                                  */
/* Copyright (c) 2024 Team 334. All Rights Reserved.                          */

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ElevatorSubsystem extends SubsystemBase {

  //harry chen code maybe fix
  private final ElevatorFeedforward _elevatorFeed = new ElevatorFeedforward(0, 0, 0);

  /** Creates a new ElevatorSubsystem . */
  public ElevatorSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //harry chen code maybe fix
    _elevatorFeed.calculate(0);
  }
}
