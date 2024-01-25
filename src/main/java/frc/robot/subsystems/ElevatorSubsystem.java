/*                                  Team 334                                  */
/* Copyright (c) 2024 Team 334. All Rights Reserved.                          */

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.NeoConfig;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ElevatorSubsystem extends SubsystemBase {

  private final CANSparkMax _leftMotor = new CANSparkMax(Constants.CAN.ELEVATOR_LEFT, MotorType.kBrushless);
  private final CANSparkMax _rightMotor = new CANSparkMax(Constants.CAN.ELEVATOR_RIGHT, MotorType.kBrushless);

  public final ElevatorFeedforward elevatorFeed = new ElevatorFeedforward(0, Constants.FeedForward.ELEVATOR_KG, 0);

  /** Creates a new ElevatorSubsystem . */
  public ElevatorSubsystem() {
    NeoConfig.configureNeo(_leftMotor, true);
    NeoConfig.configureFollowerNeo(_leftMotor, _rightMotor, true);
  }

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
    //harry chen code maybe fix
    // setMotor(elevatorFeed.calculate(0));
  }

  public double getCurrentHeight() {
    return 0.00;
  }

  public void setMotor(double speed) {
    _leftMotor.set(speed);
  }

  public void stopMotor() {
    _leftMotor.set(0);
  }



}
