/*                                  Team 334                                  */
/* Copyright (c) 2024 Team 334. All Rights Reserved.                          */

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

/**
 * @author Elvis Osmanov
 * @author Peleh Liu
 * @author Cherine Soewingjo
 */
public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax _leftMotor = new CANSparkMax(Constants.CAN.SHOOTER_LEFT, MotorType.kBrushless);
  private final CANSparkMax _rightMotor = new CANSparkMax(Constants.CAN.SHOOTER_RIGHT, MotorType.kBrushless);

  private final RelativeEncoder _leftEncoder = _leftMotor.getEncoder();

  private final PIDController _shooterController = new PIDController(Constants.Physical.SHOOTER_PID_KP, 0, 0); 


  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    _rightMotor.follow(_leftMotor,true);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void spinMotor() {
    _leftMotor.set(-1.0);
  }

  public void stopMotors() {
    _leftMotor.set(0);
  }

  /** Get the velocity of the back wheel (left side) in m/s. */
  public double getShooterVelocity(){
    double neo_rps = _leftEncoder.getVelocity() / 60;

    return(neo_rps / Constants.Physical.SHOOTER_GEAR_RATIO ) * Constants.Physical.SHOOTER_FLYWHEEL_CIRCUMFERENCE;
  }

  /** Set the velocity of the back wheels in m/s. */
  public void setVelocity(double velocity){
    double flywheel_output = (velocity / Constants.Speeds.SHOOTER_MAX_SPEED); // FEEDFORWARD (main output)
    double flywheel_pid = _shooterController.calculate(getShooterVelocity(), velocity); // PID for distrubances

    _leftMotor.set(flywheel_output + flywheel_pid);
  }
}
