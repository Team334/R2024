/*                                  Team 334                                  */
/* Copyright (c) 2024 Team 334. All Rights Reserved.                          */

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.NeoConfig;

/**
 * @author Elvis Osmanov
 * @author Peleh Liu
 * @author Cherine Soewingjo
 */
public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax _leftMotor =
      new CANSparkMax(Constants.CAN.SHOOTER_LEFT, MotorType.kBrushless);
  private final CANSparkMax _rightMotor =
      new CANSparkMax(Constants.CAN.SHOOTER_RIGHT, MotorType.kBrushless);

  private final RelativeEncoder _leftEncoder = _leftMotor.getEncoder();

  private final ArmFeedforward _angleFeed = new ArmFeedforward(0, 0, 0);
  private final PIDController _angleController =
      new PIDController(getAngle(), getVelocity(), getAngle());

  private final PIDController _shooterController =
      new PIDController(Constants.PID.SHOOTER_FLYWHEEL_KP, 0, 0);

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    NeoConfig.configureNeo(_leftMotor, true);
    NeoConfig.configureFollowerNeo(_rightMotor, _leftMotor, true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /** Returns true if the shooter is at the last desired height setpoint. */
  public boolean atDesiredAngle() {
    return _shooterController.atSetpoint();
  }

  /** Set the angle of the shooter in degrees. MUST be called repeatedly. */
  public void setAngle(double angleDegrees) {
    driveAngle(
        _angleFeed.calculate(Math.toRadians(getAngle()), 0)
            + _angleController.calculate(getAngle(), angleDegrees));
  }

  /** Get the angle of the shooter in degrees. */
  public double getAngle() {
    return 0;
  }

  /** Drives the angle motors at the desired percent output */
  public void driveAngle(double speed) {}

  /** Stops the angle motors completely, should be called with caution. */
  public void stopAngle() {}

  /** Get the velocity of the back wheel (left side) in m/s. */
  public double getVelocity() {
    double neo_rps = _leftEncoder.getVelocity() / 60;

    return (neo_rps / Constants.Physical.SHOOTER_GEAR_RATIO)
        * Constants.Physical.SHOOTER_FLYWHEEL_CIRCUMFERENCE;
  }

  /** Set the velocity of the back wheels in m/s. */
  public void setVelocity(double velocity) {
    double flywheel_output =
        (velocity / Constants.Speeds.SHOOTER_MAX_SPEED); // FEEDFORWARD (main output)
    double flywheel_pid =
        _shooterController.calculate(getVelocity(), velocity); // PID for distrubances

    // a similar controller setup can be found in SwerveModule
    _leftMotor.set(flywheel_output + flywheel_pid);
  }

  /** Spins the shooter forward. */
  public void spinShooter() {
    _leftMotor.set(-1.0);
  }

  /** Stops spinning the shooter. */
  public void stopShooter() {
    _leftMotor.set(0);
  }
}
