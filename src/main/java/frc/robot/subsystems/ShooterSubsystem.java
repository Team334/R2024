/* Copyright (C) 2024 Team 334. All Rights Reserved.*/
package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.UtilFuncs;
import frc.robot.utils.configs.NeoConfig;

/**
 * @author Elvis Osmanov
 * @author Peleh Liu
 * @author Cherine Soewingjo
 */
public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax _leftMotor = new CANSparkMax(Constants.CAN.SHOOTER_LEFT, MotorType.kBrushless);
  private final CANSparkMax _rightMotor = new CANSparkMax(Constants.CAN.SHOOTER_RIGHT, MotorType.kBrushless);
  private final CANSparkMax _angleMotor = new CANSparkMax(Constants.CAN.SHOOTER_ANGLE, MotorType.kBrushless);

  private final RelativeEncoder _leftEncoder = _leftMotor.getEncoder();
  private final RelativeEncoder _angleEncoder = _angleMotor.getEncoder();


  private final ArmFeedforward _angleFeed = new ArmFeedforward(0, 0, 0); // nothing for now
  private final PIDController _angleController = new PIDController(0.05, 0, 0.01);
   

  private final PIDController _shooterController = new PIDController(Constants.PID.SHOOTER_FLYWHEEL_KP, 0, 0);

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    NeoConfig.configureNeo(_leftMotor, false);
    NeoConfig.configureFollowerNeo(_rightMotor, _leftMotor, true);

    NeoConfig.configureNeo(_angleMotor, true);

    _angleEncoder.setPosition(0);
    
    // soft limits
    _angleMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    _angleMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    
    _angleMotor.setSoftLimit(SoftLimitDirection.kForward, (float) (90 * Constants.Physical.SHOOTER_ANGLE_GEAR_RATIO / 360));
    _angleMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) (-90 * Constants.Physical.SHOOTER_ANGLE_GEAR_RATIO / 360));

    _angleController.setTolerance(0.5);
    SmartDashboard.putData("ANGLE PID", _angleController);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("SHOOTER ANGLE ENCODER", _angleEncoder.getPosition());
    SmartDashboard.putNumber("SHOOTER ANGLE", getAngle());
  }

  /** Returns true if the shooter is at the last desired height setpoint. */
  public boolean atDesiredAngle() {
    return _shooterController.atSetpoint();
  }

  /** Set the angle of the shooter in degrees. MUST be called repeatedly. */
  public void setAngle(double angleDegrees) {
    double pid = MathUtil.clamp(
      _angleController.calculate(getAngle(), _angleController.getSetpoint()), // test
      -Constants.Speeds.SHOOTER_ANGLE_MAX_SPEED,
      Constants.Speeds.SHOOTER_ANGLE_MAX_SPEED
    );

    driveAngle(pid);
  }

  /** Get the angle of the shooter in degrees. */
  public double getAngle() {
    return _angleEncoder.getPosition() / Constants.Physical.SHOOTER_ANGLE_GEAR_RATIO * 360;
  }

  /**
   * Drives the angle motors at the desired percent output (feedforward is
   * included).
   */
  public void driveAngle(double speed) {
    _angleMotor.set(UtilFuncs.FromVolts(_angleFeed.calculate(Math.toRadians(getAngle()), 0)) + speed);
  }

  /** Stops the shooter's angular movement. */
  public void stopAngle() {
    driveAngle(0);
  }

  // /** Get the velocity of the back wheel (left side) in m/s. */
  // public double getVelocity() {
  //   double neo_rps = _leftEncoder.getVelocity() / 60;

  //   double number = (neo_rps / Constants.Physical.SHOOTER_GEAR_RATIO)
  //       * Constants.Physical.SHOOTER_FLYWHEEL_CIRCUMFERENCE;

  //   if (number < 0) {
  //     number = 0;
  //   }

  //   return number;
  // }

  // /** Set the velocity of the back wheels in m/s. */
  // public void setVelocity(double velocity) {
  //   double flywheel_output = (velocity / Constants.Speeds.SHOOTER_MAX_SPEED); // FEEDFORWARD (main output)
  //   double flywheel_pid = _shooterController.calculate(getVelocity(), velocity); // PID for distrubances

  //   // a similar controller setup can be found in SwerveModule
  //   _leftMotor.set(flywheel_output + flywheel_pid);
  // }

  public void spinShooter(double speed) {
    _leftMotor.set(speed);
  }

  /** Spins the shooter forward. */
  public void spinShooter() {
    _leftMotor.set(1.0);
  }

  /** Stops spinning the shooter. */
  public void stopShooter() {
    _leftMotor.set(0);
  }
}
