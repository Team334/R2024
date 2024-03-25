/* Copyright (C) 2024 Team 334. All Rights Reserved.*/
package frc.robot.utils;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Constants;
import frc.robot.Constants.PID;
import frc.robot.utils.configs.TalonFXConfig;

/**
 * @author Peter Gutkovich
 * @author Elvis Osmanov
 */
public class SwerveModule {
  private final TalonFX _driveMotor;
  private final TalonFX _rotationMotor;

  private final PIDController _driveController;
  private final PIDController _rotationController;

  private final SimpleMotorFeedforward _driveFeedforward = new SimpleMotorFeedforward(
      Constants.FeedForward.MODULE_DRIVE_KS, Constants.FeedForward.MODULE_DRIVE_KV);

  private final CANcoder _encoder;

  private final String _name;

  /**
   * Represents a single swerve module built with talons for rotation and drive
   * control, and a cancoder for angle.
   *
   * @param name
   *            - The name of this module.
   * @param driveMotorId
   *            - CAN ID of drive motor.
   * @param rotationMotorId
   *            - CAN ID of rotation motor.
   * @param encoderId
   *            - CAN ID of cancoder.
   */
  public SwerveModule(String name, int driveMotorId, int rotationMotorId, int encoderId) {
    _driveMotor = new TalonFX(driveMotorId);
    _rotationMotor = new TalonFX(rotationMotorId);
    _encoder = new CANcoder(encoderId);

    _name = name;

    _driveController = new PIDController(PID.MODULE_DRIVE_KP, 0, 0);

    _rotationController = new PIDController(PID.MODULE_ROTATION_KP, 0, 0);
    _rotationController.enableContinuousInput(-180, 180);

    TalonFXConfig.configureFalcon(_driveMotor, false);
    TalonFXConfig.configureFalcon(_rotationMotor, true);

    // TODO: make sure this works
    CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs();

    currentConfig.SupplyCurrentLimit = 60;
    currentConfig.SupplyCurrentThreshold = 30;
    currentConfig.SupplyTimeThreshold = 0.9;

    currentConfig.StatorCurrentLimit = 120;

    currentConfig.SupplyCurrentLimitEnable = false;
    currentConfig.StatorCurrentLimitEnable = false;

    _driveMotor.getConfigurator().apply(currentConfig);
  }

  /** Display's this module's info on SmartDashboard through a supplied builder. */
  public void displayInfo(SendableBuilder builder) {
    builder.addDoubleProperty(_name + " Angle", () -> getAngle(), null);
    builder.addDoubleProperty(_name + " Velocity", () -> getDriveVelocity(), null);

    builder.addDoubleProperty(_name + " Drive Supply Current", () -> _driveMotor.getSupplyCurrent().getValueAsDouble(), null);
    builder.addDoubleProperty(_name + " Drive Stator Current", () -> _driveMotor.getStatorCurrent().getValueAsDouble(), null);
  }

  /**
   * Get the talons belonging to this module as an array.
   *
   * @return [drive talon, rotation talon]
   */
  public TalonFX[] getTalons() {
    TalonFX[] fxes = new TalonFX[2];
    fxes[0] = _driveMotor;
    fxes[1] = _rotationMotor;
    return fxes;
  }

  /** Set the percentage output of the drive motor. */
  public void drive(double speed) {
    _driveMotor.set(speed);
  }

  /** Set the percentage output of the rotation motor. */
  public void rotate(double speed) {
    _rotationMotor.set(speed);
  }

  /** Get the absolute angle of the module as an int (-180 to 180 degrees). */
  public double getAngle() {
    return _encoder.getAbsolutePosition().getValueAsDouble() * 2 * 180;
    // return Double.valueOf(_encoder.getAbsolutePosition().getValueAsDouble() * 2 * 180).intValue(); // ctre update
  }

  /** Get the velocity of the drive wheel (meters per second). */
  public double getDriveVelocity() {
    double talon_rps = _driveMotor.getRotorVelocity().getValueAsDouble(); // ctre update

    // return the speed of the drive wheel itself (talon rps times gear ratio time
    // wheel size)
    // in
    // m/s
    return (talon_rps / Constants.Physical.SWERVE_DRIVE_GEAR_RATIO)
        * Constants.Physical.SWERVE_DRIVE_WHEEL_CIRCUMFERENCE;
  }

  /**
   * Get the position of this module (distance traveled by wheel + angle).
   *
   * @see SwerveModulePosition
   */
  public SwerveModulePosition getPosition() {
    double talon_rotations = _driveMotor.getPosition().getValueAsDouble();
    double distance = (talon_rotations / Constants.Physical.SWERVE_DRIVE_GEAR_RATIO)
        * Constants.Physical.SWERVE_DRIVE_WHEEL_CIRCUMFERENCE;

    return new SwerveModulePosition(distance, Rotation2d.fromDegrees(getAngle()));
  }

  /**
   * Set the state of this module. This function must be called repeatedly for the
   * state to be set.
   *
   * @see SwerveModuleState
   */
  public void setState(SwerveModuleState state) {
    // current system for setting the state of a module
    // rotation: pure pid control
    // velocity: feedforward control mainly along with pid control for small
    // disturbances

    state = SwerveModuleState.optimize(state, new Rotation2d(Math.toRadians(getAngle())));

    double speed = MathUtil.clamp(state.speedMetersPerSecond, -Constants.Speeds.SWERVE_DRIVE_MAX_SPEED,
        Constants.Speeds.SWERVE_DRIVE_MAX_SPEED);

    double rotation_pid = MathUtil.clamp(_rotationController.calculate(getAngle(), state.angle.getDegrees()),
        -0.150, 0.150);

    // double drive_feedforward = (speed / Constants.Speeds.SWERVE_DRIVE_MAX_SPEED);
    double drive_feedforward = UtilFuncs.FromVolts(_driveFeedforward.calculate(speed));
    double drive_pid = _driveController.calculate(getDriveVelocity(), speed);

    drive_pid = 0;

    rotate(rotation_pid);
    drive(drive_feedforward + drive_pid);
  }

  /**
   * Get the state of this module.
   *
   * @see SwerveModuleState
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromDegrees(getAngle()));
  }
}
