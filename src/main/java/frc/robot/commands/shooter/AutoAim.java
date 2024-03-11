/* Copyright (C) 2024 Team 334. All Rights Reserved.*/
package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.Physical;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utils.UtilFuncs;

/**
 * @author Elvis Osmanov
 * @author Peter Gutkovich
 * @author Cherine Soewingjo
 */
public class AutoAim extends Command {
  private final ShooterSubsystem _shooter;
  private final ElevatorSubsystem  _elevator;
  private final SwerveDriveSubsystem _swerve;
  private final LEDSubsystem _leds;

  private final DoubleSupplier _xSpeed;
  private final DoubleSupplier _ySpeed;

  private boolean _reachedSwerveHeading;
  private boolean _reachedShooterAngle;
  private boolean _reachedElevatorHeight;

  private double _desiredSwerveHeading = 0;
  private double _desiredShooterAngle = 0;
  private double _desiredElevatorHeight = 0;

  private boolean _runOnce;
  private boolean _overrideDesired;

  private PIDController _headingController = new PIDController(Constants.PID.SWERVE_HEADING_KP, 0,
      Constants.PID.SWERVE_HEADING_KD);

  /**
   * Constructs an AutoAim that does NOT finish when reaching setpoints.
   * It calculates angle/height/heading setpoints.
   * 
   * (USE FOR TELEOP)
   * 
   * @param shooter Shooter subsystem.
   * @param elevator Elevator subsystem.
   * @param leds Led subsystem.
   * @param swerve Swerve subsystem.
   * @param xSpeed X joystick speed.
   * @param ySpeed Y joystick speed.
   */
  public AutoAim(
    ShooterSubsystem shooter,
    ElevatorSubsystem elevator,
    LEDSubsystem leds, 
    SwerveDriveSubsystem swerve,
    DoubleSupplier xSpeed,
    DoubleSupplier ySpeed
  ) {
    _leds = leds;
    _shooter = shooter;
    _elevator = elevator;
    _swerve = swerve;

    _xSpeed = xSpeed;
    _ySpeed = ySpeed;

    _runOnce = false;
    _overrideDesired = false;

    _headingController.setTolerance(2);
    _headingController.enableContinuousInput(-180, 180);

    addRequirements(_swerve, _shooter, _elevator);
  }

  /**
   * Constructs an AutoAim that finishes when it reaches its initial setpoints.
   * It calculates angle/height/heading setpoints.
   * 
   * (USE FOR AUTON)
   * 
   * @param shooter Shooter subsystem.
   * @param elevator Elevator subsystem.
   * @param leds Led subsystem.
   * @param swerve Swerve subsystem.
   */
  public AutoAim(ShooterSubsystem shooter, ElevatorSubsystem elevator, LEDSubsystem leds, SwerveDriveSubsystem swerve) {
    this(shooter, elevator, leds, swerve, () -> 0, () -> 0);

    _runOnce = true;
  }

  /**
   * Constructs an AutoAim that finishes when it reaches its initial setpoint. 
   * It does NOT calculate angle/height/heading setpoints, and overrides them with supplied setpoints instead.
   * 
   * (USE FOR AUTON)
   * 
   * @param shooterAngle Desired shooter angle.
   * @param elevatorHeight Desired elevator height.
   * @param swerveHeading Desired swerve heading.
   */
  public AutoAim(
    ShooterSubsystem shooter, 
    ElevatorSubsystem elevator, 
    LEDSubsystem leds, 
    SwerveDriveSubsystem swerve,
    double shooterAngle,
    double elevatorHeight,
    double swerveHeading
  ) {
    this(shooter, elevator, leds, swerve);

    _desiredShooterAngle = shooterAngle;
    _desiredElevatorHeight = elevatorHeight;
    _desiredSwerveHeading = swerveHeading;

    _overrideDesired = true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _reachedSwerveHeading = false;
    _reachedShooterAngle = false;
    _reachedElevatorHeight = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("AIMING");
    
    double currentSwerveHeading = _swerve.getHeading().getDegrees();

    if (!_overrideDesired) {
      double[] setpoints = _swerve.speakerSetpoints();

      _desiredSwerveHeading = setpoints[0];
      _desiredShooterAngle = setpoints[1];
      _desiredElevatorHeight = setpoints[2];
    }
    
    double rotationVelocity = MathUtil.clamp(
      _headingController.calculate(currentSwerveHeading, _desiredSwerveHeading),
      -Constants.Speeds.SWERVE_DRIVE_MAX_ANGULAR_SPEED,
      Constants.Speeds.SWERVE_DRIVE_MAX_ANGULAR_SPEED
    );

    _reachedSwerveHeading = _headingController.atSetpoint();
    _reachedShooterAngle = _shooter.atDesiredAngle();
    _reachedElevatorHeight = _elevator.atDesiredHeight();

    if (_reachedSwerveHeading) rotationVelocity = 0;

    SmartDashboard.putNumber("Y", _desiredSwerveHeading);

    // if (_reachedSwerveHeading && _reachedShooterAngle) {
    //   _leds.setColor(Constants.LEDColors.GREEN);
    // } else {
    //   _leds.blink(Constants.LEDColors.YELLOW, Constants.LEDColors.NOTHING, 0.2);
    // }

    _swerve.driveChassis(
      new ChassisSpeeds(
        _xSpeed.getAsDouble() * Constants.Speeds.SWERVE_DRIVE_MAX_SPEED * Constants.Speeds.SWERVE_DRIVE_COEFF,
        _ySpeed.getAsDouble() * Constants.Speeds.SWERVE_DRIVE_MAX_SPEED * Constants.Speeds.SWERVE_DRIVE_COEFF,
        rotationVelocity
      )
    );

    _shooter.setAngle(_desiredShooterAngle);
    _elevator.setHeight(_desiredElevatorHeight);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _swerve.driveChassis(new ChassisSpeeds());
    _elevator.stopElevator();;
    _shooter.stopAngle();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _runOnce && _reachedSwerveHeading && _reachedShooterAngle && _reachedElevatorHeight;
  }
}
