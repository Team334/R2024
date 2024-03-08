/* Copyright (C) 2024 Team 334. All Rights Reserved.*/
package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
  private final SwerveDriveSubsystem _swerve;
  private final LEDSubsystem _leds;

  private final DoubleSupplier _xSpeed;
  private final DoubleSupplier _ySpeed;

  private boolean _reachedSwerveHeading;
  private boolean _reachedShooterAngle;

  private boolean _runOnce;

  private PIDController _headingController = new PIDController(Constants.PID.SWERVE_HEADING_KP, 0,
      Constants.PID.SWERVE_HEADING_KD);

  /** Creates a new AutoAim. */
  public AutoAim(
    ShooterSubsystem shooter, 
    LEDSubsystem leds, 
    SwerveDriveSubsystem swerve,
    DoubleSupplier xSpeed, 
    DoubleSupplier ySpeed
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    _leds = leds;
    _shooter = shooter;
    _swerve = swerve;

    _xSpeed = xSpeed;
    _ySpeed = ySpeed;

    _runOnce = false;

    _headingController.setTolerance(2);
    _headingController.enableContinuousInput(-180, 180);


    addRequirements(_swerve);
  }

  /** Creates an auton AutoAim that ends when it reaches the first setpoints. */
  public AutoAim(ShooterSubsystem shooter, LEDSubsystem leds, SwerveDriveSubsystem swerve) {
    this(shooter, leds, swerve, () -> 0, () -> 0);

    _runOnce = true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _reachedSwerveHeading = false;
    _reachedShooterAngle = true; // FOR NOW
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentSwerveHeading = _swerve.getHeading().getDegrees();

    double[] angles = _swerve.speakerAngles(Physical.ELEVATOR_LOWEST_HEIGHT /** + calculated elevator height */);

    double desiredSwerveHeading = angles[0];
    double desiredShooterAngle = angles[1];

    System.out.println(desiredShooterAngle);

    double rotationVelocity = MathUtil.clamp(
        _headingController.calculate(currentSwerveHeading, desiredSwerveHeading),
        -Constants.Speeds.SWERVE_DRIVE_MAX_ANGULAR_SPEED * 2,
        Constants.Speeds.SWERVE_DRIVE_MAX_ANGULAR_SPEED * 2);

    _reachedSwerveHeading = _headingController.atSetpoint();
    _reachedShooterAngle = _shooter.atDesiredAngle();

    if (_reachedSwerveHeading) rotationVelocity = 0; // to prevent oscillation

    // if (_reachedSwerveHeading && _reachedShooterAngle) {
    //   _leds.setColor(Constants.LEDColors.GREEN);
    // } else {
    //   _leds.blink(Constants.LEDColors.YELLOW, Constants.LEDColors.NOTHING, 0.2);
    // }

    _swerve.driveChassis(new ChassisSpeeds(
        _xSpeed.getAsDouble() * Constants.Speeds.SWERVE_DRIVE_MAX_SPEED * Constants.Speeds.SWERVE_DRIVE_COEFF,
        _ySpeed.getAsDouble() * Constants.Speeds.SWERVE_DRIVE_MAX_SPEED * Constants.Speeds.SWERVE_DRIVE_COEFF,
        rotationVelocity));
      
    _shooter.setAngle(desiredShooterAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _runOnce && _reachedSwerveHeading && _reachedShooterAngle;
  }
}
