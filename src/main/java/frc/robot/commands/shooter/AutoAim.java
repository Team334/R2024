/*                                  Team 334                                  */
/* Copyright (c) 2024 Team 334. All Rights Reserved.                          */

package frc.robot.commands.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LEDStrip;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import java.util.function.DoubleSupplier;

/**
 * @author Elvis Osmanov
 * @author Peter Gutkovich
 * @author Cherine Soewingjo
 */
public class AutoAim extends Command {
  private final ShooterSubsystem _shooter;
  private final VisionSubsystem _vision;
  private final SwerveDriveSubsystem _swerve;
  private final LEDStrip _leds;

  private final DoubleSupplier _xSpeed;
  private final DoubleSupplier _ySpeed;

  private boolean _reachedSwerveHeading;
  private boolean _reachedShooterAngle;

  private boolean _runOnce = false;

  private PIDController _headingController =
      new PIDController(Constants.PID.SWERVE_HEADING_KP, 0, Constants.PID.SWERVE_HEADING_KD);

  /** Creates a new AutoAim. */
  public AutoAim(
      LEDStrip leds,
      ShooterSubsystem shooter,
      VisionSubsystem vision,
      SwerveDriveSubsystem swerve,
      DoubleSupplier xSpeed,
      DoubleSupplier ySpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    _leds = leds;
    _shooter = shooter;
    _vision = vision;
    _swerve = swerve;

    _xSpeed = xSpeed;
    _ySpeed = ySpeed;
  
    _headingController.setTolerance(2);
    _headingController.enableContinuousInput(-180, 180);

    addRequirements(_shooter, _vision, _swerve, _leds);
  }

  /** Creates an auton AutoAim that ends when it reaches the first setpoints. */
  public AutoAim(LEDStrip leds, ShooterSubsystem shooter, VisionSubsystem vision, SwerveDriveSubsystem swerve) {
    this(leds, shooter, vision, swerve, () -> 0, () -> 0);

    _runOnce = true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _reachedSwerveHeading = false;
    _reachedShooterAngle = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO: Vision or Pose estimator only?

    // double desiredShooterAngle = 0;
    // double desiredSwerveHeading = 0;

    // double currentSwerveHeading = _swerve.getHeading().getDegrees();

    // SmartDashboard.putBoolean("VISIBLE TAG", _vision.isApriltagVisible());

    // double[] visionAngles = _vision.anglesToSpeaker();

    // if (_vision.isApriltagVisible() && visionAngles != null) {
    //   desiredShooterAngle = visionAngles[1];
    //   desiredSwerveHeading = visionAngles[0];

    //   SmartDashboard.putNumberArray("VISION ANGLES", visionAngles);

    // } else {
    //   desiredShooterAngle = _swerve.anglesToSpeaker()[1];
    //   desiredSwerveHeading = _swerve.anglesToSpeaker()[0];
    // }

    // desiredSwerveHeading -= currentSwerveHeading;

    // SmartDashboard.putNumber("DESIRED HEADING 360", desiredSwerveHeading);

    // desiredSwerveHeading = MathUtil.angleModulus(Math.toRadians(desiredSwerveHeading));
    // desiredSwerveHeading = Math.toDegrees(desiredSwerveHeading);

    // _shooter.setAngle(desiredShooterAngle);

    // SmartDashboard.putNumber("DESIRED HEADING", desiredSwerveHeading);

    // desiredSwerveHeading = _swerve.speakerAngles()[0];

    double currentSwerveHeading = _swerve.getHeading().getDegrees();
    double desiredSwerveHeading = _swerve.speakerAngles()[0];

    SmartDashboard.putNumber("DESIRED SWERVE HEADING", desiredSwerveHeading);
    SmartDashboard.putNumber("SHOOTER ANGLE", _swerve.speakerAngles()[1]);

    double rotationVelocity =
        MathUtil.clamp(
            _headingController.calculate(currentSwerveHeading, desiredSwerveHeading),
            -Constants.Speeds.SWERVE_DRIVE_MAX_ANGULAR_SPEED * 2,
            Constants.Speeds.SWERVE_DRIVE_MAX_ANGULAR_SPEED * 2);

    _reachedSwerveHeading = _headingController.atSetpoint();
    _reachedShooterAngle = true; // TODO: make this actually use the shooter

    if (_reachedSwerveHeading) rotationVelocity = 0; // to prevent oscillation

    if (_reachedSwerveHeading && _reachedShooterAngle) {
      _leds.setColor(Constants.LEDColors.greenLEDs);
    } else {
      _leds.blink(Constants.LEDColors.orangeLEDs, Constants.LEDColors.nothingLEDs, 25);
    }
    
    _swerve.driveChassis(
        new ChassisSpeeds(
            _xSpeed.getAsDouble()
                * Constants.Speeds.SWERVE_DRIVE_MAX_SPEED
                * Constants.Speeds.SWERVE_DRIVE_COEFF,
            _ySpeed.getAsDouble()
                * Constants.Speeds.SWERVE_DRIVE_MAX_SPEED
                * Constants.Speeds.SWERVE_DRIVE_COEFF,
            rotationVelocity));
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
