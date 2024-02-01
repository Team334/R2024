/*                                  Team 334                                  */
/*               Copyright (c) 2024 Team 334. All Rights Reserved.            */

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.commands.shooter.AutoAim;
import frc.robot.commands.shooter.SpinShooter;
import frc.robot.commands.swerve.BrakeSwerve;
import frc.robot.commands.swerve.PivotMotor;
import frc.robot.commands.swerve.ResetPose;
import frc.robot.commands.swerve.TeleopDrive;
import frc.robot.commands.swerve.ToggleSwerveOrient;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final VisionSubsystem _visionSubsystem = new VisionSubsystem();
  private final SwerveDriveSubsystem _swerveSubsystem = new SwerveDriveSubsystem(_visionSubsystem);
  private final ShooterSubsystem _shooterSubsystem = new ShooterSubsystem();
  private final ElevatorSubsystem _elevatorSubsystem = new ElevatorSubsystem();
  private final LEDSubsystem _ledSubsystem = new LEDSubsystem(Constants.Ports.LEDS, 14);

  // controllers (for driver and operator)
  private final CommandPS4Controller _driveController =
      new CommandPS4Controller(Constants.Ports.DRIVER_CONTROLLER);

  // slew rate limiters applied to joysticks
  private final SlewRateLimiter _driveFilterLeftX = new SlewRateLimiter(4);
  private final SlewRateLimiter _driveFilterLeftY = new SlewRateLimiter(4);
  private final SlewRateLimiter _driveFilterRightX = new SlewRateLimiter(4);
  private final SlewRateLimiter _driveFilterRightY = new SlewRateLimiter(4);

  // sendable chooser for auton commands
  private final SendableChooser<Command> _autonChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // TODO: should switch to regsiterCommands for more neatness
    NamedCommands.registerCommand("printHello", new PrintCommand("AUTON HELLO"));
    NamedCommands.registerCommand("waitCommand", new WaitCommand(3));
    NamedCommands.registerCommand("interruptSwerve", new BrakeSwerve(_swerveSubsystem, 3));
    NamedCommands.registerCommand(
        "speakerAim",
        new AutoAim(_ledSubsystem, _shooterSubsystem, _visionSubsystem, _swerveSubsystem));

    _swerveSubsystem.setDefaultCommand(
        new TeleopDrive(
            _swerveSubsystem,
            _ledSubsystem,
            () ->
                MathUtil.applyDeadband(
                    -_driveFilterLeftY.calculate(_driveController.getLeftY()), 0.1),
            () ->
                MathUtil.applyDeadband(
                    -_driveFilterLeftX.calculate(_driveController.getLeftX()), 0.1),
            () ->
                MathUtil.applyDeadband(
                    -_driveFilterRightX.calculate(_driveController.getRightX()), 0.1)));

    // _ledSubsystem.setDefaultCommand(new DefaultLED(_ledSubsystem));

    // _elevatorSubsystem.setDefaultCommand(new HoldElevator(_elevatorSubsystem));
    // _shooterSubsystem.setDefaultCommand(new HoldShooter(_shooterSubsystem));

    // configure trigger bindings
    configureBindings();

    _autonChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("AUTON CHOOSER", _autonChooser);
  }

  // to configure button bindings
  private void configureBindings() {
    _driveController.R1().onTrue(new ToggleSwerveOrient(_swerveSubsystem));
    _driveController.square().onTrue(new ResetPose(_swerveSubsystem));
    _driveController.circle().whileTrue(new SpinShooter(_shooterSubsystem));
    _driveController.cross().whileTrue(new BrakeSwerve(_swerveSubsystem));
    _driveController
        .L1()
        .whileTrue(
            new AutoAim(
                _shooterSubsystem,
                _ledSubsystem,
                _visionSubsystem,
                _swerveSubsystem,
                () ->
                    MathUtil.applyDeadband(
                        -_driveFilterLeftY.calculate(_driveController.getLeftY()), 0.1),
                () ->
                    MathUtil.applyDeadband(
                        -_driveFilterLeftX.calculate(_driveController.getLeftX()), 0.1)));

    // for testing velocity output (forward at 0.3 m/s), is it straight?
    // ...

    _driveController
        .triangle()
        .whileTrue(
            Commands.run(
                () -> {
                  _swerveSubsystem.driveChassis(new ChassisSpeeds(0.3, 0, 0));
                },
                _swerveSubsystem));

    _driveController
        .L2()
        .whileTrue(
            new PivotMotor(
                _ledSubsystem, _swerveSubsystem, true, () -> -_driveController.getLeftY()));

    _driveController
        .R2()
        .whileTrue(
            new PivotMotor(
                _ledSubsystem, _swerveSubsystem, false, () -> -_driveController.getLeftY()));
  }

  /**
   * @return The Command to schedule for auton.
   */
  public Command getAutonCommand() {
    _swerveSubsystem.fieldOriented =
        false; // make sure swerve is robot-relative for pathplanner to work

    return _autonChooser.getSelected();
  }
}
