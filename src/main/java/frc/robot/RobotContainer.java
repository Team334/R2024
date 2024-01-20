/*                                  Team 334                                  */
/* Copyright (c) 2024 Team 334. All Rights Reserved.                          */

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.commands.shooter.Shooter;
import frc.robot.commands.swerve.BrakeSwerve;
import frc.robot.commands.swerve.ResetGyro;
import frc.robot.commands.swerve.ResetPose;
import frc.robot.commands.swerve.TeleopDrive;
import frc.robot.commands.swerve.ToggleSwerveOrient;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final VisionSubsystem _visionSubsystem = new VisionSubsystem();
  private final SwerveDriveSubsystem _swerveDrive = new SwerveDriveSubsystem(_visionSubsystem);
  private final ShooterSubsystem _shooterSubsystem = new ShooterSubsystem();
  // controllers (for driver and operator)
  private final CommandPS4Controller _driveController = new CommandPS4Controller(Constants.Ports.DRIVER_CONTROLLER);

  // slew rate limiters applied to joysticks
  private final SlewRateLimiter _driveFilterLeftX = new SlewRateLimiter(4);
  private final SlewRateLimiter _driveFilterLeftY = new SlewRateLimiter(4);
  private final SlewRateLimiter _driveFilterRightX = new SlewRateLimiter(4);
  private final SlewRateLimiter _driveFilterRightY = new SlewRateLimiter(4);

  // sendable chooser for auton commands
  private final SendableChooser<Command> autonChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    Command brakeSwerve = new BrakeSwerve(_swerveDrive, 3);

    NamedCommands.registerCommand("printHello", new PrintCommand("AUTON HELLO"));
    NamedCommands.registerCommand("waitCommand", new WaitCommand(3));
    NamedCommands.registerCommand("brakeSwerve", brakeSwerve);

    _swerveDrive.setDefaultCommand(new TeleopDrive(
      _swerveDrive,
      () -> -_driveFilterLeftY.calculate(_driveController.getLeftY()),
      () -> -_driveFilterLeftX.calculate(_driveController.getLeftX()),
      () -> -_driveFilterRightX.calculate(_driveController.getRightX())
    ));

    // configure trigger bindings
    configureBindings();

    autonChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("AUTON CHOOSER", autonChooser);
  }

  // to configure button bindings
  private void configureBindings() {
    _driveController.R1().onTrue(new ToggleSwerveOrient(_swerveDrive));
    _driveController.square().onTrue(new ResetPose(_swerveDrive));
    _driveController.circle().whileTrue(new Shooter(_shooterSubsystem));
    _driveController.cross().whileTrue(new BrakeSwerve(_swerveDrive, 0));
  }

  /** @return The Command to schedule for auton. */
  public Command getAutonCommand() {
    _swerveDrive.fieldOriented = false; // make sure swerve is robot-relative for pathplanner to work

    return autonChooser.getSelected();
  }
}
