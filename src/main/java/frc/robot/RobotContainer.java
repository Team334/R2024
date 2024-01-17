// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.ResetPose;
import frc.robot.commands.Shooter;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.ToggleSwerveOrient;
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


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    _swerveDrive.setDefaultCommand(new TeleopDrive(
      _swerveDrive,
      () -> -_driveFilterLeftY.calculate(_driveController.getLeftY()),
      () -> -_driveFilterLeftX.calculate(_driveController.getLeftX()),
      () -> -_driveFilterRightX.calculate(_driveController.getRightX())
    ));

    configureBindings();

    _swerveDrive.resetPose(_visionSubsystem.getBotpose());
  }

  private void configureBindings() {
    _driveController.R1().onTrue(new ToggleSwerveOrient(_swerveDrive));
    _driveController.L1().onTrue(new ResetGyro(_swerveDrive));
    _driveController.cross().onTrue(new ResetPose(_swerveDrive));
    _driveController.circle().whileTrue(new Shooter(_shooterSubsystem));
  }
}