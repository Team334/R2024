/*                                  Team 334                                  */
/* Copyright (c) 2024 Team 334. All Rights Reserved.                          */

package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * @author Elvis Osmanov
 */
public class ResetGyro extends Command {
  private SwerveDriveSubsystem _swerveDrive;

  /** Creates a new ResetGyro. */
  public ResetGyro(SwerveDriveSubsystem swerveDrive) {
    // Use addRequirements() here to declare subsystem dependencies.
    _swerveDrive = swerveDrive;
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("RESET GYRO COMMAND");

    _swerveDrive.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
