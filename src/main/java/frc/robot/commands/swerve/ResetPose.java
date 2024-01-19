/*                                  Team 334                                  */
/* Copyright (c) 2024 Team 334. All Rights Reserved.                          */

package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * @author Quazi Hossain
 */
public class ResetPose extends Command {
  private SwerveDriveSubsystem _swerveDrive;

  /** Creates a new ResetPose. */
  public ResetPose(SwerveDriveSubsystem swerveDrive) {
    _swerveDrive = swerveDrive;
    addRequirements(_swerveDrive);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _swerveDrive.resetPose(new Pose2d());

    System.out.println("RESETTING POSE");
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
