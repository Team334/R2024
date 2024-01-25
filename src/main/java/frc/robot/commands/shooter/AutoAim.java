/*                                  Team 334                                  */
/* Copyright (c) 2024 Team 334. All Rights Reserved.                          */

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutoAim extends Command {
  private ShooterSubsystem _shooter;
  private VisionSubsystem _vision;
  private SwerveDriveSubsystem _swerve;

  /** Creates a new AutoAim. */
  public AutoAim(ShooterSubsystem shooter, VisionSubsystem vision, SwerveDriveSubsystem swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    _shooter = shooter;
    _vision = vision;
    _swerve = swerve;
    addRequirements(_shooter, _vision, _swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double shooterAngle = 0;

    if (_vision.isApriltagVisible()) {
      shooterAngle = _vision.shooterAngleToSpeaker();
    } else {
      shooterAngle = _swerve.shooterAngleToSpeaker();
    }

    SmartDashboard.putNumber("ANGLE", shooterAngle);

    _shooter.setAngle(shooterAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
