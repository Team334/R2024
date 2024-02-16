/* Copyright (C) 2024 Team 334. All Rights Reserved.*/
package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Speeds;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;

/**
 * @author Elvis Osmanov
 * @author Cherine Soewingjo
 * @author Peleh Liu
 */
public class SpinShooter extends Command {
  private ShooterSubsystem _shooter;
  private ShooterState _state;

  public SpinShooter(ShooterSubsystem shooter, ShooterState state) {
    _shooter = shooter;
    _state = state;

    // NO SHOOTER REQUIREMENT TO NOT MESS WITH SHOOTER ANGLING COMMANDS
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _shooter.setShooterState(_state);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _shooter.setShooterState(ShooterState.NONE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
