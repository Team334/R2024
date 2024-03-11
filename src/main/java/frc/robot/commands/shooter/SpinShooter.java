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
  private final ShooterSubsystem _shooter;
  private final ShooterState _state;
  private final boolean _instant;

  public SpinShooter(ShooterSubsystem shooter, ShooterState state, boolean instant) {
    _shooter = shooter;
    _state = state;
    _instant = instant;

    // NO SHOOTER SUBSYSTEM REQUIREMENT TO NOT MESS WITH SHOOTER ANGLING COMMANDS
  }

  /** SpinShooter that never ends. */
  public SpinShooter(ShooterSubsystem shooter, ShooterState state) {
    this(shooter, state, false);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _shooter.setShooterState(_state);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("SPINNING");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _instant;
  }
}
