/* Copyright (C) 2024 Team 334. All Rights Reserved.*/
package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
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
  private final boolean _runOnce;

  public SpinShooter(ShooterSubsystem shooter, ShooterState state, boolean runOnce) {
    _shooter = shooter;
    _state = state;
    _runOnce = runOnce;

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
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!_runOnce && _state == ShooterState.SHOOT) {
      _shooter.setShooterState(ShooterState.IDLE);
    } 
    
    if (!_runOnce && _state != ShooterState.SHOOT) {
      _shooter.setShooterState(ShooterState.NONE);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _runOnce /**|| (_state == ShooterState.AMP && _shooter.holdNote())*/;
  }
}
