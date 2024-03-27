/* Copyright (C) 2024 Team 334. All Rights Reserved.*/
package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.auto.AutonShoot;
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

  private final Timer _revTimer;

  public SpinShooter(ShooterSubsystem shooter, ShooterState state, boolean runOnce) {
    _shooter = shooter;
    _state = state;
    _runOnce = runOnce;

    _revTimer = new Timer();

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

    _revTimer.start();

    // If not set to shoot state then not revved (AUTON ONLY).
    if (_runOnce && _state != ShooterState.SHOOT) _revTimer.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    AutonShoot.isRevved = _revTimer.hasElapsed(2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!_runOnce && _state != ShooterState.AMP) _shooter.setShooterState(ShooterState.IDLE);
    if (!_runOnce && _state == ShooterState.AMP) _shooter.setShooterState(ShooterState.NONE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _runOnce /**|| (_state == ShooterState.AMP && _shooter.holdNote())*/;
  }
}
