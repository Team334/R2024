// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;


/**
 * @author Elvis Osmanov
 * @author Harry Chen
 */

public class AngleShooter extends Command {
  /** Creates a new AngleShooter. */
  private ShooterSubsystem _shooter;
  private DoubleSupplier _angle;

  public AngleShooter(ShooterSubsystem shooter, DoubleSupplier angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    _shooter = shooter;
    _angle = angle;
    addRequirements(_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(
    
  ) {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
