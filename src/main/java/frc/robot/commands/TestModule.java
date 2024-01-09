// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class TestModule extends CommandBase {
  private final DoubleSupplier _getLeft;
  private final DoubleSupplier _getRight;

  private final SwerveDriveSubsystem _swerveDrive;

  /** Creates a new TestModule. */
  public TestModule(SwerveDriveSubsystem swerveDrive, DoubleSupplier getLeft, DoubleSupplier getRight) {
    // Use addRequirements() here to declare subsystem dependencies.

    _swerveDrive = swerveDrive;
    
    _getLeft = getLeft;
    _getRight = getRight;

    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _swerveDrive.stateTest(new SwerveModuleState(
      SmartDashboard.getNumber("State Speed", 0),
      new Rotation2d(Math.toRadians(SmartDashboard.getNumber("State Angle", 0)))
    ));
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