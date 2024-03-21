// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Speeds;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class NoteAlign extends Command {
  private SwerveDriveSubsystem _swerve;
  private VisionSubsystem _vision;

  private DoubleSupplier _xSpeed;
  private DoubleSupplier _ySpeed;

  private PIDController _headingController = new PIDController(0, 0, 0);

  /** Creates a new NoteAlign. */
  public NoteAlign(SwerveDriveSubsystem swerve, VisionSubsystem vision, DoubleSupplier xSpeed, DoubleSupplier ySpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    _swerve = swerve;
    _vision = vision;

    _xSpeed = xSpeed;
    _ySpeed = ySpeed;

    addRequirements(_swerve, _vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double noteX = _vision.getNoteAngles()[0];
    
    double rotationVelocity = MathUtil.clamp(
      _headingController.calculate(noteX, 0),
      -Speeds.SWERVE_DRIVE_MAX_ANGULAR_SPEED,
      Speeds.SWERVE_DRIVE_MAX_ANGULAR_SPEED
    );

    _swerve.driveChassis(new ChassisSpeeds(
      _xSpeed.getAsDouble(),
      _ySpeed.getAsDouble(),
      rotationVelocity 
    ));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _swerve.driveChassis(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _headingController.atSetpoint();
  }
}
