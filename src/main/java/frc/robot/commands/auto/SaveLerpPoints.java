// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.UtilFuncs;

public class SaveLerpPoints extends Command {
  private final ShooterSubsystem _shooter;
  private final ElevatorSubsystem _elevator;

  private ArrayList<double[]> shooterPoints = new ArrayList<>();
  private ArrayList<double[]> elevatorPoints = new ArrayList<>();

  private final String SHOOTER_LERP_NAME = "SHOOTER_DISTANCE_ANGLE";
  private final String ELEVATOR_LERP_NAME = "ELEVATOR_DISTANCE_HEIGHT";

  /** Creates a new SaveLerpPoints. */
  public SaveLerpPoints(
    ShooterSubsystem shooter,
    ElevatorSubsystem elevator
  ) {
    _shooter = shooter;
    _elevator = elevator;

    addRequirements(_shooter, _elevator);
  }

  private void showLine(String className, double[] point) {
    System.out.println(className + ".put(" + Double.toString(point[0]) + ", " + Double.toString(point[1]) + ");");
  }

  /**
   * Creates the code that can be copied into Constants.java for lerp points.
   */
  public void showCode() {
    for (double[] point : shooterPoints) {
      showLine(SHOOTER_LERP_NAME, point);
    }

    for (double[] point : elevatorPoints) {
      showLine(ELEVATOR_LERP_NAME, point);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double shotDistance = UtilFuncs.ShotVector().getNorm();

    double[] shooterLerp = {shotDistance, _shooter.getAngle()};
    double[] elevatorLerp = {shotDistance, _elevator.getHeight()};

    shooterPoints.add(shooterLerp);
    elevatorPoints.add(elevatorLerp);
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
