
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.SetElevator;
import frc.robot.commands.intake.FeedActuate;
import frc.robot.commands.shooter.SpinShooter;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem.ActuatorState;
import frc.robot.subsystems.IntakeSubsystem.FeedMode;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoAmp extends SequentialCommandGroup {
  /** Creates a new AutoAmp. */
  public AutoAmp(
    ShooterSubsystem shooter,
    IntakeSubsystem intake
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new FeedActuate(intake, FeedMode.INTAKE).withTimeout(0.3),
      new SpinShooter(shooter, ShooterState.SLOW, false).withTimeout(1.00),
      new FeedActuate(intake, ActuatorState.STOWED, FeedMode.OUTTAKE).withTimeout(0.5)
      // new SpinShooter(shooter, ShooterState.SLOW, false).withTimeout(0.1)
    );
  }
}
