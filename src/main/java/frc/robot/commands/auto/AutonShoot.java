 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.intake.FeedActuate;
import frc.robot.commands.shooter.SpinShooter;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem.FeedMode;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonShoot extends SequentialCommandGroup {
  /** Creates a new AutonShoot. */
  public AutonShoot(
    ShooterSubsystem shooter,
    ElevatorSubsystem elevator,
    SwerveDriveSubsystem swerve,
    IntakeSubsystem intake
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // This command will squeeze the note and rev up the shooter if needed, all while auto-aiming.
      new ParallelCommandGroup(
        new SpinShooter(shooter, ShooterState.SHOOT, true).andThen(new WaitUntilCommand(shooter::isRevved)),
        new FeedActuate(intake, FeedMode.INTAKE).withTimeout(1).onlyIf(() -> !intake.hasNoteAuton())
        // ,
        // new AutoAim(swerve, shooter, elevator)
      ),

      new FeedActuate(intake, FeedMode.OUTTAKE).withTimeout(0.5)
      // new SpinShooter(shooter, ShooterState.IDLE, true)
    );
  }
}
