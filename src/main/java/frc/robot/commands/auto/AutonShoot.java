 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.concurrent.locks.Condition;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Presets;
import frc.robot.commands.intake.FeedActuate;
import frc.robot.commands.shooter.SpinShooter;
import frc.robot.commands.swerve.BrakeSwerve;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem.FeedMode;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;
import frc.robot.utils.UtilFuncs;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonShoot extends SequentialCommandGroup {
  /** 
   * Indicates whether the note in the intake can be shot right away, this means that the note is already squished, and
   * that the shooter is already revved enough.
   */
  public static boolean canShoot = false;

  /** Creates a new AutonShoot. */
  public AutonShoot(
    ShooterSubsystem shooter,
    ElevatorSubsystem elevator,
    LEDSubsystem leds,
    SwerveDriveSubsystem swerve,
    IntakeSubsystem intake
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        // This command will squeeze the note while revving up the shooter. It will only run if the note can't be shot right away. 
        new ParallelCommandGroup(
          new SpinShooter(shooter, ShooterState.SHOOT, true).andThen(new WaitCommand(2)),
          new FeedActuate(intake, FeedMode.INTAKE).withTimeout(1)
        ).onlyIf(() -> !canShoot).andThen(() -> canShoot = true),

        new AutoAim(swerve, shooter, elevator, leds)
      ),

      new FeedActuate(intake, FeedMode.OUTTAKE).withTimeout(1),
      new SpinShooter(shooter, ShooterState.IDLE, true)
    );
  }

  private double headingPreset() {
    return (UtilFuncs.GetAlliance() == Alliance.Red) ? 0 : 180;
  }
}
