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
  private static boolean _preloadShot = false;

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
      // new FeedActuate(intake, FeedMode.INTAKE).onlyIf(() -> _preloadShot).withTimeout(1),

      // Parallel command group that aims, revs, and squeezes note. ONLY APPLIES TO PRELOADED NOTE.
      new ParallelCommandGroup(
        new SpinShooter(shooter, ShooterState.SHOOT).withTimeout(2),
        new AutoAim(swerve, shooter, elevator, leds, () -> Presets.CLOSE_SHOOTER_ANGLE, () -> Presets.CLOSE_ELEVATOR_HEIGHT, this::headingPreset).withTimeout(3),
        new FeedActuate(intake, FeedMode.INTAKE).withTimeout(1)
      ).onlyIf(() -> !_preloadShot).andThen(() -> _preloadShot = true),

      new FeedActuate(intake, FeedMode.OUTTAKE).withTimeout(1),
      new SpinShooter(shooter, ShooterState.NONE, true)
    );
  }

  private double headingPreset() {
    return (UtilFuncs.GetAlliance() == Alliance.Red) ? 0 : 180;
  }
}
