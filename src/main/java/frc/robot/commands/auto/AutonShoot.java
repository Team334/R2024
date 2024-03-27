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
  /** If the note is intaked all the way to shoot. */
  public static boolean isIntaked = false;

  /** If the shooter is revved all the way to shoot. */
  public static boolean isRevved = false; 

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
          new SpinShooter(shooter, ShooterState.SHOOT, true).onlyIf(() -> !isRevved).andThen(new WaitCommand(2)),
          new FeedActuate(intake, FeedMode.INTAKE).onlyIf(() -> !isIntaked).withTimeout(1)
        ),

        new AutoAim(swerve, shooter, elevator, leds)
      ),

      new FeedActuate(intake, FeedMode.OUTTAKE).withTimeout(1),
      new SpinShooter(shooter, ShooterState.IDLE, true)
    );
  }

  /** Resets this command for re-testing auton. */
  public static void reset() {
    isIntaked = false;
    isRevved  = false;
  }

  private double headingPreset() {
    return (UtilFuncs.GetAlliance() == Alliance.Red) ? 0 : 180;
  }
}
