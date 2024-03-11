// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.concurrent.locks.Condition;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Presets;
import frc.robot.commands.intake.FeedActuate;
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
  private boolean _isAimed = false;

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
        new SpinShooter(shooter, ShooterState.SHOOT).unless(() -> shooter.isState(ShooterState.SHOOT)).withTimeout(1.5),
        new AutoAim(shooter, elevator, leds, swerve, Presets.CLOSE_SHOOTER_ANGLE, Presets.CLOSE_ELEVATOR_HEIGHT, this::headingPreset).onlyIf(
          () -> !_isAimed
        ).withTimeout(3).andThen(() -> _isAimed = true),
        new FeedActuate(intake, FeedMode.INTAKE).unless(() -> intake.isFeedMode(FeedMode.INTAKE)).withTimeout(1)
      ),
      // pre-shooting (rev up if needed, while auto aiming, while feeding in for squish)
      // new WaitUntilCommand(() -> UtilFuncs.InRange(shooter.getVelocity(), Encoders.SHOOTER_SHOOT_VEL, 1)),
      new FeedActuate(intake, FeedMode.OUTTAKE).withTimeout(0.5),
      new SpinShooter(shooter, ShooterState.NONE, true)
    );
  }

  private double headingPreset() {
    return (UtilFuncs.GetAlliance() == Alliance.Red) ? 0 : 180;
  }
}
