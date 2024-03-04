// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.Encoders;
import frc.robot.commands.intake.FeedActuate;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem.ActuatorState;
import frc.robot.subsystems.IntakeSubsystem.FeedMode;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;
import frc.robot.utils.UtilFuncs;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonShoot extends SequentialCommandGroup {
  /** Creates a new AutonShoot. */
  public AutonShoot(
    ShooterSubsystem shooter,
    LEDSubsystem leds,
    SwerveDriveSubsystem swerve,
    IntakeSubsystem intake
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoAim(shooter, leds, swerve),
      new WaitUntilCommand(() -> UtilFuncs.InRange(shooter.getVelocity(), Encoders.SHOOTER_SHOOT_VEL, 1)),
      new FeedActuate(intake, ActuatorState.STOWED, FeedMode.OUTTAKE).until(intake::atDesiredActuatorState),
      new WaitCommand(1),
      new SpinShooter(shooter, ShooterState.NONE).until(() -> true)
    );
  }
}
