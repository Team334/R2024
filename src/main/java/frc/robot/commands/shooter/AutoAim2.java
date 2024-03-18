// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.elevator.SetElevator;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoAim2 extends ParallelCommandGroup {
  /** 
   * Creates a new AutoAim2.
   * 
   * @param swerve The swerve drive.
   * @param shooter The shooter.
   * @param elevator The elevator.
   * 
   * @param swerveHeading The heading to set the chassis to (only valid if overrideCalculated is true).
   * @param shooterAngle The angle to set the shooter to (only valid if overrideCalculated is true).
   * @param elevatorHeight The height to set the elevator to (only valid if overrideCalculated is true).
   * 
   * @param overrideCalculated Ignore calculated setpoints, and override them with different supplied setpoints.
   * @param runOnce Run the command for only a single set of aiming setpoints.
  */
  public AutoAim2(
    SwerveDriveSubsystem swerve,
    ShooterSubsystem shooter,
    ElevatorSubsystem elevator,
    DoubleSupplier swerveHeading,
    DoubleSupplier shooterAngle,
    DoubleSupplier elevatorHeight,
    boolean overrideCalculated,
    boolean runOnce
  ) {
    DoubleSupplier heading;
    DoubleSupplier angle;
    DoubleSupplier height;

    if (overrideCalculated) {
      heading = swerveHeading;
      angle = shooterAngle;
      height = elevatorHeight;
    } else {
      heading = () -> 0; // TODO: function that'll return auto-aimed angle
      angle = () -> 0;  // TODO: function that'll return auto-aimed height
      height = () -> 0; // TODO: function that'll return auto-aimed heading
    }

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // swerve heading command here
      new SetShooter(shooter, angle).until(() -> (runOnce && shooter.atDesiredAngle())),
      new SetElevator(elevator, height).until(() -> (runOnce && elevator.atDesiredHeight()))
      // led command here
    );
  }
}
