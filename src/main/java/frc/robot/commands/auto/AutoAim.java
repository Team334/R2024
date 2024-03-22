// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.elevator.SetElevator;
import frc.robot.commands.shooter.SetShooter;
import frc.robot.commands.swerve.SetHeading;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoAim extends ParallelCommandGroup {
  /** 
   * Creates a new AutoAim.
   * 
   * (THIS IS THE MAIN CONSTRUCTOR, USEABLE CONSTRUCTORS ARE BELOW)
   * 
   * @param swerve The swerve drive.
   * @param shooter The shooter.
   * @param elevator The elevator.
   * @param leds The led strip.
   * 
   * @param xSpeed The x speed of the drive joystick.
   * @param ySpeed The y speed of the drive joystick.
   * 
   * @param swerveHeading The heading to set the chassis to.
   * @param shooterAngle The angle to set the shooter to.
   * @param elevatorHeight The height to set the elevator to.
   * 
   * @param runOnce Run the command for only a single set of aiming setpoints.
  */
  private AutoAim(
    SwerveDriveSubsystem swerve,
    ShooterSubsystem shooter,
    ElevatorSubsystem elevator,
    LEDSubsystem leds,
    DoubleSupplier xSpeed,
    DoubleSupplier ySpeed,
    DoubleSupplier swerveHeading,
    DoubleSupplier shooterAngle,
    DoubleSupplier elevatorHeight,
    boolean runOnce
  ) {
    handleInterrupt(() -> SmartDashboard.putBoolean("AUTOAIM REACHED", true));

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      Commands.runOnce(() -> SmartDashboard.putBoolean("AUTOAIM REACHED", false)),
      new SetHeading(swerve, xSpeed, ySpeed, swerveHeading, runOnce),
      new SetShooter(shooter, shooterAngle, runOnce),
      new SetElevator(elevator, elevatorHeight, runOnce)
      // led command here
    );
  }

  /** 
   * Creates a new AutoAim2. <strong>(CALCULATED TELEOP)</strong>
   * 
   * This command will auto-aim to calculated setpoints repeatedly.
   */
  public AutoAim(
    SwerveDriveSubsystem swerve,
    ShooterSubsystem shooter,
    ElevatorSubsystem elevator,
    LEDSubsystem leds,
    DoubleSupplier xSpeed,
    DoubleSupplier ySpeed
  ) {
    this(swerve, shooter, elevator, leds, xSpeed, ySpeed, swerve::speakerHeading, shooter::speakerAngle, elevator::speakerHeight, false);
  }

  /** 
   * Creates a new AutoAim2. <strong>(PRESET TELEOP)</strong>
   * 
   * This command will auto-aim to preset setpoints repeatedly.
   */
  public AutoAim(
    SwerveDriveSubsystem swerve,
    ShooterSubsystem shooter,
    ElevatorSubsystem elevator,
    LEDSubsystem leds,
    DoubleSupplier xSpeed,
    DoubleSupplier ySpeed,
    DoubleSupplier swerveHeading,
    DoubleSupplier shooterAngle,
    DoubleSupplier elevatorHeight
  ) {
    this(swerve, shooter, elevator, leds, xSpeed, ySpeed, swerveHeading, shooterAngle, elevatorHeight, false);
  }

  /** 
   * Creates a new AutoAim2. <strong>(CALCULATED AUTON)</strong>
   * 
   * This command will auto-aim to calculated setpoints once.
   */
  public AutoAim(
    SwerveDriveSubsystem swerve,
    ShooterSubsystem shooter,
    ElevatorSubsystem elevator,
    LEDSubsystem leds
  ) {
    this(swerve, shooter, elevator, leds, () -> 0, () -> 0, swerve::speakerHeading, shooter::speakerAngle, elevator::speakerHeight, true);
  }

  /** 
   * Creates a new AutoAim2. <strong>(PRESET AUTON)</strong>
   * 
   * This command will auto-aim to preset setpoints once.
   */
  public AutoAim(
    SwerveDriveSubsystem swerve,
    ShooterSubsystem shooter,
    ElevatorSubsystem elevator,
    LEDSubsystem leds,
    DoubleSupplier swerveHeading,
    DoubleSupplier shooterAngle,
    DoubleSupplier elevatorHeight
  ) {
    this(swerve, shooter, elevator, leds, () -> 0, () -> 0, swerveHeading, shooterAngle, elevatorHeight, true);
  }
}
