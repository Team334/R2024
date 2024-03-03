/* Copyright (C) 2024 Team 334. All Rights Reserved.*/
package frc.robot;

import javax.print.attribute.standard.MediaSize.NA;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandStadiaController;
import frc.robot.commands.elevator.OperateElevator;
import frc.robot.commands.elevator.SetElevator;
import frc.robot.commands.intake.FeedActuate;
import frc.robot.commands.leds.DefaultLED;
import frc.robot.commands.shooter.AutoAim;
import frc.robot.commands.shooter.OperateShooter;
import frc.robot.commands.shooter.SetShooter;
import frc.robot.commands.shooter.SpinShooter;
import frc.robot.commands.swerve.BrakeSwerve;
import frc.robot.commands.swerve.PivotMotor;
import frc.robot.commands.swerve.ResetPose;
import frc.robot.commands.swerve.TeleopDrive;
import frc.robot.commands.swerve.ToggleSwerveOrient;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.ActuatorState;
import frc.robot.subsystems.IntakeSubsystem.FeedMode;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final VisionSubsystem _visionSubsystem = new VisionSubsystem();
  // private final SwerveDriveSubsystem _swerveSubsystem = new SwerveDriveSubsystem(_visionSubsystem);
  private final ShooterSubsystem _shooterSubsystem = new ShooterSubsystem();
  private final ElevatorSubsystem _elevatorSubsystem = new ElevatorSubsystem();
  private final IntakeSubsystem _intakeSubsystem = new IntakeSubsystem();
// private final LEDSubsystem _ledSubsystem = new LEDSubsystem(Constants.Ports.LEDS, 14);

  // controllers (for driver and operator)
  private final CommandPS4Controller _driveController = new CommandPS4Controller(Constants.Ports.DRIVER_CONTROLLER);
  // private final CommandPS4Controller _operatorController = new CommandPS4Controller(Constants.Ports.OPERATOR_CONTROLLER);
  private final CommandPS5Controller _operatorController = new CommandPS5Controller(Constants.Ports.OPERATOR_CONTROLLER);

  // private final Command

  // slew rate limiters applied to joysticks
  private final SlewRateLimiter _driveFilterLeftX = new SlewRateLimiter(4);
  private final SlewRateLimiter _driveFilterLeftY = new SlewRateLimiter(4);
  private final SlewRateLimiter _driveFilterRightX = new SlewRateLimiter(4);
  private final SlewRateLimiter _driveFilterRightY = new SlewRateLimiter(4);

  private final SlewRateLimiter _operatorFilterLeftY = new SlewRateLimiter(2);
  private final SlewRateLimiter _operatorFilterRightY = new SlewRateLimiter(4);

  // sendable chooser for auton commands
  // private final SendableChooser<Command> _autonChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    Command autonActuate = new FeedActuate(_intakeSubsystem, ActuatorState.OUT, FeedMode.INTAKE);

    // NamedCommands.registerCommand("printHello", new PrintCommand("AUTON HELLO"));
    // NamedCommands.registerCommand("waitCommand", new WaitCommand(3));
    // NamedCommands.registerCommand("interruptSwerve", new BrakeSwerve(_swerveSubsystem, _ledSubsystem));
    // NamedCommands.registerCommand("speakerAim",
    //     new AutoAim(_ledSubsystem, _shooterSubsystem, _visionSubsystem, _swerveSubsystem));

    // less complex auton actuate in case note-safety doesn't work
    NamedCommands.registerCommand("actuateOut", autonActuate);
    NamedCommands.registerCommand("actuateIn", Commands.runOnce(autonActuate::cancel));

    // Drive/Operate default commands


    // _swerveSubsystem.setDefaultCommand(new TeleopDrive(_swerveSubsystem,
    //     () -> MathUtil.applyDeadband(-_driveFilterLeftY.calculate(_driveController.getLeftY()), 0.1),
    //     () -> MathUtil.applyDeadband(-_driveFilterLeftX.calculate(_driveController.getLeftX()), 0.1),
    //     () -> MathUtil.applyDeadband(-_driveFilterRightX.calculate(_driveController.getRightX()), 0.1)));

    _shooterSubsystem.setDefaultCommand(new OperateShooter(
      _shooterSubsystem,
      () -> -MathUtil.applyDeadband(_operatorController.getLeftY(), 0.05)
    ));

    _elevatorSubsystem.setDefaultCommand(new OperateElevator(
      _elevatorSubsystem,
      () -> -_operatorFilterRightY.calculate(MathUtil.applyDeadband(_operatorController.getRightY(), 0.05))
    ));

    // Non drive/operate default commands
    // _intakeSubsystem.setDefaultCommand(new FeedActuate(_intakeSubsystem));

    // _ledSubsystem.setDefaultCommand(new DefaultLED(_ledSubsystem));

    // _elevatorSubsystem.setDefaultCommand(new HoldElevator(_elevatorSubsystem));
    // _shooterSubsystem.setDefaultCommand(new HoldShooter(_shooterSubsystem));
    
    // _shooterSubsystem.setDefaultCommand(new OperateShooter(
    //   _shooterSubsystem,
    //   () -> MathUtil.applyDeadband(_operatorController.getRightY(), 0.05)
    // ));

    _intakeSubsystem.setDefaultCommand(new FeedActuate(_intakeSubsystem, ActuatorState.STOWED, FeedMode.NONE));

    // configure trigger bindings
    configureBindings();

    // _autonChooser = AutoBuilder.buildAutoChooser();

    // SmartDashboard.putData("AUTON CHOOSER", _autonChooser);
  }

  // to configure button bindings
  private void configureBindings() {
    Command safeActuate = new SetShooter(
      _shooterSubsystem,
      () -> 0
    ).andThen(new FeedActuate(_intakeSubsystem, ActuatorState.OUT, FeedMode.NONE));

    safeActuate = new FeedActuate(_intakeSubsystem, ActuatorState.OUT, FeedMode.NONE);

    // _driveController.R1().onTrue(new ToggleSwerveOrient(_swerveSubsystem));
    // _driveController.square().onTrue(new ResetPose(_swerveSubsystem));
    // 
    // _driveController.cross().whileTrue(new BrakeSwerve(_swerveSubsystem, _ledSubsystem));
    // _driveController.L1()
    //     .whileTrue(new AutoAim(_shooterSubsystem, _ledSubsystem, _visionSubsystem, _swerveSubsystem,
    //         () -> MathUtil.applyDeadband(-_driveFilterLeftY.calculate(_driveController.getLeftY()), 0.1),
    //         () -> MathUtil.applyDeadband(-_driveFilterLeftX.calculate(_driveController.getLeftX()), 0.1)));

    // _driveController.L2()
    //     .whileTrue(new PivotMotor(_ledSubsystem, _swerveSubsystem, true, () -> -_driveController.getLeftY()));

    // _driveController.R2()
    //     .whileTrue(new PivotMotor(_ledSubsystem, _swerveSubsystem, false, () -> -_driveController.getLeftY()));

    // _operatorController.circle().whileTrue();

    // _operatorController.triangle().whileTrue(new FeedIntake(_intakeSubsystem, ActuatorState.STOWED, Fe                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           edMode.NONE));
    // _operatorController.square().whileTrue(new FeedIntake(_intakeSubsystem, ActuatorState.OUT, FeedMode.NONE));

    // _operatorController.L1().whileTrue(
    //   Commands.run(() -> _elevatorSubsystem.driveElevator(-0.5), _elevatorSubsystem).handleInterrupt(() -> _elevatorSubsystem.stopElevator())
    // );  
    // _operatorController.R1().whileTrue(
    //   Commands.run(() -> _elevatorSubsystem.driveElevator(0.5), _elevatorSubsystem).handleInterrupt(() -> _elevatorSubsystem.stopElevator())
    // );

    _operatorController.L1().whileTrue(new SpinShooter(_shooterSubsystem, ShooterState.SHOOT));
    _operatorController.L2().whileTrue(new SpinShooter(_shooterSubsystem, ShooterState.AMP));

    // _operatorController.triangle().whileTrue(new FeedActuate(_intakeSubsystem, ActuatorState.STOWED, FeedMode.OUTTAKE));
    // _operatorController.square().whileTrue(new FeedActuate(_intakeSubsystem, ActuatorState.OUT, FeedMode.INTAKE));

    _operatorController.circle().whileTrue(new FeedActuate(_intakeSubsystem, ActuatorState.NONE, FeedMode.INTAKE));
    _operatorController.cross().whileTrue(new FeedActuate(_intakeSubsystem, ActuatorState.NONE, FeedMode.NONE));
    // _operatorController.square().whileTrue(
    //   Commands.run(() -> _intakeSubsystem.feed(FeedMode.INTAKE), _intakeSubsystem).handleInterrupt(() -> _intakeSubsystem.feed(FeedMode.NONE))
    // );
    _operatorController.square().whileTrue(safeActuate);
    _operatorController.triangle().whileTrue(new FeedActuate(_intakeSubsystem, ActuatorState.STOWED, FeedMode.OUTTAKE));


    _operatorController.R1().whileTrue(new SetShooter(_shooterSubsystem, () -> 38));
    // _operatorController.R2().whileTrue(new SetElevator(_elevatorSubsystem, () -> 45));

    // _operatorController.L1().whileTrue(
    //   new FeedActuate(_intakeSubsystem, ActuatorState.OUT, FeedMode.INTAKE).alongWith(
    //     new SetShooter(_shooterSubsystem, () -> 45)
    //   ).alongWith(new SetElevator(_elevatorSubsystem))
    // );


    // _operatorController.circle().whileTrue(new FeedIntake(_intakeSubsystem, ActuatorState.NONE, FeedMode.INTAKE));
    // _operatorController.cross().whileTrue(new FeedIntake(_intakeSubsystem, ActuatorState.NONE, FeedMode.OUTTAKE));

    // _operatorController.circle().whileTrue(new SetShooter(_shooterSubsystem, () -> 45));
    // _operatorController.circle().whileTrue(new );
  }

  /** @return The Command to schedule for auton. */
  public Command getAutonCommand() {
    // _swerveSubsystem.fieldOriented = false; // make sure swerve is robot-relative for pathplanner to work

    // return _autonChooser.getSelected();
    return null;
  }
}
