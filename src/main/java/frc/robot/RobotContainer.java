/* Copyright (C) 2024 Team 334. All Rights Reserved.*/
package frc.robot;

import javax.print.attribute.standard.MediaSize.NA;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.CANSparkBase.FaultID;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandStadiaController;
import frc.robot.Constants.Presets;
import frc.robot.commands.elevator.OperateElevator;
import frc.robot.commands.elevator.SetElevator;
import frc.robot.commands.intake.FeedActuate;
import frc.robot.commands.leds.DefaultLED;
import frc.robot.commands.shooter.AutoAim;
import frc.robot.commands.shooter.AutonShoot;
import frc.robot.commands.shooter.OperateShooter;
import frc.robot.commands.shooter.SetShooter;
import frc.robot.commands.shooter.SpinShooter;
import frc.robot.commands.swerve.BrakeSwerve;
import frc.robot.commands.swerve.PivotMotor;
import frc.robot.commands.swerve.TeleopDrive;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.ActuatorState;
import frc.robot.subsystems.IntakeSubsystem.FeedMode;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;
import frc.robot.utils.UtilFuncs;
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
  private final SwerveDriveSubsystem _swerveSubsystem = new SwerveDriveSubsystem(_visionSubsystem);
  private final ShooterSubsystem _shooterSubsystem = new ShooterSubsystem();
  private final ElevatorSubsystem _elevatorSubsystem = new ElevatorSubsystem();
  private final IntakeSubsystem _intakeSubsystem = new IntakeSubsystem();
  private final LEDSubsystem _ledSubsystem = new LEDSubsystem(Constants.Ports.LEDS, 20);

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

  private final SlewRateLimiter _operatorFilterLeftY = new SlewRateLimiter(4);
  private final SlewRateLimiter _operatorFilterRightY = new SlewRateLimiter(4);

  // sendable chooser for auton commands
  private final SendableChooser<Command> _autonChooser;


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    NamedCommands.registerCommand("brake", new BrakeSwerve(_swerveSubsystem, _ledSubsystem));
    
    NamedCommands.registerCommand("actuateOut", new SetShooter(_shooterSubsystem, () -> Presets.ACTUATE_SHOOTER_ANGLE).andThen(
      new FeedActuate(_intakeSubsystem, ActuatorState.OUT, FeedMode.INTAKE)
    ));

    NamedCommands.registerCommand("actuateIn", new FeedActuate(_intakeSubsystem, ActuatorState.STOWED, FeedMode.INTAKE).alongWith(
      new SpinShooter(_shooterSubsystem, ShooterState.SHOOT, true)
    ));

    NamedCommands.registerCommand("shoot", new AutonShoot(_shooterSubsystem, _elevatorSubsystem, _ledSubsystem, _swerveSubsystem, _intakeSubsystem));

    // Drive/Operate default commands
    _swerveSubsystem.setDefaultCommand(new TeleopDrive(_swerveSubsystem,
        () -> MathUtil.applyDeadband(-_driveFilterLeftY.calculate(_driveController.getLeftY()), 0.05),
        () -> MathUtil.applyDeadband(-_driveFilterLeftX.calculate(_driveController.getLeftX()), 0.05),
        () -> MathUtil.applyDeadband(-_driveFilterRightX.calculate(_driveController.getRightX()), 0.05)));

    _shooterSubsystem.setDefaultCommand(new OperateShooter(
      _shooterSubsystem,
      () -> -MathUtil.applyDeadband(_operatorController.getRightY(), 0.05)
    ));

    _elevatorSubsystem.setDefaultCommand(new OperateElevator(
      _elevatorSubsystem,
      () -> -_operatorFilterLeftY.calculate(MathUtil.applyDeadband(_operatorController.getLeftY(), 0.05))
    ));

    // Non drive/operate default commands
    _intakeSubsystem.setDefaultCommand(new FeedActuate(_intakeSubsystem, ActuatorState.STOWED, FeedMode.NONE));
    _ledSubsystem.setDefaultCommand(new DefaultLED(_ledSubsystem));

    configureBindings();

    UtilFuncs.ShootFast(_swerveSubsystem::speakerDistance);

    _autonChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("AUTON CHOOSER", _autonChooser);
  }

  // to configure button bindings
  private void configureBindings() {
    Command safeFeedIn = new SetShooter(_shooterSubsystem, () -> Presets.ACTUATE_SHOOTER_ANGLE).andThen(
      new FeedActuate(_intakeSubsystem, ActuatorState.OUT, FeedMode.INTAKE)
    );

    Command feedOut = new FeedActuate(_intakeSubsystem, ActuatorState.OUT).onlyWhile(() -> !_intakeSubsystem.atDesiredActuatorState()).andThen(
      new FeedActuate(_intakeSubsystem, ActuatorState.OUT, FeedMode.OUTTAKE)
    );

    Runnable stopShooter = () -> _shooterSubsystem.stopShooter();

    // operator bindings
    _operatorController.L1().whileTrue(new SpinShooter(_shooterSubsystem, ShooterState.SHOOT).handleInterrupt(stopShooter));
    _operatorController.L2().whileTrue(new SpinShooter(_shooterSubsystem, ShooterState.AMP).handleInterrupt(stopShooter));

    _operatorController.square().whileTrue(safeFeedIn);
    _operatorController.circle().whileTrue(feedOut);
    _operatorController.triangle().whileTrue(new FeedActuate(_intakeSubsystem, ActuatorState.STOWED, FeedMode.OUTTAKE));
    _operatorController.cross().whileTrue(new FeedActuate(_intakeSubsystem, FeedMode.INTAKE));

    // _operatorController.R2().whileTrue(
    //   Commands.runOnce(_intakeSubsystem::disableReverseSoftLimit, _intakeSubsystem).andThen(
    //     Commands.run(() -> _intakeSubsystem.actuate(-0.3), _intakeSubsystem).handleInterrupt(() -> _intakeSubsystem.actuate(0))
    // ));

    // _operatorController.R2().onTrue(
    //   Commands.runOnce(_intakeSubsystem::toggleReverseSoftLimit, _intakeSubsystem)
    // );

    // _operatorController.R1().onTrue(
    //   Commands.run(() -> _intakeSubsystem.actuate(-0.1), _intakeSubsystem).handleInterrupt(
    //     _intakeSubsystem::resetActuatorEncoder
    //   )
    // );

    // _operatorController.R1().onTrue(
    //   Commands.runOnce(_intakeSubsystem::resetReverseSoftLimit, _intakeSubsystem)
    // );

    // driver bindings
    _driveController.R1().onTrue(Commands.runOnce(() -> _swerveSubsystem.fieldOriented = !_swerveSubsystem.fieldOriented, _swerveSubsystem));
    _driveController.L1().onTrue(Commands.runOnce(() -> _swerveSubsystem.resetPose(new Pose2d()), _swerveSubsystem));
    _driveController.cross().whileTrue(new BrakeSwerve(_swerveSubsystem, _ledSubsystem));
    _driveController.L2().onTrue(Commands.runOnce(_swerveSubsystem::toggleSpeed, _swerveSubsystem));
    
    _driveController.R2().whileTrue(
      new AutoAim(
        _shooterSubsystem,
        _elevatorSubsystem,
        null,
        _swerveSubsystem,
        () -> MathUtil.applyDeadband(-_driveFilterLeftY.calculate(_driveController.getLeftY()), 0.05),
        () -> MathUtil.applyDeadband(-_driveFilterLeftX.calculate(_driveController.getLeftX()), 0.05)
      )
    );
  }

  /** @return The Command to schedule for auton. */
  public Command getAutonCommand() {
    _swerveSubsystem.fieldOriented = false; // make sure swerve is robot-relative for pathplanner to work

    return _autonChooser.getSelected();
    // return null;
  }
}
