/* Copyright (C) 2024 Team 334. All Rights Reserved.*/
package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.Presets;
import frc.robot.commands.auto.AutoAim;
import frc.robot.commands.auto.AutonShoot;
import frc.robot.commands.auto.NoteAlign;
import frc.robot.commands.elevator.OperateElevator;
import frc.robot.commands.elevator.SetElevator;
import frc.robot.commands.intake.FeedActuate;
import frc.robot.commands.leds.DefaultLED;
import frc.robot.commands.shooter.OperateShooter;
import frc.robot.commands.shooter.SetShooter;
import frc.robot.commands.shooter.SpinShooter;
import frc.robot.commands.swerve.BrakeSwerve;
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
  private final ElevatorSubsystem _elevatorSubsystem = new ElevatorSubsystem();
  private final IntakeSubsystem _intakeSubsystem = new IntakeSubsystem();
  private final LEDSubsystem _ledSubsystem = new LEDSubsystem(Constants.Ports.LEDS, 35);
  private final ShooterSubsystem _shooterSubsystem = new ShooterSubsystem();

  // controllers (for driver and operator)
  private final CommandPS4Controller _driveController = new CommandPS4Controller(Constants.Ports.DRIVER_CONTROLLER);
  // private final CommandPS4Controller _operatorController = new CommandPS4Controller(Constants.Ports.OPERATOR_CONTROLLER);
  private final CommandPS5Controller _operatorController = new CommandPS5Controller(Constants.Ports.OPERATOR_CONTROLLER);


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
    // brings the actuator out, while intaking in
    NamedCommands.registerCommand("actuateOut", new FeedActuate(_intakeSubsystem, ActuatorState.OUT, FeedMode.INTAKE));

    // brings the actuator in, while intaking and revving up the shooter for a shot, this will take time and is done while driving
    NamedCommands.registerCommand("actuateIn", new FeedActuate(_intakeSubsystem, ActuatorState.STOWED, FeedMode.INTAKE).alongWith(
      new SpinShooter(_shooterSubsystem, ShooterState.SHOOT, true)
    ));

    // brings the actuator in immediately, this is fast and shooting can't be done immediately 
    NamedCommands.registerCommand("actuateInFast", new FeedActuate(_intakeSubsystem, ActuatorState.STOWED, FeedMode.NONE, true));

    // revs and intakes if necessary while aiming, then shoots
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
    _ledSubsystem.setDefaultCommand(new DefaultLED(
      _ledSubsystem,
      () -> _swerveSubsystem.atDesiredHeading() && _elevatorSubsystem.atDesiredHeight() && _shooterSubsystem.atDesiredAngle()
    ));

    configureBindings();

    UtilFuncs.ShotVector(_swerveSubsystem::shotVector);

    _autonChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("AUTON CHOOSER", _autonChooser);
  }

  // to configure button bindings
  private void configureBindings() {
    Command feedOut = new FeedActuate(_intakeSubsystem, ActuatorState.OUT, FeedMode.NONE, true).andThen(
      new FeedActuate(_intakeSubsystem, ActuatorState.OUT, FeedMode.OUTTAKE)
    );

    // operator bindings
    _operatorController.L1().whileTrue(new SpinShooter(_shooterSubsystem, ShooterState.SHOOT));
    _operatorController.L2().whileTrue(new SpinShooter(_shooterSubsystem, ShooterState.AMP));
    _operatorController.R2().whileTrue(new SpinShooter(_shooterSubsystem, ShooterState.INTAKE));

    _operatorController.square().whileTrue(new FeedActuate(_intakeSubsystem, ActuatorState.OUT, FeedMode.INTAKE));
    _operatorController.circle().whileTrue(feedOut);
    _operatorController.triangle().whileTrue(new FeedActuate(_intakeSubsystem, ActuatorState.STOWED, FeedMode.OUTTAKE));
    _operatorController.cross().whileTrue(new FeedActuate(_intakeSubsystem, ActuatorState.STOWED, FeedMode.INTAKE));

    _operatorController.R1().whileTrue(
      Commands.parallel(
        new SetShooter(_shooterSubsystem, () -> Presets.SHOOTER_AMP_HANDOFF),
        new SetElevator(_elevatorSubsystem, () -> Presets.ELEVATOR_AMP_HANDOFF)
      )
    );

    // driver bindings
    _driveController.L1().onTrue(Commands.runOnce(_swerveSubsystem::toggleSpeed, _swerveSubsystem));
    _driveController.R1().onTrue(Commands.runOnce(() -> _swerveSubsystem.fieldOriented = !_swerveSubsystem.fieldOriented, _swerveSubsystem));
    _driveController.cross().whileTrue(new BrakeSwerve(_swerveSubsystem, _ledSubsystem));

    // TESTING ONLY!!!
    _driveController.triangle().onTrue(Commands.runOnce(() -> _swerveSubsystem.resetGyro(180), _swerveSubsystem));
    
    // TESTING ONLY!!!
    _driveController.circle().onTrue(Commands.runOnce(() -> {
      Optional<double[]> pose = _visionSubsystem.getBotposeBlue();
      
      if (pose.isPresent()) {
        Pose2d botpose = UtilFuncs.ToPose(pose.get());
        _swerveSubsystem.resetPose(new Pose2d(botpose.getX(), botpose.getY(), _swerveSubsystem.getHeading()));
      }
    }, _swerveSubsystem));

    _driveController.options().whileTrue(new NoteAlign(
      _swerveSubsystem,
      _visionSubsystem,
      () -> MathUtil.applyDeadband(-_driveFilterLeftY.calculate(_driveController.getLeftY()), 0.05),
      () -> MathUtil.applyDeadband(-_driveFilterLeftX.calculate(_driveController.getLeftX()), 0.05)
    ));

    _driveController.L2().whileTrue(
      new AutoAim(
        _swerveSubsystem,
        _shooterSubsystem, 
        _elevatorSubsystem,
        _ledSubsystem,
        () -> (UtilFuncs.GetAlliance() == Alliance.Red) ? 0 : 180,
        () -> Presets.CLOSE_SHOOTER_ANGLE, 
        () -> Presets.CLOSE_ELEVATOR_HEIGHT
      )
    );

    _driveController.R2().whileTrue(
      new AutoAim(
        _swerveSubsystem,
        _shooterSubsystem,
        _elevatorSubsystem,
        _ledSubsystem,
        () -> MathUtil.applyDeadband(-_driveFilterLeftY.calculate(_driveController.getLeftY()), 0.05),
        () -> MathUtil.applyDeadband(-_driveFilterLeftX.calculate(_driveController.getLeftX()), 0.05)
      )
    );
  }

  /**
   * Updates all subsystems on teleop init.
   */
  public void teleopInit() {
    _swerveSubsystem.fieldOriented = true;
    _shooterSubsystem.setShooterState(ShooterState.IDLE);
  }

  /** @return The Command to schedule for auton. */
  public Command getAutonCommand() {
    AutonShoot.reset();
    _swerveSubsystem.fieldOriented = false; // make sure swerve is robot-relative for pathplanner to work
    _shooterSubsystem.setShooterState(ShooterState.SHOOT);

    return _autonChooser.getSelected();
    // return null;
  }
}
