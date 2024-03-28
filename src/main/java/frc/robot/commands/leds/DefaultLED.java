/* Copyright (C) 2024 Team 334. All Rights Reserved.*/
package frc.robot.commands.leds;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.LEDColors;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.utils.UtilFuncs;

/**
 * @author Lucas Ou
 */
public class DefaultLED extends Command {
  private final LEDSubsystem _leds;

  private final BooleanSupplier _isAimed;

  /** Creates a new TestLED. */
  public DefaultLED(
    LEDSubsystem leds,
    BooleanSupplier isAimed
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    _leds = leds;
    _isAimed = isAimed;

    addRequirements(_leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (UtilFuncs.GetLEDs()) {
      case DEFAULT:
        _leds.setColor(UtilFuncs.GetAlliance() == Alliance.Red ? LEDColors.RED : LEDColors.BLUE);
        break;
    
      case AIM:
        if (!_isAimed.getAsBoolean()) {
          _leds.blink(Constants.LEDColors.YELLOW, Constants.LEDColors.NOTHING, 1);
        } else {
          _leds.setColor(Constants.LEDColors.GREEN);
        }
        break;

      default:
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
