/* Copyright (C) 2024 Team 334. All Rights Reserved.*/
package frc.robot.commands.leds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;

public class DefaultLED extends Command {
  private LEDSubsystem _leds;

  /** Creates a new TestLED. */
  public DefaultLED(LEDSubsystem leds) {
    // Use addRequirements() here to declare subsystem dependencies.
    _leds = leds;
    addRequirements(_leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // _leds.setColor(Constants.LEDColors.ALLIANCE_RGB);
    _leds.movingPixels(44); // TESTING ONLY!!!!!!!!
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
