// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDStrip;

public class TestLED extends Command {
  private LEDStrip _leds;
  private int[] f = {0, 0, 0};
  private int[] s = {255, 17, 0};

  /** Creates a new TestLED. */
  public TestLED(LEDStrip leds) {
    // Use addRequirements() here to declare subsystem dependencies.
    _leds = leds;
    addRequirements(_leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _leds.rainbow();
    // _leds.blink(f, s, 25);
    // _leds.setColor(s);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
