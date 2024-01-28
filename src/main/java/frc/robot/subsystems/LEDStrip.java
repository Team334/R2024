// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDStrip extends SubsystemBase {
  private AddressableLED _ledStrip;
  private AddressableLEDBuffer _ledBuffer;
  private int _ledNumber;

  // Current counter will be how we manage time of our blinking pattern
  private int _currentCounter = 0;
  // colorOn used to control blinking.
  private boolean _colorOn = false;
  
  /** Creates a new LEDStrip. */
  public LEDStrip(int port, int ledNumber) {
    _ledNumber = ledNumber;

    _ledStrip = new AddressableLED(port);
    _ledBuffer = new AddressableLEDBuffer(_ledNumber);
    _ledStrip.setLength(_ledBuffer.getLength());

    _ledStrip.setData(_ledBuffer);
    _ledStrip.start();
  }

  public void setColor(int[] color) {
    // For every pixel in RGB format!!!
    for (var i = 0; i < _ledBuffer.getLength(); i++) {
      _ledBuffer.setRGB(i, color[0], color[1], color[2]);
    }

    _ledStrip.setData(_ledBuffer);
  }

  public void rainbow() {
    for (var i = 0; i < _ledBuffer.getLength(); i++) {
      // TODO: Make rainbow w/ setRGB
    }
    // Make the rainbow "move"
  }

  public void blink(int[] color, int timeBetween) {
    if (_currentCounter > timeBetween) {
      if (!_colorOn) {
        // If LEDs are not on...
        // Set them on to given color
        for (var i = 0; i < _ledBuffer.getLength(); i++) {
          _ledBuffer.setRGB(i, color[0], color[1], color[2]);
        }
        _ledStrip.setData(_ledBuffer);
      } else {
        // If LEDs are on...
        // Set them off
        for (var i = 0; i < _ledBuffer.getLength(); i++) {
          _ledBuffer.setRGB(i, 0, 0, 0);
        }
        _ledStrip.setData(_ledBuffer);
      }
      _currentCounter = 0;
    } else {
      ++_currentCounter;
    }
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
