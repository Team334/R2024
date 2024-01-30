// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// https://www.chiefdelphi.com/t/programming-leds-with-roborio-but-leds-stuck/377176/8 ?
public class LEDStrip extends SubsystemBase {
  private AddressableLED _ledStrip;
  private AddressableLEDBuffer _ledBuffer;
  private int _ledNumber;

  private int _hue; // For rainbow
  private int _firstPixelHue; // For rainbow

  // Current counter will be how we manage time of our blinking pattern
  // 1 = 20ms if command is put in a periodic func.
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
    for (int i = 0; i < _ledBuffer.getLength(); i++) {
      _ledBuffer.setRGB(i, color[0], color[1], color[2]);
    }

    _ledStrip.setData(_ledBuffer);
  }

  public void rainbow() {
    for (int i = 0; i < _ledBuffer.getLength(); i++) {
      // Get the distance of the rainbow between two pixels. (180 / _ledBuffer.getLength())
      // Times the index of current pixel. (i)
      // Plus the hue of the first pixel.
      // ^This will get us the "moved" hue for the current pixel^
      _hue = (_firstPixelHue + (i * (180 / _ledBuffer.getLength()))) % 180;
      _ledBuffer.setHSV(i, _hue, 255, 255);
    }
    
    _firstPixelHue += 3;
    _firstPixelHue %= 180;
  }

  public void blink(int[] firstColor, int[] secondColor, int timeBetween) {
    if (_currentCounter > timeBetween) {
      if (!_colorOn) {
        // If LEDs are not on...
        // Set them on to given color
        for (int i = 0; i < _ledBuffer.getLength(); i++) {
          _ledBuffer.setRGB(i, firstColor[0], firstColor[1], firstColor[2]);
        }
        _ledStrip.setData(_ledBuffer);
      } else {
        // If LEDs are on...
        // Set them off
        for (int i = 0; i < _ledBuffer.getLength(); i++) {
          _ledBuffer.setRGB(i, secondColor[0], secondColor[1], secondColor[2]);
        }
        _ledStrip.setData(_ledBuffer);
      }
      _currentCounter = 0;
    } else {
      ++_currentCounter;
    }
  }

  public void onePixelTest() {
    _ledBuffer.setRGB(0, 252, 186, 3);
    _ledStrip.setData(_ledBuffer);
    _ledStrip.start();
  }

  public void colorTest(int[] color) {
    // For every pixel
    for (var i = 0; i < _ledBuffer.getLength(); i++) {
      _ledBuffer.setRGB(i, color[0], color[1], color[2]);
    }

    _ledStrip.setData(_ledBuffer);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
