/* Copyright (C) 2024 Team 334. All Rights Reserved.*/
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** @author Lucas Ou */
public class LEDSubsystem extends SubsystemBase {
  private AddressableLED _ledStrip;
  private AddressableLEDBuffer _ledBuffer;
  private int _ledNumber;

  private int _hue; // For rainbow
  private int _firstPixelHue; // For rainbow

  private int _value; // For moving pixel pattern.
  private int _firstPixelValue; // For moving pixel pattern.

  // Current counter will be how we manage time of our blinking pattern
  // 1 = 20ms if command is put in a periodic func.
  private int _currentCounter = 0;
  // colorOn used to control blinking.
  private boolean _colorOn = false;

  // For blinkingTimerTest
  private Timer _ledTimer = new Timer();

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem(int port, int ledNumber) {
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
      // Get the distance of the rainbow between two pixels. (180 /
      // _ledBuffer.getLength())
      // Times the index of current pixel. (i)
      // Plus the hue of the first pixel.
      // ^This will get us the "moved" hue for the current pixel^
      _hue = (_firstPixelHue + (i * (180 / _ledBuffer.getLength()))) % 180;
      _ledBuffer.setHSV(i, _hue, 255, 255);
    }
    _ledStrip.setData(_ledBuffer);

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
        _colorOn = true;
      } else {
        // If LEDs are on...
        // Set them off
        for (int i = 0; i < _ledBuffer.getLength(); i++) {
          _ledBuffer.setRGB(i, secondColor[0], secondColor[1], secondColor[2]);
        }
        _ledStrip.setData(_ledBuffer);
        _colorOn = false;
      }
      _currentCounter = 0;
    } else {
      ++_currentCounter;
    }
  }

  public void outwardPixels(int[] color, int speed, boolean isOut) {
    // IDEA: Pixels move outward beginning from middle.
  }

  public void movingPixels(int hueHSV) {
    // IDEA: Pixels move right to left or left to right.
    for (int i = 0; i < _ledBuffer.getLength(); i++) {
      if ((i - _firstPixelHue) % 3 == 0) {
        _value = 85;
      } else if ((i - _firstPixelHue) % 3 == 1) {
        _value = 85;
      } else {
        _value = 255;
      }
      _ledBuffer.setHSV(i, hueHSV, 255, _value);
    }
    _ledStrip.setData(_ledBuffer);

    _firstPixelValue += 1;
    if (_firstPixelValue == 2) {
      _firstPixelValue = 0;
    }
  }

  // timeBetween will now be in seconds 
  public void blinkTimerTest(int[] firstColor, int[] secondColor, int timeBetween) {
    if (_ledTimer.get() > timeBetween) {
      if (!_colorOn) {
        // If LEDs are not on...
        // Set them on to given color
        for (int i = 0; i < _ledBuffer.getLength(); i++) {
          _ledBuffer.setRGB(i, firstColor[0], firstColor[1], firstColor[2]);
        }
        _ledStrip.setData(_ledBuffer);
        _colorOn = true;
        _ledTimer.reset();
        _ledTimer.start();
      } else {
        // If LEDs are on...
        // Set them off
        for (int i = 0; i < _ledBuffer.getLength(); i++) {
          _ledBuffer.setRGB(i, secondColor[0], secondColor[1], secondColor[2]);
        }
        _ledStrip.setData(_ledBuffer);
        _colorOn = false;
        _ledTimer.reset();
        _ledTimer.start();
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
