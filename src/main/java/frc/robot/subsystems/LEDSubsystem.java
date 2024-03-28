/* Copyright (C) 2024 Team 334. All Rights Reserved.*/
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** @author Lucas Ou */
public class LEDSubsystem extends SubsystemBase {
  private AddressableLED _ledStrip;
  private AddressableLEDBuffer _ledBuffer;
  private int _ledNumber;

  private int _hue; // For rainbow
  private int _firstPixelHue; // For rainbow

  private int _value; // For moving pixel pattern.
  private int _firstPixelIndex; // For moving pixel pattern.

  // colorOn used to control blinking status.
  private boolean _colorOn = false;

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

  /**
   * Sets color of LED strip.
   * 
   * @param color List of RGB values for color.
   */
  public void setColor(int[] color) {
    // For every pixel in RGB format!!!
    for (int i = 0; i < _ledBuffer.getLength(); i++) {
      // System.out.println("COLOR SETTING");
      _ledBuffer.setRGB(i, color[0], color[1], color[2]);
    }

    _ledStrip.setData(_ledBuffer);
  }

  // timeBetween will now be in seconds
  /**
   * Blinks a color on LED strip.
   * 
   * @param firstColor First alternating color in RGB.
   * @param secondColor Second alternating color in RGB.
   * @param timeBetween Time between each blink (in seconds).
   */
  public void blink(int[] firstColor, int[] secondColor, double timeBetween) {
    _ledTimer.start();
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
    }
  }

  /**
   * A rainbow pattern for LEDs.
   */
  public void rainbow() {
    _ledTimer.start();
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

    if (_ledTimer.get() >= 0.02) {
      _firstPixelHue += 3;
      _firstPixelHue %= 180;
      _ledTimer.reset();
      _ledTimer.start();
    }
  }

  /**
   * Several lights traveling along the LED strip.
   * 
   * @param hueHSV Color of moving light in HSV (hue).
   * @param speed Seed of which the light is moving.
   */
  public void movingPixels(int hueHSV, double speed) {
    _ledTimer.start();

    for (int i = 0; i < _ledBuffer.getLength(); i+=20) {
      for (int x = 0; x < 6; x++) {
        _ledBuffer.setHSV(x+i, hueHSV, 255, 255);
      }
      for (int x=6; x < 16; x++) {
        _ledBuffer.setHSV(x, 0, 0, 0);
      }
    }
    
    _ledStrip.setData(_ledBuffer);
    if (_ledTimer.get() >= speed) {
      _firstPixelIndex += 1;
      _ledTimer.reset();
      _ledTimer.start();
    }

    if (_firstPixelIndex == 3) {
      _firstPixelIndex = 0;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
