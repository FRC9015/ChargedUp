// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  private static LEDSubsystem INSTANCE;

  public static LEDSubsystem getInstance() {
    if (INSTANCE == null)
      INSTANCE = new LEDSubsystem();
    return INSTANCE;
  }

  private final int LED_PORT = 9;
  private final int LED_LENGTH = 139;

  public final Color QE_NAVYBLUE = new Color(17, 43, 60);
  public final Color QE_BLUE = new Color(32, 83, 117);
  public final Color QE_ORANGE = new Color(246, 107, 14);
  public final Color CONE = Color.kYellow;
  public final Color CUBE = Color.kPurple;

  private AddressableLED ledStrip;
  private AddressableLEDBuffer ledBuffer;

  private int rainbowFirstPixelHue;

  private double freq=10, amp=1, speed=10;
  //private Color color1=new Color(32, 83, 250), color2=new Color(246, 107, 14);

  public enum LEDeffect{
    SingleColorWave,
    DoubleColorWave,
    SingleColorPulse,
    Rainbow,
    off
  }
  public enum LEDPreset{
    CONE,
    CUBE,
    RAINBOW,
    LOGOSLOW,
    LOGOFAST,
    OFF
  }
  LEDPreset chosenPreset = LEDPreset.LOGOSLOW;
  LEDeffect choseneffect = LEDeffect.DoubleColorWave;

  private LEDSubsystem() {

    SmartDashboard.putNumber("LED R", 0);
    SmartDashboard.putNumber("LED G", 0);
    SmartDashboard.putNumber("LED B", 0);
    SmartDashboard.putNumber("freq", 0);
    SmartDashboard.putNumber("amp", 0);
    SmartDashboard.putNumber("another", 0);



    ledStrip = new AddressableLED(LED_PORT);
    ledStrip.setLength(LED_LENGTH);

    ledBuffer = new AddressableLEDBuffer(LED_LENGTH);

    ledStrip.setData(ledBuffer);
    ledStrip.start();
  }

  @Override
  public void periodic() {
    // Read the current slider values and calculate the LED color
    double red = SmartDashboard.getNumber("LED R", 0);
    double green = SmartDashboard.getNumber("LED G", 0);
    double blue = SmartDashboard.getNumber("LED B", 0);
    //double freq = SmartDashboard.getNumber("freq", 0);
    //double amp = SmartDashboard.getNumber("amp", 0);
    //double speed = SmartDashboard.getNumber("another", 0);

    //Color color = new Color(red, green, blue);

    // Update the LED strip with the new color
    //Color mycolor = new Color(MathUtil.clamp((int) red, 0, 255), MathUtil.clamp((int) green, 0, 255), MathUtil.clamp((int) blue, 0, 255));
    //pulseColorSolid(mycolor,freq,amp,speed);

    //pulsetwoColor(new Color(255,0,0), new Color(0,0,255), freq, speed);
    //pulsetwoColor(new Color(246,107,14), new Color(10,10,250), freq, speed);
    switch (chosenPreset){
      case CONE:
        freq=2;
        amp=255;
        speed=15;
        pulseColor(new Color(255, 100, 0), freq,amp, speed);
        break;
      case CUBE:
        freq=2;
        amp=255;
        speed=15;
        pulseColor(new Color(20, 10, 255), freq,amp, speed);
        break;
      case LOGOFAST:

        pulsetwoColor(new Color(32, 83, 250), new Color(246, 107, 14), 10, 30);
        break;
      case LOGOSLOW:
        freq=5;
        speed=5;
        pulsetwoColor(new Color(32, 83, 250), new Color(246, 107, 14), freq, speed);
        break;
      case OFF:
        setStaticColor(new Color(0, 0, 0));
        break;
      case RAINBOW:
       rainbow();
      default:
        break;

    }
    
    ledStrip.setData(ledBuffer);
  }

  public void setEffect(LEDeffect eff, Color mColor1, Color mColor2,double mfreq, double mamp, double mspeed){
    choseneffect=eff;
    //color1 = mColor1;
    //color2 = mColor2;
    freq=mfreq;
    amp = mamp;
    speed = mspeed;

  }

  public void setPreset(LEDPreset preset){
    chosenPreset = preset;
  }


  /* ---------- BASE LED METHODS ---------- */

  /**
   * Set a single static color across the LED strip
   * 
   * @param color The color to set
   */
  public void setStaticColor(Color color) {
    synchronized (ledBuffer) {
      for (int i = 0; i < ledBuffer.getLength(); i++) {
        ledBuffer.setLED(i, color);
      }
    }
  }
  
  
  public void pulseColor(Color color, double frequency, double amplitude,double another) {
    synchronized (ledBuffer) {
        double time = System.currentTimeMillis() / 1000.0; // current time in seconds
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            double brightness = 0.7 + 0.4 * Math.sin(2 * Math.PI * frequency * i / ledBuffer.getLength() + time*another);
            brightness *= amplitude;
            int red = (int) (color.red* brightness);
            int green = (int) (color.green * brightness);
            int blue = (int) (color.blue * brightness);
            ledBuffer.setLED(i, new Color(red, green, blue));
        }
    }
}
  
  public void pulseColorSolid(Color color, double frequency, double amplitude,double another) {
    synchronized (ledBuffer) {
      double time = System.currentTimeMillis() / 1000.0; // current time in seconds
      for (int i = 0; i < ledBuffer.getLength(); i++) {
          double brightness = 0.75 + 0.25 * Math.sin(2 * Math.PI * frequency * 1 / ledBuffer.getLength() + time*another);
          brightness *= amplitude;
          int red = (int) (color.red* brightness);
          int green = (int) (color.green * brightness);
          int blue = (int) (color.blue * brightness);
          ledBuffer.setLED(i, new Color(red, green, blue));
      }
  }
}
public void pulsetwoColor(Color color1, Color color2, double frequency, double another) {
  synchronized (ledBuffer) {
    double time = System.currentTimeMillis() / 1000.0; // current time in seconds
    for (int i = 0; i < ledBuffer.getLength(); i++) {
        double brightness = 0.5 + 0.5 * Math.sin(2 * Math.PI * frequency * i / ledBuffer.getLength() + time*another);
        //brightness *= amplitude;
        
        int red = (int) ((color1.red* brightness + color2.red* (1-brightness))*255);
        int green = (int) ((color1.green * brightness+color2.green* (1-brightness))*255);
        int blue = (int) ((color1.blue * brightness+color2.blue* (1-brightness))*255);

        ledBuffer.setLED(i, new Color(red, green, blue));
    }
}
}




  /**
   * Set a color within a range of LEDs
   * 
   * @param start first led to set
   * @param end   last led to set
   * @param color color to set
   */
  public void setStaticColorOnRange(int start, int end, Color color) {
    start = MathUtil.clamp(start, 0, ledBuffer.getLength());
    end = MathUtil.clamp(end, 0, ledBuffer.getLength());

    synchronized (ledBuffer) {
      for (int i = start; i < end; i++) {
        ledBuffer.setLED(i, color);
      }
    }
  }

  /* --------- BASIC COLORS --------- */

  public void red() {
    setStaticColor(Color.kRed);
  }

  public void red(int start, int end) {
    setStaticColorOnRange(start, end, Color.kRed);
  }

  public void blue() {
    setStaticColor(Color.kBlue);
  }

  public void blue(int start, int end) {
    setStaticColorOnRange(start, end, Color.kBlue);
  }

  /* -------- EFFECTS ---------- */

  public void rainbow() {
    synchronized (ledBuffer) {
      // For every pixel
      for (var i = 0; i < ledBuffer.getLength(); i++) {
        // Calculate the hue - hue is easier for rainbows because the color
        // shape is a circle so only one value needs to precess
        final var hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
        // Set the value
        ledBuffer.setHSV(i, hue, 255, 128);
      }
      // Increase by to make the rainbow "move"
      rainbowFirstPixelHue += 3;
      // Check bounds
      rainbowFirstPixelHue %= 180;
    }
  }
}
