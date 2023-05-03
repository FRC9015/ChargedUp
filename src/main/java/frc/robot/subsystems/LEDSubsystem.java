// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Filesystem;
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
  private final int LED_LENGTH = 1728;

  public final Color QE_NAVYBLUE = new Color(17, 43, 60);
  public final Color QE_BLUE = new Color(32, 83, 117);
  public final Color QE_ORANGE = new Color(255, 126, 14);
  public final Color CONE = Color.kYellow;
  public final Color CUBE = Color.kPurple;

  private AddressableLED ledStrip;
  private AddressableLEDBuffer ledBuffer;

  private int rainbowFirstPixelHue;

  private FileReader fileReader;
  private List<AddressableLEDBuffer> ledBuffers;

  private int frame =0;

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
    OFF,
    FLASHINGRED,
    GREEN
  }
  LEDPreset chosenPreset = LEDPreset.FLASHINGRED;
  LEDeffect choseneffect = LEDeffect.DoubleColorWave;

  private LEDSubsystem() {

    File deploydirectory = Filesystem.getDeployDirectory();
    File roll = new File(deploydirectory, "roll.txt");
    try {
      fileReader = new FileReader(roll);
    } catch (FileNotFoundException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    
    BufferedReader bufferedReader = new BufferedReader(fileReader);
    String line;
    try{
      ledBuffers = new ArrayList<>();
      AddressableLEDBuffer currentBuffer = null;
      int currentIndex = 0;
      while ((line = bufferedReader.readLine()) != null) {
        if (line.startsWith("new")) {
          currentBuffer = new AddressableLEDBuffer(LED_LENGTH);
          ledBuffers.add(currentBuffer);
          currentIndex=0;
      }else {
        String[] rgb = line.split(",");
        int r = Integer.parseInt(rgb[0].trim());
        int g = Integer.parseInt(rgb[1].trim());
        int b = Integer.parseInt(rgb[2].trim());



        currentBuffer.setRGB(currentIndex, r, g, b);
        currentIndex++;
      }
    }} catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    System.out.println(ledBuffers.size());

  
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
        freq=2;
        speed=2;
        pulsetwoColor(new Color(20, 10, 255), QE_ORANGE, freq, speed);
        break;
      case OFF:
        setStaticColor(new Color(0, 0, 0));
        break;
      case RAINBOW:
       rainbow();
       break;
      case FLASHINGRED:
        pulseColor(new Color(255,0,0), 0, 255, 15);
        break;
      case GREEN:
        pulseColor(new Color(0,255,0), 1, 300, 5);
        break;
      
      default:
        break;

    }
    
    //ledStrip.setData(ledBuffers.get(frame));
    ledStrip.setData(ledBuffers.get(Math.round(frame/2)));

    System.out.println(frame);
    if (frame==ledBuffers.size()*2-4){
      frame=-1;
    }
    frame++;
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
            double brightness = 0.5 + 0.5 * Math.sin(2 * Math.PI * frequency * i / ledBuffer.getLength() + time*another);
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
