package frc.robot.subsystems;

import java.util.function.Function;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Helpers;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class LEDSubsystem extends SubsystemBase {
  private static LEDSubsystem INSTANCE;

  public static LEDSubsystem getInstance() {
    if (INSTANCE == null)
      INSTANCE = new LEDSubsystem();
    return INSTANCE;
  }

  private final int LED_PORT = 0;
  private final int LED_LENGTH = 139;

  public final Color QE_NAVYBLUE = new Color(17, 43, 60);
  public final Color QE_BLUE = new Color(32, 83, 117);
  public final Color QE_ORANGE = new Color(255, 126, 14);
  public final Color CONE = Color.kYellow;
  public final Color CUBE = Color.kPurple;

  private AddressableLED ledStrip;
  private AddressableLEDBuffer ledBuffer;

  // Effect Variables
  private int rainbowFirstPixelHue;
  private int chaseOffset;
  private int scannerEyePosition = 0;
  private int scannerScanDirection = 1;

  private double freq = 10, amp = 1, speed = 10;
  // private Color color1=new Color(32, 83, 250), color2=new Color(246, 107, 14);

  public enum LEDeffect {
    SingleColorWave,
    DoubleColorWave,
    SingleColorPulse,
    Rainbow,
    off
  }

  public enum LEDPreset {
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
    // double red = SmartDashboard.getNumber("LED R", 0);
    // double green = SmartDashboard.getNumber("LED G", 0);
    // double blue = SmartDashboard.getNumber("LED B", 0);
    // double freq = SmartDashboard.getNumber("freq", 0);
    // double amp = SmartDashboard.getNumber("amp", 0);
    // double speed = SmartDashboard.getNumber("another", 0);

    // Update the LED strip with the new color
    // Color mycolor = new Color(MathUtil.clamp((int) red, 0, 255),
    // MathUtil.clamp((int) green, 0, 255),
    // MathUtil.clamp((int) blue, 0, 255));

    ledStrip.setData(ledBuffer);
  }

  double red = SmartDashboard.getNumber("LED R", 0);
  double green = SmartDashboard.getNumber("LED G", 0);
  double blue = SmartDashboard.getNumber("LED B", 0);
  // double freq = SmartDashboard.getNumber("freq", 0);
  // double amp = SmartDashboard.getNumber("amp", 0);
  // double speed = SmartDashboard.getNumber("another", 0);

  // Color color = new Color(red, green, blue);

  // Update the LED strip with the new color
  // Color mycolor = new Color(MathUtil.clamp((int) red, 0, 255),
  // MathUtil.clamp((int) green, 0, 255), MathUtil.clamp((int) blue, 0, 255));
  // pulseColorSolid(mycolor,freq,amp,speed);

  // pulsetwoColor(new Color(255,0,0), new Color(0,0,255), freq, speed);
  // pulsetwoColor(new Color(246,107,14), new Color(10,10,250), freq, speed);
  switch(chosenPreset){
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

  ledStrip.setData(ledBuffer);
  }

  public void setEffect(LEDeffect eff, Color mColor1, Color mColor2, double mfreq, double mamp, double mspeed) {
    choseneffect = eff;
    // color1 = mColor1;
    // color2 = mColor2;
    freq = mfreq;
    amp = mamp;
    speed = mspeed;

  }

  public void setPreset(LEDPreset preset) {
    chosenPreset = preset;
  }

  /* ---------- BASE LED METHODS ---------- */

  /**
   * Set a single static color across the LED strip
   * 
   * @param color The color to set
   */
  public void staticColor(Color color) {
    synchronized (ledBuffer) {
      for (int i = 0; i < ledBuffer.getLength(); i++) {
        ledBuffer.setLED(i, color);
      }
    }
  }

  public void pulsetwoColor(Color color1, Color color2, double frequency, double another) {
    synchronized (ledBuffer) {
      double time = System.currentTimeMillis() / 1000.0; // current time in seconds
      for (int i = 0; i < ledBuffer.getLength(); i++) {
        double brightness = 0.5 + 0.5 * Math.sin(2 * Math.PI * frequency * i / ledBuffer.getLength() + time * another);
        // brightness *= amplitude;

        int red = (int) ((color1.red * brightness + color2.red * (1 - brightness)) * 255);
        int green = (int) ((color1.green * brightness + color2.green * (1 - brightness)) * 255);
        int blue = (int) ((color1.blue * brightness + color2.blue * (1 - brightness)) * 255);

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
  public void staticColorInRange(int start, int end, Color color) {
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
    staticColor(Color.kRed);
  }

  public void red(int start, int end) {
    staticColorInRange(start, end, Color.kRed);
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
      for (int i = 0; i < ledBuffer.getLength(); i++) {
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

  public void pulseColor(Color color, double frequency, double amplitude, double another) {
    synchronized (ledBuffer) {
      double time = System.currentTimeMillis() / 1000.0; // current time in seconds
      for (int i = 0; i < ledBuffer.getLength(); i++) {
        double brightness = 0.5 + 0.5 * Math.sin(2 * Math.PI * frequency * i / ledBuffer.getLength() + time * another);
        brightness *= amplitude;
        int red = (int) (color.red * brightness);
        int green = (int) (color.green * brightness);
        int blue = (int) (color.blue * brightness);
        ledBuffer.setLED(i, new Color(red, green, blue));
      }
    }
  }

  public void pulseColorSolid(Color color, double frequency, double amplitude, double another) {
    synchronized (ledBuffer) {
      double time = System.currentTimeMillis() / 1000.0; // current time in seconds
      for (int i = 0; i < ledBuffer.getLength(); i++) {
        double brightness = 0.5 + 0.5 * Math.sin(2 * Math.PI * frequency / ledBuffer.getLength() + time * another);
        brightness *= amplitude;
        int red = (int) (color.red * brightness);
        int green = (int) (color.green * brightness);
        int blue = (int) (color.blue * brightness);
        ledBuffer.setLED(i, new Color(red, green, blue));
      }
    }
  }

  public void pulseTwoColor(Color color1, Color color2, double frequency, double another) {
    synchronized (ledBuffer) {
      double time = System.currentTimeMillis() / 1000.0; // current time in seconds
      for (int i = 0; i < ledBuffer.getLength(); i++) {
        double brightness = 0.5 + 0.5 * Math.sin(2 * Math.PI * frequency * i / ledBuffer.getLength() + time * another);
        // brightness *= amplitude;

        int red = (int) ((color1.red * brightness + color2.red * (1 - brightness)) * 255);
        int green = (int) ((color1.green * brightness + (color2.green * 1 - brightness)) * 255);
        int blue = (int) ((color1.blue * brightness + (color2.blue * 1 - brightness)) * 255);
        System.out.print(brightness);
        System.out.print("   ");
        System.out.print(color1.blue);
        System.out.print("   ");

        System.out.println(color2.blue);
        ledBuffer.setLED(i, new Color(red, green, blue));
      }
    }
  }

  /**
   * Color Chasing Pattern; segments of each color "chase" down the led strip
   * 
   * @param colors       array of colors to chase with
   * @param segmentWidth number of leds that should be filled with each color
   */
  public void chaseColors(Color[] colors, int segmentWidth) {
    int numberOfColors = colors.length;
    int effectiveIndex;
    int colorIndex;
    int bufferLength = ledBuffer.getLength();
    synchronized (ledBuffer) {
      for (int index = 0; index < bufferLength; index++) {
        effectiveIndex = (index + chaseOffset) % bufferLength;
        colorIndex = (index / segmentWidth) % numberOfColors;
        ledBuffer.setLED(effectiveIndex, colors[colorIndex]);
      }
    }

    chaseOffset = (chaseOffset + 1) % bufferLength;
  }

  /**
   * 
   * @param lowColor
   * @param highColor
   * @param intensity number in range 0 -> 1 where 0 is the low color and 1 is the
   *                  high color
   */
  public void intensityPattern(Color lowColor, Color highColor, double intensity) {
    intensity = MathUtil.clamp(intensity, 0, 1);
    double red = MathUtil.interpolate(lowColor.red, highColor.red, intensity);
    double green = MathUtil.interpolate(lowColor.green, highColor.green, intensity);
    double blue = MathUtil.interpolate(lowColor.blue, highColor.blue, intensity);
    synchronized (ledBuffer) {
      for (int index = 0; index < ledBuffer.getLength(); index++) {
        ledBuffer.setLED(index, new Color(red, green, blue));
      }
    }
  }

  /**
   * Color Scanning effect; the "eye" color scans back and forth. This is
   * different than "chase" because the eye fades in and out over it's length
   * 
   * @param background the color that fills the rest of the LED strip
   * @param eye        the color that scans back and forth
   * @param length     the length of the scanner
   */
  public void scanner(Color background, Color eye, int length) {
    int bufferLength = ledBuffer.getLength();
    double intensity;
    double red;
    double green;
    double blue;
    double distanceFromEye;

    synchronized (ledBuffer) {
      for (int index = 0; index < bufferLength; index++) {
        distanceFromEye = MathUtil.clamp(Math.abs(scannerEyePosition - index), 0, length);
        intensity = 1 - distanceFromEye / length;
        red = MathUtil.interpolate(background.red, eye.red, intensity);
        green = MathUtil.interpolate(background.green, eye.green, intensity);
        blue = MathUtil.interpolate(background.blue, eye.blue, intensity);

        ledBuffer.setLED(index, new Color(red, green, blue));
      }
    }

    if (scannerEyePosition == 0) {
      scannerScanDirection = 1;
    } else if (scannerEyePosition == bufferLength - 1) {
      scannerScanDirection = -1;
    }

    scannerEyePosition += scannerScanDirection;
  }

  /**
   * Pure color chaos. Random colors on each pixel.
   * 
   * @param firstTime
   */
  public void chaos(boolean firstTime) {
    // This function randomly regenerates a color
    Function<Color, Color> randomColorShift = (Color aColor) -> new Color(Helpers.randomShift(aColor.red),
        Helpers.randomShift(aColor.green), Helpers.randomShift(aColor.blue));

    synchronized (ledBuffer) {
      if (firstTime) {
        for (int index = 0; index < ledBuffer.getLength(); index++) {
          ledBuffer.setLED(index, new Color(Math.random(), Math.random(), Math.random()));
        }
        firstTime = false;
      }
      for (int index = 0; index < ledBuffer.getLength(); index++) {
        ledBuffer.setLED(index, randomColorShift.apply(ledBuffer.getLED(index)));
      }
    }
  }

  /**
   * Alternates colors in the given list as it goes down the line. This pattern
   * does not move.
   * 
   * @param colors list of colors to alternate between
   */
  public void alternateStatic(Color[] colors) {
    synchronized (ledBuffer) {
      for (int index = 0; index < ledBuffer.getLength(); index++) {
        ledBuffer.setLED(index, colors[index % colors.length]);
      }
    }
  }

  /* ---------- HIGH LEVEL METHODS ---------- */

  public void pulseTeamColors() {
    pulseTwoColor(Color.kBlue, QE_ORANGE, 0.5, 2);
  }
}
